#include "hypertron_ros2_bridge/astrall_sdk_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "hypertron_ros2_bridge/thread_safe_queue.hpp"

namespace hypertron_ros2_bridge {
namespace {

std::uint64_t to_nanoseconds(IMonotonicClock::time_point value) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          value.time_since_epoch())
          .count());
}

ControllerStatus controller_status(const SdkSnapshot& snapshot) {
  ControllerStatus status;
  status.sdk_linked = snapshot.sdk_linked;
  status.control_authority = snapshot.control_authority;
  status.sport_status = snapshot.sport_status;
  status.error_code = snapshot.error_code;
  return status;
}

}  // namespace

struct AgentRuntime::Impl {
  struct ModeRequest {
    std::uint32_t sequence{};
    std::uint16_t mode{};
    std::uint16_t expected_status{};
    std::string name;
  };

  Impl(AgentConfig config_value, IAstrallSdk& sdk_value,
       IByteStream& stream_value, IMonotonicClock& clock_value,
       AgentUdpSources udp_value)
      : config(std::move(config_value)),
        sdk(sdk_value),
        stream(stream_value),
        clock(clock_value),
        udp(udp_value),
        controller({config.command_deadman}, clock),
        control_output(config.output_queue_capacity, OverflowPolicy::RejectNew),
        state_output(config.output_queue_capacity, OverflowPolicy::RejectNew),
        odometry_output(config.output_queue_capacity, OverflowPolicy::DropOldest),
        sensor_output(config.output_queue_capacity, OverflowPolicy::DropOldest),
        camera_output(2U, OverflowPolicy::DropOldest),
        mode_requests(config.output_queue_capacity, OverflowPolicy::RejectNew),
        decoder(config.max_payload) {
    if (config.heartbeat_period.count() <= 0 ||
        config.motion_period.count() <= 0 || config.state_period.count() <= 0 ||
        config.application_timeout.count() <= 0 ||
        config.mode_timeout.count() <= 0 ||
        config.mode_poll_period.count() <= 0) {
      throw std::invalid_argument("agent periods must be positive");
    }
  }

  AgentConfig config;
  IAstrallSdk& sdk;
  IByteStream& stream;
  IMonotonicClock& clock;
  AgentUdpSources udp;
  RobotController controller;
  ThreadSafeQueue<Frame> control_output;
  ThreadSafeQueue<Frame> state_output;
  ThreadSafeQueue<Frame> odometry_output;
  ThreadSafeQueue<Frame> sensor_output;
  ThreadSafeQueue<Frame> camera_output;
  ThreadSafeQueue<ModeRequest> mode_requests;
  ProtocolHandler decoder;
  std::mutex sdk_mutex;
  std::atomic_bool stop{false};
  std::atomic_bool initialized{false};
  std::atomic_bool handshake_complete{false};
  std::atomic<std::uint32_t> negotiated_capabilities{0};
  std::atomic_bool application_timed_out{false};
  std::atomic<std::uint32_t> next_sequence{1};
  std::atomic<std::uint32_t> last_velocity_sequence{0};
  std::mutex ping_mutex;
  IMonotonicClock::time_point last_ping{};
  std::mutex error_mutex;
  std::string last_error;

  void set_last_error(std::string value) {
    std::lock_guard<std::mutex> lock(error_mutex);
    last_error = std::move(value);
  }

  std::string get_last_error() {
    std::lock_guard<std::mutex> lock(error_mutex);
    return last_error;
  }

  std::uint32_t runtime_capabilities() const noexcept {
    std::uint32_t capabilities =
        CapabilityImu | CapabilitySport | CapabilitySystemState;
    if (udp.odometry != nullptr) capabilities |= CapabilityOdometry;
    if (config.camera_enabled && udp.camera != nullptr) {
      capabilities |= CapabilityCamera;
    }
    return capabilities;
  }

  bool negotiated(std::uint32_t capability) const noexcept {
    return handshake_complete.load() &&
           (negotiated_capabilities.load() & capability) != 0U;
  }

  Frame make_frame(MessageType type, std::vector<std::uint8_t> payload = {}) {
    return {type, next_sequence.fetch_add(1), to_nanoseconds(clock.now()),
            std::move(payload)};
  }

  void enqueue_control(Frame frame) {
    if (!control_output.push(std::move(frame))) {
      set_last_error("control output queue saturated");
      stop.store(true);
    }
  }

  void enqueue_state(Frame frame) {
    if (!state_output.push(std::move(frame))) {
      set_last_error("state output queue saturated");
      stop.store(true);
    }
  }

  SdkCallbacks callbacks() {
    SdkCallbacks value;
    value.on_link = [this](bool linked, bool authority) {
      auto state = controller.status();
      state.sdk_linked = linked;
      state.control_authority = authority;
      controller.update_robot_state(state);
    };
    value.on_imu = [this](ImuPayload payload) {
      if (!negotiated(CapabilityImu)) return;
      try {
        sensor_output.push(make_frame(MessageType::Imu, encode_imu(payload)));
      } catch (const std::exception& error) {
        set_last_error(error.what());
      }
    };
    value.on_sport = [this](SportPayload payload) {
      if (!negotiated(CapabilitySport)) return;
      try {
        auto state = controller.status();
        payload.sport_status = state.sport_status;
        sensor_output.push(
            make_frame(MessageType::Sport, encode_sport(payload)));
      } catch (const std::exception& error) {
        set_last_error(error.what());
      }
    };
    return value;
  }

  bool initialize() {
    if (initialized.load()) {
      return true;
    }
    std::uint16_t result{};
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      result = sdk.init(callbacks(), config.init_timeout_ms);
    }
    if (result != kAstrallSuccess) {
      set_last_error("AstrallSdkInit failed: " + std::to_string(result));
      return false;
    }
    initialized.store(true);
    {
      const bool camera = config.camera_enabled && udp.camera != nullptr;
      const bool lidar = udp.lidar != nullptr || udp.odometry != nullptr;
      std::lock_guard<std::mutex> lock(sdk_mutex);
      const auto subscription = sdk.configure_udp_streams(
          camera, lidar, config.sdk_call_timeout_ms);
      if (subscription != kAstrallSuccess) {
        set_last_error("ASTRALL UDP subscription failed: " +
                       std::to_string(subscription));
        sdk.deinit();
        initialized.store(false);
        return false;
      }
    }
    if (config.auto_acquire_control) {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      const auto auth = sdk.acquire_sdk_control(config.sdk_call_timeout_ms);
      if (auth != kAstrallSuccess) {
        set_last_error("ASTRALL control request denied: " +
                       std::to_string(auth));
      }
    }
    refresh_snapshot(false);
    {
      std::lock_guard<std::mutex> lock(ping_mutex);
      last_ping = clock.now();
    }
    return true;
  }

  SdkSnapshot refresh_snapshot(bool publish) {
    SdkSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      snapshot = sdk.snapshot();
    }
    controller.update_robot_state(controller_status(snapshot));
    if (publish && negotiated(CapabilitySystemState)) {
      RobotStatePayload state;
      state.sdk_linked = snapshot.sdk_linked;
      state.control_authority = snapshot.control_authority;
      state.emergency_stop = controller.emergency_stop_latched();
      state.camera_available = config.camera_enabled && udp.camera != nullptr;
      state.odometry_scale_verified = config.odometry_scale_verified;
      state.system_status = snapshot.system_status;
      state.error_code = snapshot.error_code;
      state.warning_code = snapshot.warning_code;
      state.sport_status = snapshot.sport_status;
      state.battery_percentage = snapshot.battery_percentage;
      state.battery_temperature = snapshot.battery_temperature;
      state.battery_voltage = snapshot.battery_voltage;
      state.battery_cycle_count = snapshot.battery_cycle_count;
      state.charge_status = snapshot.charge_status;
      state.wheel_speed = snapshot.wheel_speed;
      state.last_velocity_sequence = last_velocity_sequence.load();
      state.last_error = get_last_error();
      try {
        enqueue_state(
            make_frame(MessageType::RobotState, encode_robot_state(state)));
      } catch (const std::exception& error) {
        set_last_error(error.what());
      }
    }
    return snapshot;
  }

  void send_error(std::uint32_t request, BridgeError code,
                  const std::string& text) {
    set_last_error(text);
    enqueue_control(make_frame(
        MessageType::Error, encode_error({request, code, text})));
  }

  void send_ack(std::uint32_t request, std::uint16_t result,
                const std::string& text) {
    enqueue_control(make_frame(MessageType::Ack,
                               encode_ack({request, result, text})));
  }

  void process(Frame frame) {
    if (frame.type == MessageType::Hello) {
      const auto hello = decode_hello(frame.payload);
      if (hello.min_version > kBridgeProtocolVersion ||
          hello.max_version < kBridgeProtocolVersion) {
        send_error(frame.sequence, BridgeError::Protocol,
                   "no compatible HTBR protocol version");
        return;
      }
      const auto capabilities = hello.capabilities & runtime_capabilities();
      // The JOINT capability deliberately remains clear until the vendor
      // publishes a low-level joint SDK and message semantics.
      enqueue_control(make_frame(
          MessageType::HelloAck,
          encode_hello_ack({kBridgeProtocolVersion, capabilities,
                            hello.instance_nonce, sdk.sdk_version()})));
      negotiated_capabilities.store(capabilities);
      handshake_complete.store(true);
      controller.set_negotiated_ready(true);
      return;
    }
    if (frame.type == MessageType::Ping || frame.type == MessageType::Pong) {
      {
        std::lock_guard<std::mutex> lock(ping_mutex);
        last_ping = clock.now();
      }
      application_timed_out.store(false);
      if (frame.type == MessageType::Ping) {
        enqueue_control(make_frame(MessageType::Pong));
      }
      return;
    }
    if (!handshake_complete.load() && frame.type != MessageType::CmdEstop) {
      send_error(frame.sequence, BridgeError::Protocol,
                 "HELLO handshake is required before commands");
      return;
    }
    if (frame.type == MessageType::CmdVelocity) {
      const auto decision = controller.accept_velocity(
          decode_velocity(frame.payload));
      if (decision.accepted) {
        last_velocity_sequence.store(frame.sequence);
      } else {
        send_error(frame.sequence, decision.error, decision.reason);
      }
      return;
    }
    if (frame.type == MessageType::CmdMode) {
      const auto requested = decode_mode(frame.payload);
      std::string_view mode_name;
      std::uint16_t expected_status{};
      switch (requested.mode) {
        case 0xA101U:
          mode_name = "damping";
          expected_status = 0xB101U;
          break;
        case 0xA102U:
          mode_name = "stand";
          expected_status = 0xB102U;
          break;
        case 0xA103U:
          mode_name = "down";
          expected_status = 0xB103U;
          break;
        case 0xA104U:
          mode_name = "move";
          expected_status = 0xB104U;
          break;
        case 0xA105U:
          mode_name = "auto_charge";
          expected_status = 0xB107U;
          break;
        case 0xA106U:
          mode_name = "exit_charge";
          expected_status = 0xB10BU;
          break;
        case 0xA1FFU:
          mode_name = "recover";
          expected_status = 0xB1FFU;
          break;
        default:
          send_error(frame.sequence, BridgeError::InvalidCommand,
                     "unknown ASTRALL sport mode code");
          return;
      }
      const auto decision = controller.request_mode(mode_name);
      if (!decision.accepted) {
        send_error(frame.sequence, decision.error, decision.reason);
        return;
      }
      if (!mode_requests.push({frame.sequence, decision.mode, expected_status,
                               std::string(mode_name)})) {
        controller.complete_mode_transition(false);
        send_error(frame.sequence, BridgeError::InvalidCommand,
                   "mode request queue is full");
      }
      return;
    }
    if (frame.type == MessageType::CmdEstop) {
      const auto requested = decode_estop(frame.payload);
      if (requested.engage) {
        controller.trigger_estop();
        std::uint16_t move_result{};
        std::uint16_t mode_result{};
        {
          std::lock_guard<std::mutex> lock(sdk_mutex);
          move_result = sdk.move(0, 0, 0, config.sdk_call_timeout_ms);
          mode_result =
              sdk.set_mode(kAstrallModeDamping, config.sdk_call_timeout_ms);
        }
        if (move_result != kAstrallSuccess || mode_result != kAstrallSuccess) {
          send_error(frame.sequence, BridgeError::SdkCallFailed,
                     "emergency stop SDK call failed");
        } else {
          // 轮询等待机器人实际进入阻尼稳态（sport_status == 0xB101U）。
          // 使用专用短超时（mode_timeout/5，≤500ms）以免阻塞 process() 读循环过久。
          using namespace std::chrono_literals;
          // estop 确认轮询使用专用短超时（mode_timeout/5，上限500ms）：
          // process() 是同步调用，长时间阻塞会导致 ping 无法处理、application_expired() 误触。
          const auto estop_confirm_timeout =
              std::min(config.mode_timeout / 5,
                       std::chrono::duration_cast<decltype(config.mode_timeout)>(500ms));
          const auto estop_deadline =
              std::chrono::steady_clock::now() + estop_confirm_timeout;
          bool damping_confirmed = false;
          while (std::chrono::steady_clock::now() < estop_deadline &&
                 !stop.load()) {
            const auto snap = refresh_snapshot(false);
            // 阻尼稳态的 sport_status 为 0xB101U（见 CmdMode 分支中
            // 0xA101U -> expected_status 0xB101U 的映射）；kAstrallModeDamping
            // (0xA101U) 是模式命令码，不能直接用于状态比较。
            if (snap.sport_status == 0xB101U) {
              damping_confirmed = true;
              break;
            }
            std::this_thread::sleep_for(config.mode_poll_period);
          }
          if (damping_confirmed) {
            send_ack(frame.sequence, kAstrallSuccess,
                     "software emergency stop latched; damping state confirmed");
          } else {
            // 请求已被 SDK 接受，但在超时内未观测到阻尼稳态；仍 ACK 但注明未确认。
            send_ack(frame.sequence, kAstrallSuccess,
                     "software emergency stop latched; damping requested "
                     "(state confirmation timed out — robot may still be "
                     "transitioning)");
          }
        }
      } else {
        controller.clear_estop();
        send_ack(frame.sequence, kAstrallSuccess,
                 "software latch cleared; mode and velocity remain stopped");
      }
      return;
    }
    if (frame.type == MessageType::CmdJoint) {
      // TODO:参考手册第23-26页补充实现：厂家低层SDK提供后实现关节控制和状态语义。
      controller.reject_joint_command("ASTRALL 1.0.7 has no joint API");
      send_error(frame.sequence, BridgeError::FeatureUnavailable,
                 "joint control is unavailable in ASTRALL SDK 1.0.7");
      return;
    }
    send_error(frame.sequence, BridgeError::Protocol,
               "message type is invalid in PC-to-agent direction");
  }

  void heartbeat_loop() {
    while (!stop.load()) {
      std::uint16_t result{};
      {
        std::lock_guard<std::mutex> lock(sdk_mutex);
        result = sdk.heartbeat(config.sdk_call_timeout_ms);
      }
      if (result != kAstrallSuccess) {
        set_last_error("AstrallHeartbeat failed: " + std::to_string(result));
      }
      std::this_thread::sleep_for(config.heartbeat_period);
    }
  }

  bool application_expired() {
    std::lock_guard<std::mutex> lock(ping_mutex);
    return clock.now() - last_ping > config.application_timeout;
  }

  void motion_loop() {
    while (!stop.load()) {
      if (application_expired()) {
        if (!application_timed_out.exchange(true)) {
          controller.trigger_estop();
          {
            std::lock_guard<std::mutex> lock(sdk_mutex);
            sdk.move(0, 0, 0, config.sdk_call_timeout_ms);
            sdk.set_mode(kAstrallModeDamping, config.sdk_call_timeout_ms);
          }
          set_last_error(
              "PC application heartbeat timed out; damping requested; "
              "agent shutting down");
          // PC 应用层超时后不应继续维持 SDK 心跳与控制权：
          // 置 stop=true 触发 run() 清理路径（join 所有线程 → safe_shutdown → deinit）。
          stop.store(true);
        }
      } else {
        const auto velocity = controller.velocity_for_tick().value;
        std::lock_guard<std::mutex> lock(sdk_mutex);
        const auto result = sdk.move(velocity.vx, velocity.vy, velocity.vyaw,
                                     config.sdk_call_timeout_ms);
        if (result != kAstrallSuccess) {
          set_last_error("AstrallMove failed: " + std::to_string(result));
        }
      }
      std::this_thread::sleep_for(config.motion_period);
    }
  }

  void state_loop() {
    while (!stop.load()) {
      refresh_snapshot(true);
      std::this_thread::sleep_for(config.state_period);
    }
  }

  void mode_loop() {
    using namespace std::chrono_literals;
    while (!stop.load()) {
      auto request = mode_requests.wait_pop_for(20ms);
      if (!request) {
        if (mode_requests.closed()) return;
        continue;
      }

      const auto status = controller.status();
      if (!status.sdk_linked || !status.control_authority ||
          status.error_code != 0U ||
          (controller.emergency_stop_latched() &&
           request->mode != kAstrallModeDamping)) {
        controller.complete_mode_transition(false);
        send_error(request->sequence, BridgeError::NoControlAuthority,
                   "mode transition lost its safety prerequisites");
        continue;
      }

      std::uint16_t zero_result{};
      std::uint16_t result{};
      {
        std::lock_guard<std::mutex> lock(sdk_mutex);
        zero_result = sdk.move(0, 0, 0, config.sdk_call_timeout_ms);
        if (zero_result == kAstrallSuccess) {
          result = sdk.set_mode(request->mode, config.sdk_call_timeout_ms);
        }
      }
      if (zero_result != kAstrallSuccess) {
        controller.complete_mode_transition(false);
        send_error(request->sequence, BridgeError::SdkCallFailed,
                   "zero velocity before mode transition failed: " +
                       std::to_string(zero_result));
        continue;
      }
      if (result != kAstrallSuccess) {
        controller.complete_mode_transition(false);
        send_error(request->sequence, BridgeError::SdkCallFailed,
                   "AstrallSportModeControl failed: " +
                       std::to_string(result));
        continue;
      }

      const auto deadline = std::chrono::steady_clock::now() +
                            config.mode_timeout;
      bool completed = false;
      while (!stop.load() && std::chrono::steady_clock::now() < deadline) {
        if (controller.emergency_stop_latched() &&
            request->mode != kAstrallModeDamping) {
          controller.complete_mode_transition(false);
          send_error(request->sequence, BridgeError::EmergencyStopLatched,
                     "mode transition interrupted by emergency stop");
          completed = true;
          break;
        }
        const auto snapshot = refresh_snapshot(false);
        if (!snapshot.sdk_linked || !snapshot.control_authority ||
            snapshot.error_code != 0U) {
          controller.complete_mode_transition(false);
          send_error(request->sequence, BridgeError::NoControlAuthority,
                     "mode transition lost SDK link, authority, or health");
          completed = true;
          break;
        }
        if (snapshot.sport_status == request->expected_status) {
          controller.complete_mode_transition(true);
          send_ack(request->sequence, result,
                   "mode reached stable ASTRALL sport state");
          completed = true;
          break;
        }
        std::this_thread::sleep_for(config.mode_poll_period);
      }
      if (!completed && !stop.load()) {
        controller.complete_mode_transition(false);
        send_error(request->sequence, BridgeError::Timeout,
                   "timed out waiting for ASTRALL sport state " +
                       std::to_string(request->expected_status));
      }
    }
  }

  void odometry_loop() {
    while (!stop.load() && udp.odometry != nullptr) {
      const auto datagram = udp.odometry->receive(std::chrono::milliseconds(50));
      if (!datagram) {
        continue;
      }
      if (!negotiated(CapabilityOdometry)) continue;
      const auto parsed = parse_odometry_packet(
          datagram->bytes, config.lidar_packing,
          config.odometry_position_scale, config.odometry_quaternion_scale);
      if (!parsed) {
        set_last_error("invalid UDP 6101 odometry datagram");
        continue;
      }
      OdometryPayload payload;
      payload.device_time = parsed->device_time;
      payload.position = parsed->position;
      payload.orientation = parsed->orientation;
      odometry_output.push(
          make_frame(MessageType::Odometry, encode_odometry(payload)));
    }
  }

  void lidar_loop() {
    while (!stop.load() && udp.lidar != nullptr) {
      const auto datagram = udp.lidar->receive(std::chrono::milliseconds(50));
      if (!handshake_complete.load()) continue;
      if (datagram && !parse_point_cloud_packet(
                          datagram->bytes, config.lidar_packing,
                          config.point_coordinate_scale)) {
        set_last_error("invalid UDP 6100 point-cloud datagram");
      }
    }
  }

  void camera_loop() {
    std::uint32_t datagram_sequence{};
    while (!stop.load() && config.camera_enabled && udp.camera != nullptr) {
      const auto datagram = udp.camera->receive(std::chrono::milliseconds(50));
      if (!datagram) {
        continue;
      }
      if (!negotiated(CapabilityCamera)) continue;
      // TODO:参考手册第24页补充实现：实机确认UDP 6000 H.264分片及关键帧边界。
      CameraChunkPayload payload;
      payload.stream_id = 0;
      payload.receive_time_ns = datagram->receive_time_ns;
      payload.datagram_sequence = datagram_sequence++;
      payload.data = datagram->bytes;
      camera_output.push(make_frame(MessageType::CameraH264,
                                    encode_camera_chunk(payload)));
    }
  }

  void writer_loop() {
    using namespace std::chrono_literals;
    while (true) {
      auto frame = control_output.try_pop();
      if (!frame) frame = state_output.try_pop();
      if (!frame) frame = odometry_output.try_pop();
      if (!frame) frame = sensor_output.try_pop();
      if (!frame) frame = camera_output.wait_pop_for(10ms);
      if (frame) {
        try {
          if (!stream.write_all(ProtocolHandler::encode(*frame))) {
            stop.store(true);
            stream.close();
            return;
          }
        } catch (const std::exception& error) {
          set_last_error(error.what());
          stop.store(true);
          stream.close();
          return;
        }
        continue;
      }
      if (control_output.closed() && state_output.closed() &&
          odometry_output.closed() && sensor_output.closed() &&
          camera_output.closed() && control_output.size() == 0U &&
          state_output.size() == 0U && odometry_output.size() == 0U &&
          sensor_output.size() == 0U && camera_output.size() == 0U) {
        return;
      }
    }
  }

  void close_udp() noexcept {
    if (udp.camera != nullptr) udp.camera->close();
    if (udp.lidar != nullptr) udp.lidar->close();
    if (udp.odometry != nullptr) udp.odometry->close();
  }

  void safe_shutdown() noexcept {
    if (!initialized.exchange(false)) {
      return;
    }
    try {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      sdk.move(0, 0, 0, config.sdk_call_timeout_ms);
      sdk.set_mode(kAstrallModeDamping, config.sdk_call_timeout_ms);
      sdk.deinit();
    } catch (...) {
    }
  }
};

AgentRuntime::AgentRuntime(AgentConfig config, IAstrallSdk& sdk,
                           IByteStream& stream, IMonotonicClock& clock,
                           AgentUdpSources udp)
    : impl_(std::make_unique<Impl>(std::move(config), sdk, stream, clock, udp)) {
}

AgentRuntime::~AgentRuntime() {
  request_stop();
  impl_->safe_shutdown();
}

int AgentRuntime::run(std::function<bool()> external_stop_requested) {
  if (!impl_->initialize()) {
    impl_->stream.close();
    return 2;
  }

  std::thread writer([this] { impl_->writer_loop(); });
  std::thread heartbeat([this] { impl_->heartbeat_loop(); });
  std::thread motion([this] { impl_->motion_loop(); });
  std::thread state([this] { impl_->state_loop(); });
  std::thread mode([this] { impl_->mode_loop(); });
  std::thread odometry;
  std::thread lidar;
  std::thread camera;
  if (impl_->udp.odometry != nullptr) {
    odometry = std::thread([this] { impl_->odometry_loop(); });
  }
  if (impl_->udp.lidar != nullptr) {
    lidar = std::thread([this] { impl_->lidar_loop(); });
  }
  if (impl_->config.camera_enabled && impl_->udp.camera != nullptr) {
    camera = std::thread([this] { impl_->camera_loop(); });
  }

  try {
    while (!impl_->stop.load()) {
      if (external_stop_requested && external_stop_requested()) {
        impl_->stop.store(true);
        break;
      }
      const auto bytes =
          impl_->stream.read_some(std::chrono::milliseconds(50));
      if (!bytes) break;
      if (bytes->empty()) continue;
      for (auto& frame : impl_->decoder.feed(*bytes)) {
        impl_->process(std::move(frame));
      }
    }
  } catch (const std::exception& error) {
    impl_->set_last_error(error.what());
  }

  impl_->stop.store(true);
  impl_->close_udp();
  impl_->mode_requests.close();
  if (heartbeat.joinable()) heartbeat.join();
  if (motion.joinable()) motion.join();
  if (state.joinable()) state.join();
  if (mode.joinable()) mode.join();
  if (odometry.joinable()) odometry.join();
  if (lidar.joinable()) lidar.join();
  if (camera.joinable()) camera.join();
  impl_->control_output.close();
  impl_->state_output.close();
  impl_->odometry_output.close();
  impl_->sensor_output.close();
  impl_->camera_output.close();
  if (writer.joinable()) writer.join();
  impl_->safe_shutdown();
  impl_->stream.close();
  return 0;
}

void AgentRuntime::request_stop() noexcept {
  impl_->stop.store(true);
  impl_->close_udp();
  impl_->mode_requests.close();
  impl_->control_output.close();
  impl_->state_output.close();
  impl_->odometry_output.close();
  impl_->sensor_output.close();
  impl_->camera_output.close();
  impl_->stream.close();
}

void AgentRuntime::step_for(std::chrono::milliseconds duration) {
  if (!impl_->initialize()) {
    return;
  }
  const auto ticks = duration / impl_->config.motion_period;
  auto heartbeat_elapsed = std::chrono::milliseconds::zero();
  for (std::int64_t tick = 0; tick < ticks; ++tick) {
    const auto velocity = impl_->controller.velocity_for_tick().value;
    {
      std::lock_guard<std::mutex> lock(impl_->sdk_mutex);
      impl_->sdk.move(velocity.vx, velocity.vy, velocity.vyaw,
                      impl_->config.sdk_call_timeout_ms);
    }
    heartbeat_elapsed += impl_->config.motion_period;
    while (heartbeat_elapsed >= impl_->config.heartbeat_period) {
      std::lock_guard<std::mutex> lock(impl_->sdk_mutex);
      impl_->sdk.heartbeat(impl_->config.sdk_call_timeout_ms);
      heartbeat_elapsed -= impl_->config.heartbeat_period;
    }
  }
}

}  // namespace hypertron_ros2_bridge
