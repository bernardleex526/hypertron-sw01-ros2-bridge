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
        telemetry_output(config.output_queue_capacity,
                         OverflowPolicy::DropOldest),
        decoder(config.max_payload) {
    if (config.heartbeat_period.count() <= 0 ||
        config.motion_period.count() <= 0 || config.state_period.count() <= 0 ||
        config.application_timeout.count() <= 0) {
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
  ThreadSafeQueue<Frame> telemetry_output;
  ProtocolHandler decoder;
  std::mutex sdk_mutex;
  std::atomic_bool stop{false};
  std::atomic_bool initialized{false};
  std::atomic_bool handshake_complete{false};
  std::atomic_bool application_timed_out{false};
  std::atomic<std::uint32_t> next_sequence{1};
  std::atomic<std::uint32_t> last_velocity_sequence{0};
  std::mutex ping_mutex;
  IMonotonicClock::time_point last_ping{};
  std::string last_error;

  Frame make_frame(MessageType type, std::vector<std::uint8_t> payload = {}) {
    return {type, next_sequence.fetch_add(1), to_nanoseconds(clock.now()),
            std::move(payload)};
  }

  void enqueue_control(Frame frame) {
    if (!control_output.push(std::move(frame))) {
      last_error = "control output queue saturated";
      stop.store(true);
    }
  }

  void enqueue_telemetry(Frame frame) {
    telemetry_output.push(std::move(frame));
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
      try {
        enqueue_telemetry(make_frame(MessageType::Imu, encode_imu(payload)));
      } catch (const std::exception& error) {
        last_error = error.what();
      }
    };
    value.on_sport = [this](SportPayload payload) {
      try {
        auto state = controller.status();
        payload.sport_status = state.sport_status;
        enqueue_telemetry(
            make_frame(MessageType::Sport, encode_sport(payload)));
      } catch (const std::exception& error) {
        last_error = error.what();
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
      last_error = "AstrallSdkInit failed: " + std::to_string(result);
      return false;
    }
    initialized.store(true);
    if (config.auto_acquire_control) {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      const auto auth = sdk.acquire_sdk_control(config.sdk_call_timeout_ms);
      if (auth != kAstrallSuccess) {
        last_error = "ASTRALL control request denied: " +
                     std::to_string(auth);
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
    if (publish) {
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
      state.last_error = last_error;
      try {
        enqueue_telemetry(
            make_frame(MessageType::RobotState, encode_robot_state(state)));
      } catch (const std::exception& error) {
        last_error = error.what();
      }
    }
    return snapshot;
  }

  void send_error(std::uint32_t request, BridgeError code,
                  const std::string& text) {
    last_error = text;
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
      std::uint32_t capabilities = CapabilityImu | CapabilitySport |
                                   CapabilityOdometry | CapabilitySystemState;
      if (config.camera_enabled && udp.camera != nullptr) {
        capabilities |= CapabilityCamera;
      }
      // The JOINT capability deliberately remains clear until the vendor
      // publishes a low-level joint SDK and message semantics.
      enqueue_control(make_frame(
          MessageType::HelloAck,
          encode_hello_ack({kBridgeProtocolVersion, capabilities,
                            hello.instance_nonce, sdk.sdk_version()})));
      handshake_complete.store(true);
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
    if (!handshake_complete.load()) {
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
      std::uint16_t result{};
      {
        std::lock_guard<std::mutex> lock(sdk_mutex);
        result = sdk.set_mode(requested.mode, config.sdk_call_timeout_ms);
      }
      if (result == kAstrallSuccess) {
        send_ack(frame.sequence, result, "mode command accepted");
      } else {
        send_error(frame.sequence, BridgeError::SdkCallFailed,
                   "AstrallSportModeControl failed: " +
                       std::to_string(result));
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
          send_ack(frame.sequence, kAstrallSuccess,
                   "software emergency stop latched and damping requested");
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
        last_error = "AstrallHeartbeat failed: " + std::to_string(result);
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
          std::lock_guard<std::mutex> lock(sdk_mutex);
          sdk.move(0, 0, 0, config.sdk_call_timeout_ms);
          sdk.set_mode(kAstrallModeDamping, config.sdk_call_timeout_ms);
          last_error = "PC application heartbeat timed out; damping requested";
        }
      } else {
        const auto velocity = controller.velocity_for_tick().value;
        std::lock_guard<std::mutex> lock(sdk_mutex);
        const auto result = sdk.move(velocity.vx, velocity.vy, velocity.vyaw,
                                     config.sdk_call_timeout_ms);
        if (result != kAstrallSuccess) {
          last_error = "AstrallMove failed: " + std::to_string(result);
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

  void odometry_loop() {
    while (!stop.load() && udp.odometry != nullptr) {
      const auto datagram = udp.odometry->receive(std::chrono::milliseconds(50));
      if (!datagram) {
        continue;
      }
      const auto parsed = parse_odometry_packet(
          datagram->bytes, config.lidar_packing,
          config.odometry_position_scale, config.odometry_quaternion_scale);
      if (!parsed) {
        last_error = "invalid UDP 6101 odometry datagram";
        continue;
      }
      OdometryPayload payload;
      payload.device_time = parsed->device_time;
      payload.position = parsed->position;
      payload.orientation = parsed->orientation;
      enqueue_telemetry(
          make_frame(MessageType::Odometry, encode_odometry(payload)));
    }
  }

  void lidar_loop() {
    while (!stop.load() && udp.lidar != nullptr) {
      const auto datagram = udp.lidar->receive(std::chrono::milliseconds(50));
      if (datagram && !parse_point_cloud_packet(
                          datagram->bytes, config.lidar_packing,
                          config.point_coordinate_scale)) {
        last_error = "invalid UDP 6100 point-cloud datagram";
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
      // TODO:参考手册第24页补充实现：实机确认UDP 6000 H.264分片及关键帧边界。
      CameraChunkPayload payload;
      payload.stream_id = 0;
      payload.datagram_sequence = datagram_sequence++;
      payload.receive_time_ns = datagram->receive_time_ns;
      payload.keyframe_hint = false;
      payload.data = datagram->bytes;
      enqueue_telemetry(make_frame(MessageType::CameraH264,
                                   encode_camera_chunk(payload)));
    }
  }

  void writer_loop() {
    using namespace std::chrono_literals;
    while (true) {
      auto frame = control_output.try_pop();
      if (!frame) {
        frame = telemetry_output.wait_pop_for(10ms);
      }
      if (frame) {
        try {
          if (!stream.write_all(ProtocolHandler::encode(*frame))) {
            stop.store(true);
            return;
          }
        } catch (const std::exception& error) {
          last_error = error.what();
          stop.store(true);
          return;
        }
        continue;
      }
      if (control_output.closed() && telemetry_output.closed() &&
          control_output.size() == 0U && telemetry_output.size() == 0U) {
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

int AgentRuntime::run() {
  if (!impl_->initialize()) {
    impl_->stream.close();
    return 2;
  }

  std::thread writer([this] { impl_->writer_loop(); });
  std::thread heartbeat([this] { impl_->heartbeat_loop(); });
  std::thread motion([this] { impl_->motion_loop(); });
  std::thread state([this] { impl_->state_loop(); });
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
      const auto bytes = impl_->stream.read_some();
      if (bytes.empty()) {
        break;
      }
      for (auto& frame : impl_->decoder.feed(bytes)) {
        impl_->process(std::move(frame));
      }
    }
  } catch (const std::exception& error) {
    impl_->last_error = error.what();
  }

  impl_->stop.store(true);
  impl_->close_udp();
  if (heartbeat.joinable()) heartbeat.join();
  if (motion.joinable()) motion.join();
  if (state.joinable()) state.join();
  if (odometry.joinable()) odometry.join();
  if (lidar.joinable()) lidar.join();
  if (camera.joinable()) camera.join();
  impl_->control_output.close();
  impl_->telemetry_output.close();
  if (writer.joinable()) writer.join();
  impl_->safe_shutdown();
  impl_->stream.close();
  return 0;
}

void AgentRuntime::request_stop() noexcept {
  impl_->stop.store(true);
  impl_->close_udp();
  impl_->control_output.close();
  impl_->telemetry_output.close();
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
