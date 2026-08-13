#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/astrall_sdk_adapter.hpp"

namespace hypertron_ros2_bridge {
namespace {
using namespace std::chrono_literals;

class ManualClock final : public IMonotonicClock {
 public:
  time_point now() const override { return now_; }
  void advance(std::chrono::milliseconds value) { now_ += value; }

 private:
  time_point now_{};
};

class FakeSdk : public IAstrallSdk {
 public:
  std::uint16_t init(const SdkCallbacks& callbacks, std::uint32_t) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callbacks_ = callbacks;
    }
    record("init");
    return kAstrallSuccess;
  }
  void deinit() noexcept override { record("deinit"); }
  std::uint16_t heartbeat(std::uint32_t) override {
    record("heartbeat");
    return kAstrallSuccess;
  }
  std::uint16_t acquire_sdk_control(std::uint32_t) override {
    record("acquire");
    return kAstrallSuccess;
  }
  std::uint16_t configure_udp_streams(bool camera, bool lidar,
                                      std::uint32_t) override {
    record("udp:" + std::to_string(camera ? 1 : 0) + "," +
           std::to_string(lidar ? 1 : 0));
    return kAstrallSuccess;
  }
  std::uint16_t move(float vx, float vy, float vyaw,
                     std::uint32_t) override {
    record("move:" + std::to_string(vx) + "," + std::to_string(vy) +
           "," + std::to_string(vyaw));
    return kAstrallSuccess;
  }
  std::uint16_t set_mode(std::uint16_t mode, std::uint32_t) override {
    record("mode:" + std::to_string(mode));
    return kAstrallSuccess;
  }
  SdkSnapshot snapshot() override {
    SdkSnapshot value;
    value.sdk_linked = true;
    value.control_authority = true;
    value.sport_status = sport_status_.load();
    return value;
  }
  std::string sdk_version() const override { return "fake-1"; }
  void set_sport_status(std::uint16_t value) { sport_status_.store(value); }
  void emit_imu(ImuPayload value) {
    std::function<void(ImuPayload)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback = callbacks_.on_imu;
    }
    if (callback) callback(std::move(value));
  }

  std::vector<std::string> calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }
  std::size_t count(const std::string& value) const {
    const auto copy = calls();
    return static_cast<std::size_t>(std::count(copy.begin(), copy.end(), value));
  }
  std::size_t count_prefix(const std::string& prefix) const {
    const auto copy = calls();
    return static_cast<std::size_t>(std::count_if(
        copy.begin(), copy.end(), [&](const std::string& value) {
          return value.rfind(prefix, 0) == 0;
        }));
  }

 private:
  void record(std::string value) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(std::move(value));
  }
  mutable std::mutex mutex_;
  std::vector<std::string> calls_;
  SdkCallbacks callbacks_;
  std::atomic<std::uint16_t> sport_status_{0xB104U};
};

class EofStream final : public IByteStream {
 public:
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    return std::nullopt;
  }
  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    writes_.push_back(bytes);
    return true;
  }
  void close() noexcept override { closed_ = true; }
  bool closed() const { return closed_; }

 private:
  std::vector<std::vector<std::uint8_t>> writes_;
  bool closed_{false};
};

class ScriptStream final : public IByteStream {
 public:
  explicit ScriptStream(std::vector<std::uint8_t> script)
      : script_(std::move(script)) {}
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    if (delivered_) return std::nullopt;
    delivered_ = true;
    return script_;
  }
  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    writes_.push_back(bytes);
    return true;
  }
  void close() noexcept override { closed_ = true; }

 private:
  std::vector<std::uint8_t> script_;
  std::vector<std::vector<std::uint8_t>> writes_;
  bool delivered_{};
  bool closed_{};
};

class BlockingFailedWriterStream final : public IByteStream {
 public:
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds timeout) override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!delivered_) {
      delivered_ = true;
      return ProtocolHandler::encode(
          {MessageType::Hello, 1, 1,
           encode_hello({1, 1, CapabilitySystemState, 7})});
    }
    if (!cv_.wait_for(lock, timeout, [this] { return closed_; })) {
      return std::vector<std::uint8_t>{};
    }
    return std::nullopt;
  }
  bool write_all(const std::vector<std::uint8_t>&) override { return false; }
  void close() noexcept override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool closed_{};
  bool delivered_{};
};

class ModeStream final : public IByteStream {
 public:
  explicit ModeStream(std::vector<std::uint8_t> script)
      : script_(std::move(script)) {}

  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds timeout) override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!delivered_) {
      delivered_ = true;
      return script_;
    }
    if (!cv_.wait_for(lock, timeout, [this] { return closed_; })) {
      return std::vector<std::uint8_t>{};
    }
    return std::nullopt;
  }

  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      writes_.push_back(bytes);
    }
    cv_.notify_all();
    return true;
  }

  void close() noexcept override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  bool wait_for_ack(std::uint32_t request_sequence,
                    std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      for (const auto& bytes : writes_) {
        ProtocolHandler decoder;
        for (const auto& frame : decoder.feed(bytes)) {
          if (frame.type == MessageType::Ack &&
              decode_ack(frame.payload).request_sequence == request_sequence) {
            return true;
          }
        }
      }
      return false;
    });
  }

 private:
  std::vector<std::uint8_t> script_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::vector<std::uint8_t>> writes_;
  bool delivered_{};
  bool closed_{};
};

class InteractiveStream final : public IByteStream {
 public:
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds timeout) override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout,
                      [this] { return closed_ || !inputs_.empty(); })) {
      return std::vector<std::uint8_t>{};
    }
    if (inputs_.empty()) return std::nullopt;
    auto bytes = std::move(inputs_.front());
    inputs_.erase(inputs_.begin());
    return bytes;
  }

  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      writes_.push_back(bytes);
    }
    cv_.notify_all();
    return true;
  }

  void close() noexcept override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  void push(Frame frame) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.push_back(ProtocolHandler::encode(frame));
    }
    cv_.notify_all();
  }

  bool wait_for_frame(MessageType type, std::uint32_t request_sequence,
                      std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      for (const auto& bytes : writes_) {
        ProtocolHandler decoder;
        for (const auto& frame : decoder.feed(bytes)) {
          if (frame.type != type) continue;
          if (type == MessageType::Error &&
              decode_error(frame.payload).request_sequence == request_sequence) {
            return true;
          }
          if (type == MessageType::Ack &&
              decode_ack(frame.payload).request_sequence == request_sequence) {
            return true;
          }
          if (type == MessageType::HelloAck || type == MessageType::Pong) {
            return true;
          }
        }
      }
      return false;
    });
  }

  std::vector<Frame> written_frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Frame> result;
    ProtocolHandler decoder;
    for (const auto& bytes : writes_) {
      for (auto& frame : decoder.feed(bytes)) result.push_back(std::move(frame));
    }
    return result;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::vector<std::uint8_t>> inputs_;
  std::vector<std::vector<std::uint8_t>> writes_;
  bool closed_{};
};

class TimeoutUdpSource final : public IUdpSource {
 public:
  std::optional<UdpDatagram> receive(
      std::chrono::milliseconds) override {
    return std::nullopt;
  }
  void close() noexcept override {}
};

class GatedDatagramUdpSource final : public IUdpSource {
 public:
  explicit GatedDatagramUdpSource(std::vector<std::uint8_t> bytes)
      : bytes_(std::move(bytes)) {}
  // 由测试在 HELLO 握手完成后打开，确保数据报在握手后被处理。
  void open() { gate_.store(true); }
  std::optional<UdpDatagram> receive(
      std::chrono::milliseconds) override {
    if (!gate_.load() || delivered_) return std::nullopt;
    delivered_ = true;
    UdpDatagram datagram;
    datagram.bytes = bytes_;
    return datagram;
  }
  void close() noexcept override {}

 private:
  std::vector<std::uint8_t> bytes_;
  std::atomic_bool gate_{false};
  bool delivered_{};
};

std::vector<std::uint8_t> command_script(
    const std::vector<Frame>& frames) {
  std::vector<std::uint8_t> bytes;
  for (const auto& frame : frames) {
    const auto encoded = ProtocolHandler::encode(frame);
    bytes.insert(bytes.end(), encoded.begin(), encoded.end());
  }
  return bytes;
}

AgentConfig config() {
  AgentConfig value;
  value.auto_acquire_control = true;
  value.heartbeat_period = 100ms;
  value.motion_period = 20ms;
  value.state_period = 500ms;
  value.application_timeout = 500ms;
  return value;
}

TEST(AgentRuntime, ChannelLossStopsDampsAndDeinitializesInOrder) {
  FakeSdk sdk;
  EofStream stream;
  ManualClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  EXPECT_EQ(agent.run(), 0);
  const auto calls = sdk.calls();
  ASSERT_GE(calls.size(), 5U);
  EXPECT_EQ(calls.front(), "init");
  EXPECT_EQ(calls[calls.size() - 3U],
            "move:0.000000,0.000000,0.000000");
  EXPECT_EQ(calls[calls.size() - 2U], "mode:41217");
  EXPECT_EQ(calls.back(), "deinit");
  EXPECT_TRUE(stream.closed());
}

TEST(AgentRuntime, HeartbeatAndMoveUseIndependentSchedules) {
  FakeSdk sdk;
  EofStream stream;
  ManualClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  agent.step_for(1s);
  EXPECT_EQ(sdk.count("heartbeat"), 10U);
  EXPECT_EQ(sdk.count_prefix("move:"), 50U);
}

TEST(AgentRuntime, InitializationFailureDoesNotIssueMotion) {
  class FailingSdk final : public FakeSdk {
   public:
    std::uint16_t init(const SdkCallbacks&, std::uint32_t) override {
      return 0x8005U;
    }
  } sdk;
  EofStream stream;
  ManualClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  EXPECT_NE(agent.run(), 0);
  EXPECT_EQ(sdk.count_prefix("move:"), 0U);
}

TEST(AgentRuntime, EstopLatchRejectsFollowingMoveMode) {
  FakeSdk sdk;
  ScriptStream stream(command_script({
      {MessageType::Hello, 1, 1,
       encode_hello({1, 1, CapabilitySystemState, 7})},
      {MessageType::CmdEstop, 2, 2, encode_estop({true})},
      {MessageType::CmdMode, 3, 3, encode_mode({0xA104U})},
  }));
  ManualClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  EXPECT_EQ(agent.run(), 0);
  const auto calls = sdk.calls();
  EXPECT_EQ(std::count(calls.begin(), calls.end(), "mode:41220"), 0);
}

TEST(AgentRuntime, ModeAckWaitsForDocumentedStableSportState) {
  FakeSdk sdk;
  ModeStream stream(command_script({
      {MessageType::Hello, 1, 1,
       encode_hello({1, 1, CapabilitySystemState, 7})},
      {MessageType::CmdMode, 2, 2, encode_mode({0xA102U})},
  }));
  SteadyMonotonicClock clock;
  auto agent_config = config();
  agent_config.mode_timeout = 500ms;
  agent_config.mode_poll_period = 10ms;
  AgentRuntime agent(agent_config, sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });

  for (int attempt = 0;
       attempt < 100 && sdk.count("mode:41218") == 0U; ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(sdk.count("mode:41218"), 1U);
  EXPECT_FALSE(stream.wait_for_ack(2U, 50ms));
  sdk.set_sport_status(0xB102U);
  EXPECT_TRUE(stream.wait_for_ack(2U, 500ms));

  stream.close();
  EXPECT_EQ(run.get(), 0);
}

TEST(AgentRuntime, WriterFailureClosesStreamAndUnblocksReader) {
  FakeSdk sdk;
  BlockingFailedWriterStream stream;
  SteadyMonotonicClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  const auto completed_without_external_close =
      run.wait_for(750ms) == std::future_status::ready;
  if (!completed_without_external_close) stream.close();
  EXPECT_TRUE(completed_without_external_close);
  EXPECT_EQ(run.get(), 0);
}

TEST(AgentRuntime, EnablesVendorLidarSubscriptionBeforeUdpReceive) {
  FakeSdk sdk;
  EofStream stream;
  ManualClock clock;
  TimeoutUdpSource lidar;
  AgentRuntime agent(config(), sdk, stream, clock,
                     {nullptr, &lidar, &lidar});
  agent.step_for(20ms);
  const auto calls = sdk.calls();
  const auto init = std::find(calls.begin(), calls.end(), "init");
  const auto udp = std::find(calls.begin(), calls.end(), "udp:0,1");
  const auto acquire = std::find(calls.begin(), calls.end(), "acquire");
  ASSERT_NE(init, calls.end());
  ASSERT_NE(udp, calls.end());
  ASSERT_NE(acquire, calls.end());
  EXPECT_LT(init, udp);
  EXPECT_LT(udp, acquire);
}

TEST(AgentRuntime, TelemetryIsSilentUntilHelloCompletes) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  auto agent_config = config();
  agent_config.state_period = 10ms;
  AgentRuntime agent(agent_config, sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  for (int attempt = 0; attempt < 100 && sdk.count("init") == 0U; ++attempt) {
    std::this_thread::sleep_for(2ms);
  }
  ImuPayload imu;
  imu.quaternion = {0, 0, 0, 1};
  sdk.emit_imu(imu);
  std::this_thread::sleep_for(60ms);
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);
  for (const auto& frame : stream.written_frames()) {
    EXPECT_TRUE(frame.type == MessageType::HelloAck ||
                frame.type == MessageType::Ack ||
                frame.type == MessageType::Error ||
                frame.type == MessageType::Pong);
  }
}

TEST(AgentRuntime, HelloAckIsRequestedRuntimeCapabilityIntersection) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  constexpr std::uint32_t requested = CapabilityOdometry | CapabilityCamera |
                                      CapabilitySystemState;
  stream.push({MessageType::Hello, 10, 1,
               encode_hello({1, 1, requested, 0x55AAU})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::HelloAck, 0, 500ms));
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);

  const auto frames = stream.written_frames();
  const auto ack = std::find_if(frames.begin(), frames.end(), [](const Frame& f) {
    return f.type == MessageType::HelloAck;
  });
  ASSERT_NE(ack, frames.end());
  EXPECT_EQ(decode_hello_ack(ack->payload).capabilities,
            CapabilitySystemState);
}

TEST(AgentRuntime, UnnegotiatedTelemetryCapabilityRemainsSilent) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  stream.push({MessageType::Hello, 10, 1,
               encode_hello({1, 1, CapabilitySystemState, 0x55AAU})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::HelloAck, 0, 500ms));
  ImuPayload imu;
  imu.quaternion = {0, 0, 0, 1};
  sdk.emit_imu(imu);
  std::this_thread::sleep_for(40ms);
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);
  for (const auto& frame : stream.written_frames()) {
    EXPECT_NE(frame.type, MessageType::Imu);
  }
}

TEST(AgentRuntime, ModeTimeoutKeepsZeroDespiteNewVelocityFrames) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  auto agent_config = config();
  agent_config.application_timeout = 2s;
  agent_config.mode_timeout = 80ms;
  agent_config.mode_poll_period = 5ms;
  agent_config.state_period = 2s;
  AgentRuntime agent(agent_config, sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  stream.push({MessageType::Hello, 1, 1,
               encode_hello({1, 1, CapabilitySystemState, 7})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::HelloAck, 0, 500ms));
  stream.push({MessageType::CmdVelocity, 2, 2,
               encode_velocity({0.6F, 0, 0})});
  for (int attempt = 0;
       attempt < 100 && sdk.count("move:0.600000,0.000000,0.000000") == 0U;
       ++attempt) {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_GT(sdk.count("move:0.600000,0.000000,0.000000"), 0U);

  stream.push({MessageType::CmdMode, 3, 3, encode_mode({0xA102U})});
  stream.push({MessageType::CmdVelocity, 4, 4,
               encode_velocity({0.8F, 0, 0})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::Error, 3, 1s));
  std::this_thread::sleep_for(60ms);
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);

  const auto calls = sdk.calls();
  const auto mode = std::find(calls.begin(), calls.end(), "mode:41218");
  ASSERT_NE(mode, calls.end());
  ASSERT_NE(mode, calls.begin());
  EXPECT_EQ(*(mode - 1), "move:0.000000,0.000000,0.000000");
  for (auto call = mode + 1; call != calls.end(); ++call) {
    if (call->rfind("move:", 0) == 0) {
      EXPECT_EQ(*call, "move:0.000000,0.000000,0.000000");
    }
  }
}

TEST(AgentRuntime, RequestStopWakesARealPipeAndRunsSafeShutdown) {
  int input_pipe[2]{};
  int output_pipe[2]{};
  ASSERT_EQ(::pipe(input_pipe), 0);
  ASSERT_EQ(::pipe(output_pipe), 0);
  FakeSdk sdk;
  PosixByteStream stream(input_pipe[0], output_pipe[1], true);
  SteadyMonotonicClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  for (int attempt = 0; attempt < 100 && sdk.count("init") == 0U; ++attempt) {
    std::this_thread::sleep_for(2ms);
  }
  agent.request_stop();
  EXPECT_EQ(run.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(run.get(), 0);
  const auto calls = sdk.calls();
  ASSERT_GE(calls.size(), 3U);
  EXPECT_EQ(calls[calls.size() - 3U],
            "move:0.000000,0.000000,0.000000");
  EXPECT_EQ(calls[calls.size() - 2U], "mode:41217");
  EXPECT_EQ(calls.back(), "deinit");
  ::close(input_pipe[1]);
  ::close(output_pipe[0]);
}

TEST(AgentRuntime, ExternalSignalPredicateStopsTimedPipeRead) {
  int input_pipe[2]{};
  int output_pipe[2]{};
  ASSERT_EQ(::pipe(input_pipe), 0);
  ASSERT_EQ(::pipe(output_pipe), 0);
  FakeSdk sdk;
  PosixByteStream stream(input_pipe[0], output_pipe[1], true);
  SteadyMonotonicClock clock;
  AgentRuntime agent(config(), sdk, stream, clock);
  std::atomic_bool signal_received{false};
  auto run = std::async(std::launch::async, [&] {
    return agent.run([&] { return signal_received.load(); });
  });
  std::this_thread::sleep_for(30ms);
  signal_received.store(true);
  EXPECT_EQ(run.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(run.get(), 0);
  ::close(input_pipe[1]);
  ::close(output_pipe[0]);
}

TEST(AgentRuntime, EstopConfirmationDoesNotBlockPingProcessing) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  auto agent_config = config();
  // 默认 mode_timeout=10s -> 急停确认轮询上限 500 ms。若确认仍在 process()
  // 内同步执行，PING 会排队到确认结束之后，PONG 无法在 250 ms 内返回；
  // 确认移入独立 worker 后，PING 应被立即应答。
  agent_config.mode_poll_period = 5ms;
  agent_config.application_timeout = 2s;  // 避免 CI 调度抖动触发安全停摆
  AgentRuntime agent(agent_config, sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  stream.push({MessageType::Hello, 1, 1,
               encode_hello({1, 1, CapabilitySystemState, 7})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::HelloAck, 0, 500ms));
  // FakeSdk 初始 sport_status=0xB104（非阻尼稳态），确认轮询会持续进行。
  stream.push({MessageType::CmdEstop, 2, 2, encode_estop({true})});
  stream.push({MessageType::Ping, 3, 3, {}});
  EXPECT_TRUE(stream.wait_for_frame(MessageType::Pong, 0, 250ms));
  // 稳态未达成前不得提前 ACK。
  EXPECT_FALSE(stream.wait_for_frame(MessageType::Ack, 2, 50ms));
  sdk.set_sport_status(0xB101U);
  EXPECT_TRUE(stream.wait_for_frame(MessageType::Ack, 2, 500ms));
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);
  const auto frames = stream.written_frames();
  const auto ack = std::find_if(frames.begin(), frames.end(), [](const Frame& f) {
    return f.type == MessageType::Ack &&
           decode_ack(f.payload).request_sequence == 2U;
  });
  ASSERT_NE(ack, frames.end());
  EXPECT_NE(decode_ack(ack->payload).text.find("damping state confirmed"),
            std::string::npos);
}

TEST(AgentRuntime, EstopConfirmationTimeoutStillAcksAsRequested) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  auto agent_config = config();
  agent_config.mode_timeout = 200ms;  // 确认轮询上限 = 40 ms
  agent_config.mode_poll_period = 2ms;
  agent_config.application_timeout = 2s;
  AgentRuntime agent(agent_config, sdk, stream, clock);
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  stream.push({MessageType::Hello, 1, 1,
               encode_hello({1, 1, CapabilitySystemState, 7})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::HelloAck, 0, 500ms));
  sdk.set_sport_status(0xB102U);  // stand：永远达不到阻尼稳态
  stream.push({MessageType::CmdEstop, 2, 2, encode_estop({true})});
  // 确认超时后必须仍以 ACK（注明 requested）结束，而不是 ERROR 或静默。
  EXPECT_TRUE(stream.wait_for_frame(MessageType::Ack, 2, 500ms));
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);
  const auto frames = stream.written_frames();
  const auto ack = std::find_if(frames.begin(), frames.end(), [](const Frame& f) {
    return f.type == MessageType::Ack &&
           decode_ack(f.payload).request_sequence == 2U;
  });
  ASSERT_NE(ack, frames.end());
  EXPECT_NE(decode_ack(ack->payload).text.find("damping requested"),
            std::string::npos);
}

TEST(AgentRuntime, InvalidUdp6100DatagramDoesNotClobberLastError) {
  FakeSdk sdk;
  InteractiveStream stream;
  SteadyMonotonicClock clock;
  auto agent_config = config();
  agent_config.state_period = 10ms;
  agent_config.application_timeout = 2s;
  // 一个畸形的 6100 数据报：头部 magic 正确以外全部无效。
  std::vector<std::uint8_t> garbage(64, 0x5AU);
  garbage[0] = 0x55;
  garbage[1] = 0xAA;
  GatedDatagramUdpSource lidar(std::move(garbage));
  TimeoutUdpSource odometry;
  AgentRuntime agent(agent_config, sdk, stream, clock,
                     {nullptr, &lidar, &odometry});
  auto run = std::async(std::launch::async, [&agent] { return agent.run(); });
  stream.push({MessageType::Hello, 1, 1,
               encode_hello({1, 1, CapabilitySystemState, 7})});
  ASSERT_TRUE(stream.wait_for_frame(MessageType::HelloAck, 0, 500ms));
  // 握手完成后才投递畸形数据报，确保它被 lidar_loop 实际处理。
  lidar.open();
  // 等待 RobotState 帧发布。
  std::this_thread::sleep_for(60ms);
  agent.request_stop();
  EXPECT_EQ(run.get(), 0);
  const auto frames = stream.written_frames();
  bool saw_robot_state = false;
  for (const auto& frame : frames) {
    if (frame.type != MessageType::RobotState) continue;
    saw_robot_state = true;
    const auto state = decode_robot_state(frame.payload);
    // 6100 无效数据报只进入诊断计数，不得覆盖 last_error（否则会掩盖
    // SDK/链路错误，并且与 6101 里程计错误混为一谈）。
    EXPECT_EQ(state.last_error.find("6100"), std::string::npos);
  }
  EXPECT_TRUE(saw_robot_state);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
