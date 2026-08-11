#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
  std::uint16_t init(const SdkCallbacks&, std::uint32_t) override {
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
  std::atomic<std::uint16_t> sport_status_{0xB104U};
};

class EofStream final : public IByteStream {
 public:
  std::vector<std::uint8_t> read_some() override { return {}; }
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
  std::vector<std::uint8_t> read_some() override {
    if (delivered_) return {};
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
  std::vector<std::uint8_t> read_some() override {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return closed_; });
    return {};
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
};

class ModeStream final : public IByteStream {
 public:
  explicit ModeStream(std::vector<std::uint8_t> script)
      : script_(std::move(script)) {}

  std::vector<std::uint8_t> read_some() override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!delivered_) {
      delivered_ = true;
      return script_;
    }
    cv_.wait(lock, [this] { return closed_; });
    return {};
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

class TimeoutUdpSource final : public IUdpSource {
 public:
  std::optional<UdpDatagram> receive(
      std::chrono::milliseconds) override {
    return std::nullopt;
  }
  void close() noexcept override {}
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

}  // namespace
}  // namespace hypertron_ros2_bridge
