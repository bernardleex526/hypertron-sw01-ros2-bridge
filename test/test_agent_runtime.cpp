#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
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
    value.sport_status = 0xB104U;
    return value;
  }
  std::string sdk_version() const override { return "fake-1"; }

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

}  // namespace
}  // namespace hypertron_ros2_bridge
