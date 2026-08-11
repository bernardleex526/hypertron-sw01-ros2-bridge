#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/ssh_tunnel.hpp"

namespace hypertron_ros2_bridge {
namespace {
using namespace std::chrono_literals;

class FakeBackend final : public ISshBackend {
 public:
  explicit FakeBackend(std::vector<ConnectResult> results)
      : results_(std::move(results)) {}

  ConnectResult connect(const SshConfig& config) override {
    observed_configs.push_back(config);
    if (next_ >= results_.size()) return ConnectResult::Success;
    return results_[next_++];
  }
  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    writes.push_back(bytes);
    return write_ok;
  }
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    return std::nullopt;
  }
  bool send_keepalive() override { return true; }
  void disconnect() noexcept override { ++disconnects; }

  std::vector<SshConfig> observed_configs;
  std::vector<std::vector<std::uint8_t>> writes;
  bool write_ok{true};
  int disconnects{};

 private:
  std::vector<ConnectResult> results_;
  std::size_t next_{};
};

class FakeSleeper final : public ISleeper {
 public:
  bool sleep_for(std::chrono::milliseconds delay,
                 const std::atomic_bool&) override {
    delays.push_back(delay);
    return continue_sleep;
  }
  std::vector<std::chrono::milliseconds> delays;
  bool continue_sleep{true};
};

SshConfig config() {
  SshConfig value;
  value.host = "10.18.0.100";
  value.port = 22;
  value.username = "robot";
  value.private_key = "/home/user/.ssh/id_ed25519";
  value.known_hosts = "/home/user/.ssh/known_hosts";
  value.strict_host_key_checking = true;
  value.remote_command = "/opt/hypertron/bin/hypertron_bridge_agent --stdio";
  value.reconnect_initial_delay = 1s;
  value.reconnect_max_delay = 30s;
  return value;
}

TEST(SshTunnel, UsesExponentialBackoffCappedAtThirtySeconds) {
  FakeBackend backend({ConnectResult::RetryableFailure,
                       ConnectResult::RetryableFailure,
                       ConnectResult::RetryableFailure,
                       ConnectResult::Success});
  FakeSleeper sleeper;
  SshTunnel tunnel(config(), backend, sleeper);
  EXPECT_TRUE(tunnel.run_until_connected());
  EXPECT_EQ(sleeper.delays,
            (std::vector<std::chrono::milliseconds>{1s, 2s, 4s}));
}

TEST(SshTunnel, PermanentHostKeyFailureDoesNotRetry) {
  FakeBackend backend({ConnectResult::PermanentFailure});
  FakeSleeper sleeper;
  auto strict = config();
  strict.password = "must-not-leak";
  SshTunnel tunnel(strict, backend, sleeper);
  EXPECT_FALSE(tunnel.run_until_connected());
  EXPECT_TRUE(sleeper.delays.empty());
  ASSERT_EQ(backend.observed_configs.size(), 1U);
  EXPECT_TRUE(backend.observed_configs.front().strict_host_key_checking);
}

TEST(SshTunnel, DisconnectedCommandsAreDroppedNotReplayed) {
  FakeBackend backend({ConnectResult::Success});
  FakeSleeper sleeper;
  SshTunnel tunnel(config(), backend, sleeper);
  EXPECT_FALSE(tunnel.send({MessageType::CmdVelocity, 1, 0,
                            encode_velocity({0.5F, 0, 0})}));
  EXPECT_TRUE(backend.writes.empty());
}

TEST(SshTunnel, StopInterruptsReconnectBackoff) {
  FakeBackend backend({ConnectResult::RetryableFailure});
  FakeSleeper sleeper;
  sleeper.continue_sleep = false;
  SshTunnel tunnel(config(), backend, sleeper);
  EXPECT_FALSE(tunnel.run_until_connected());
  ASSERT_EQ(sleeper.delays.size(), 1U);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
