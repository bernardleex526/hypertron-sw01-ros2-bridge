#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/ssh_tunnel.hpp"

#ifdef HYPERTRON_TEST_WITH_LIBSSH
#include <libssh/libssh.h>
#endif

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

class ConcurrentBackend final : public ISshBackend {
 public:
  ConnectResult connect(const SshConfig&) override {
    return ConnectResult::Success;
  }
  bool write_all(const std::vector<std::uint8_t>&) override { return true; }
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      in_read_ = true;
    }
    cv_.notify_all();
    std::this_thread::sleep_for(100ms);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      in_read_ = false;
    }
    return std::vector<std::uint8_t>{};
  }
  bool send_keepalive() override { return true; }
  void disconnect() noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (in_read_) concurrent_disconnect_ = true;
  }
  bool wait_until_reading() {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 1s, [this] { return in_read_; });
  }
  bool concurrent_disconnect() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return concurrent_disconnect_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool in_read_{};
  bool concurrent_disconnect_{};
};

class InvalidFrameBackend final : public ISshBackend {
 public:
  ConnectResult connect(const SshConfig&) override {
    return ConnectResult::Success;
  }
  bool write_all(const std::vector<std::uint8_t>&) override { return true; }
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    if (delivered_) return std::vector<std::uint8_t>{};
    delivered_ = true;
    auto bytes = ProtocolHandler::encode(
        {MessageType::Pong, 1, 1, {}});
    bytes.back() ^= 0x80U;
    return bytes;
  }
  bool send_keepalive() override { return true; }
  void disconnect() noexcept override {}

 private:
  bool delivered_{};
};

class BurstBackend final : public ISshBackend {
 public:
  ConnectResult connect(const SshConfig&) override {
    return ConnectResult::Success;
  }
  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    std::lock_guard<std::mutex> lock(mutex_);
    writes_.push_back(bytes);
    return true;
  }
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reading_ = true;
    }
    cv_.notify_all();
    std::this_thread::sleep_for(100ms);
    return std::vector<std::uint8_t>{};
  }
  bool send_keepalive() override { return true; }
  void disconnect() noexcept override {}
  bool wait_until_reading() {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 1s, [this] { return reading_; });
  }
  std::vector<std::vector<std::uint8_t>> writes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool reading_{};
  std::vector<std::vector<std::uint8_t>> writes_;
};

class ThrowingBackend final : public ISshBackend {
 public:
  ConnectResult connect(const SshConfig&) override {
    return ConnectResult::Success;
  }
  bool write_all(const std::vector<std::uint8_t>&) override { return true; }
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    throw std::runtime_error("injected SSH read failure");
  }
  bool send_keepalive() override { return true; }
  void disconnect() noexcept override {}
};

class CallbackFrameBackend final : public ISshBackend {
 public:
  ConnectResult connect(const SshConfig&) override {
    return ConnectResult::Success;
  }
  bool write_all(const std::vector<std::uint8_t>&) override { return true; }
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds) override {
    if (delivered_) return std::vector<std::uint8_t>{};
    delivered_ = true;
    return ProtocolHandler::encode(
        {MessageType::RobotState, 1, 1, {0x01U}});
  }
  bool send_keepalive() override { return true; }
  void disconnect() noexcept override {}

 private:
  bool delivered_{};
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

TEST(SshTunnel, RejectsConfigurationThatDisablesHostKeyChecking) {
  FakeBackend backend({ConnectResult::Success});
  FakeSleeper sleeper;
  auto insecure = config();
  insecure.strict_host_key_checking = false;
  EXPECT_THROW(SshTunnel(insecure, backend, sleeper), std::invalid_argument);
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

TEST(SshTunnel, StopDoesNotDisconnectBackendDuringWorkerReadAndIsIdempotent) {
  ConcurrentBackend backend;
  FakeSleeper sleeper;
  SshTunnel tunnel(config(), backend, sleeper);
  std::atomic<int> stopped_callbacks{0};
  ASSERT_TRUE(tunnel.start(
      [](Frame) {},
      [&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::Stopped) {
          stopped_callbacks.fetch_add(1);
        }
      }));
  ASSERT_TRUE(backend.wait_until_reading());
  tunnel.stop();
  tunnel.stop();
  EXPECT_FALSE(backend.concurrent_disconnect());
  EXPECT_EQ(stopped_callbacks.load(), 1);
  EXPECT_FALSE(tunnel.start([](Frame) {},
                            [](ConnectionState, const std::string&) {}));
}

TEST(SshTunnel, UsesStartupGraceUntilFirstApplicationPong) {
  ConcurrentBackend backend;
  FakeSleeper sleeper;
  auto startup_config = config();
  startup_config.ping_interval = 10ms;
  startup_config.application_timeout = 40ms;
  startup_config.agent_startup_timeout = 300ms;
  SshTunnel tunnel(startup_config, backend, sleeper);
  ASSERT_TRUE(tunnel.start([](Frame) {},
                           [](ConnectionState, const std::string&) {}));
  ASSERT_TRUE(backend.wait_until_reading());
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(tunnel.state(), ConnectionState::Connected);
  tunnel.stop();
}

TEST(SshTunnel, CountsProtocolCorruptionBeforeDisconnect) {
  InvalidFrameBackend backend;
  FakeSleeper sleeper;
  sleeper.continue_sleep = false;
  SshTunnel tunnel(config(), backend, sleeper);
  ASSERT_TRUE(tunnel.start([](Frame) {}, [](ConnectionState, const std::string&) {}));
  std::this_thread::sleep_for(100ms);
  tunnel.stop();
  EXPECT_EQ(tunnel.protocol_drops(), 1U);
}

TEST(SshTunnel, ClassifiesEveryLibsshNegativeReadBeforeSizingBuffer) {
  using detail::LibsshReadDisposition;
#ifdef HYPERTRON_TEST_WITH_LIBSSH
  constexpr int again = SSH_AGAIN;
  constexpr int eof = SSH_EOF;
#else
  constexpr int again = -2;
  constexpr int eof = -127;
#endif
  EXPECT_EQ(detail::classify_libssh_read(12),
            LibsshReadDisposition::Data);
  EXPECT_EQ(detail::classify_libssh_read(0),
            LibsshReadDisposition::Eof);
  EXPECT_EQ(detail::classify_libssh_read(again),
            LibsshReadDisposition::Again);
  EXPECT_EQ(detail::classify_libssh_read(eof),
            LibsshReadDisposition::Eof);
  EXPECT_EQ(detail::classify_libssh_read(-1),
            LibsshReadDisposition::Error);
  EXPECT_EQ(detail::classify_libssh_read(-99),
            LibsshReadDisposition::Error);
}

TEST(SshTunnel, WorkerExceptionBecomesDisconnectInsteadOfTermination) {
  ThrowingBackend backend;
  FakeSleeper sleeper;
  sleeper.continue_sleep = false;
  SshTunnel tunnel(config(), backend, sleeper);
  std::atomic_bool disconnected{false};
  ASSERT_TRUE(tunnel.start(
      [](Frame) {},
      [&](ConnectionState state, const std::string& detail) {
        if (state == ConnectionState::Disconnected &&
            detail.find("injected SSH read failure") != std::string::npos) {
          disconnected.store(true);
        }
      }));
  for (int attempt = 0; attempt < 100 && !disconnected.load(); ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  tunnel.stop();
  EXPECT_TRUE(disconnected.load());
}

TEST(SshTunnel, CallbackProtocolFailureIsRecordedOnDisconnect) {
  CallbackFrameBackend backend;
  FakeSleeper sleeper;
  sleeper.continue_sleep = false;
  SshTunnel tunnel(config(), backend, sleeper);
  std::atomic_bool recorded{false};
  ASSERT_TRUE(tunnel.start(
      [](Frame) { throw ProtocolError("malformed RobotState payload"); },
      [&](ConnectionState state, const std::string& detail) {
        if (state == ConnectionState::Disconnected &&
            detail.find("malformed RobotState payload") != std::string::npos) {
          recorded.store(true);
        }
      }));
  for (int attempt = 0; attempt < 100 && !recorded.load(); ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  tunnel.stop();
  EXPECT_TRUE(recorded.load());
  EXPECT_EQ(tunnel.protocol_drops(), 1U);
}

TEST(SshTunnel, EstopPreemptsBurstAndVelocityUsesLatestMailbox) {
  BurstBackend backend;
  FakeSleeper sleeper;
  SshTunnel tunnel(config(), backend, sleeper);
  ASSERT_TRUE(tunnel.start([](Frame) {}, [](ConnectionState, const std::string&) {}));
  ASSERT_TRUE(backend.wait_until_reading());
  ASSERT_TRUE(tunnel.send({MessageType::CmdVelocity, 1, 1,
                           encode_velocity({0.1F, 0, 0})}));
  ASSERT_TRUE(tunnel.send({MessageType::CmdVelocity, 2, 2,
                           encode_velocity({0.2F, 0, 0})}));
  ASSERT_TRUE(tunnel.send({MessageType::CmdVelocity, 3, 3,
                           encode_velocity({0.3F, 0, 0})}));
  ASSERT_TRUE(tunnel.send(
      {MessageType::CmdEstop, 4, 4, encode_estop({true})}));
  std::this_thread::sleep_for(150ms);
  tunnel.stop();

  std::vector<Frame> sent;
  ProtocolHandler decoder;
  for (const auto& bytes : backend.writes()) {
    for (auto& frame : decoder.feed(bytes)) {
      if (frame.type != MessageType::Ping) sent.push_back(std::move(frame));
    }
  }
  ASSERT_EQ(sent.size(), 2U);
  EXPECT_EQ(sent[0].type, MessageType::CmdEstop);
  EXPECT_EQ(sent[1].type, MessageType::CmdVelocity);
  EXPECT_FLOAT_EQ(decode_velocity(sent[1].payload).vx, 0.3F);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
