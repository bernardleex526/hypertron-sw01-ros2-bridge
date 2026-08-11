#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hypertron_ros2_bridge/protocol_handler.hpp"
#include "hypertron_ros2_bridge/thread_safe_queue.hpp"

namespace hypertron_ros2_bridge {

struct SshConfig {
  std::string host;
  std::uint16_t port{22};
  std::string username;
  std::string password;
  std::string private_key;
  std::string known_hosts;
  bool strict_host_key_checking{true};
  std::string remote_command;
  std::chrono::milliseconds connect_timeout{5000};
  std::chrono::milliseconds keepalive_interval{1000};
  std::chrono::milliseconds ping_interval{200};
  std::chrono::milliseconds application_timeout{500};
  std::chrono::milliseconds reconnect_initial_delay{1000};
  std::chrono::milliseconds reconnect_max_delay{30000};
  std::size_t queue_capacity{64};
  std::uint32_t max_payload{kDefaultMaxPayload};
};

enum class ConnectResult { Success, RetryableFailure, PermanentFailure };
enum class ConnectionState { Stopped, Connecting, Connected, Disconnected, Failed };

class ISshBackend {
 public:
  virtual ~ISshBackend() = default;
  virtual ConnectResult connect(const SshConfig& config) = 0;
  virtual bool write_all(const std::vector<std::uint8_t>& bytes) = 0;
  // nullopt means EOF/permanent channel failure; an empty vector is a timeout.
  virtual std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds timeout) = 0;
  virtual bool send_keepalive() = 0;
  virtual void disconnect() noexcept = 0;
};

class LibsshBackend final : public ISshBackend {
 public:
  LibsshBackend();
  ~LibsshBackend() override;
  LibsshBackend(const LibsshBackend&) = delete;
  LibsshBackend& operator=(const LibsshBackend&) = delete;

  ConnectResult connect(const SshConfig& config) override;
  bool write_all(const std::vector<std::uint8_t>& bytes) override;
  std::optional<std::vector<std::uint8_t>> read_some(
      std::chrono::milliseconds timeout) override;
  bool send_keepalive() override;
  void disconnect() noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class ISleeper {
 public:
  virtual ~ISleeper() = default;
  virtual bool sleep_for(std::chrono::milliseconds delay,
                         const std::atomic_bool& stop) = 0;
};

class InterruptibleSleeper final : public ISleeper {
 public:
  bool sleep_for(std::chrono::milliseconds delay,
                 const std::atomic_bool& stop) override;
};

class SshTunnel {
 public:
  using FrameCallback = std::function<void(Frame)>;
  using StateCallback =
      std::function<void(ConnectionState state, const std::string& detail)>;

  SshTunnel(SshConfig config, ISshBackend& backend, ISleeper& sleeper);
  ~SshTunnel();
  SshTunnel(const SshTunnel&) = delete;
  SshTunnel& operator=(const SshTunnel&) = delete;

  bool start(FrameCallback on_frame, StateCallback on_state);
  bool send(Frame frame);
  void stop();
  bool run_until_connected();
  ConnectionState state() const noexcept;

 private:
  void run();
  void set_state(ConnectionState state, const std::string& detail);
  void drop_pending_commands();

  SshConfig config_;
  ISshBackend& backend_;
  ISleeper& sleeper_;
  ThreadSafeQueue<Frame> outgoing_;
  std::atomic_bool stop_{false};
  std::atomic<ConnectionState> state_{ConnectionState::Stopped};
  std::atomic<std::uint32_t> ping_sequence_{1};
  mutable std::mutex callback_mutex_;
  FrameCallback on_frame_;
  StateCallback on_state_;
  std::thread worker_;
};

}  // namespace hypertron_ros2_bridge
