#include "hypertron_ros2_bridge/ssh_tunnel.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef HYPERTRON_WITH_LIBSSH
#include <libssh/libssh.h>
#endif

namespace hypertron_ros2_bridge {
namespace detail {

LibsshReadDisposition classify_libssh_read(int count) noexcept {
  if (count > 0) return LibsshReadDisposition::Data;
  if (count == -2) return LibsshReadDisposition::Again;
  if (count == 0 || count == -127) return LibsshReadDisposition::Eof;
  return LibsshReadDisposition::Error;
}

}  // namespace detail
namespace {

#ifdef HYPERTRON_WITH_LIBSSH
static_assert(SSH_AGAIN == -2, "libssh SSH_AGAIN value changed");
static_assert(SSH_EOF == -127, "libssh SSH_EOF value changed");

std::string expand_user_path(const std::string& path) {
  if (path.size() < 2U || path[0] != '~' || path[1] != '/') {
    return path;
  }
  const char* home = std::getenv("HOME");
  return home == nullptr ? path : std::string(home) + path.substr(1U);
}
#endif

}  // namespace

struct LibsshBackend::Impl {
#ifdef HYPERTRON_WITH_LIBSSH
  ssh_session session{};
  ssh_channel channel{};
#endif
};

LibsshBackend::LibsshBackend() : impl_(std::make_unique<Impl>()) {}
LibsshBackend::~LibsshBackend() { disconnect(); }

ConnectResult LibsshBackend::connect(const SshConfig& config) {
  disconnect();
#ifdef HYPERTRON_WITH_LIBSSH
  impl_->session = ssh_new();
  if (impl_->session == nullptr) return ConnectResult::RetryableFailure;

  const unsigned int port = config.port;
  const long timeout_seconds =
      std::max<long>(1, (config.connect_timeout.count() + 999) / 1000);
  const auto known_hosts = expand_user_path(config.known_hosts);
  if (ssh_options_set(impl_->session, SSH_OPTIONS_HOST,
                      config.host.c_str()) != SSH_OK ||
      ssh_options_set(impl_->session, SSH_OPTIONS_PORT, &port) != SSH_OK ||
      ssh_options_set(impl_->session, SSH_OPTIONS_USER,
                      config.username.c_str()) != SSH_OK ||
      ssh_options_set(impl_->session, SSH_OPTIONS_TIMEOUT,
                      &timeout_seconds) != SSH_OK ||
      (!known_hosts.empty() &&
       ssh_options_set(impl_->session, SSH_OPTIONS_KNOWNHOSTS,
                       known_hosts.c_str()) != SSH_OK)) {
    disconnect();
    return ConnectResult::PermanentFailure;
  }
  if (ssh_connect(impl_->session) != SSH_OK) {
    disconnect();
    return ConnectResult::RetryableFailure;
  }

  const auto known = ssh_session_is_known_server(impl_->session);
  if (known != SSH_KNOWN_HOSTS_OK) {
    disconnect();
    return ConnectResult::PermanentFailure;
  }

  int auth = SSH_AUTH_DENIED;
  if (!config.private_key.empty()) {
    ssh_key private_key{};
    if (ssh_pki_import_privkey_file(expand_user_path(config.private_key).c_str(),
                                    nullptr, nullptr, nullptr,
                                    &private_key) == SSH_OK) {
      auth = ssh_userauth_publickey(impl_->session, nullptr, private_key);
      ssh_key_free(private_key);
    }
  }
  if (auth != SSH_AUTH_SUCCESS) {
    auth = ssh_userauth_publickey_auto(impl_->session, nullptr, nullptr);
  }
  if (auth != SSH_AUTH_SUCCESS && !config.password.empty()) {
    auth = ssh_userauth_password(impl_->session, nullptr,
                                 config.password.c_str());
  }
  if (auth != SSH_AUTH_SUCCESS) {
    disconnect();
    return ConnectResult::PermanentFailure;
  }

  impl_->channel = ssh_channel_new(impl_->session);
  if (impl_->channel == nullptr ||
      ssh_channel_open_session(impl_->channel) != SSH_OK ||
      config.remote_command.empty() ||
      ssh_channel_request_exec(impl_->channel,
                               config.remote_command.c_str()) != SSH_OK) {
    disconnect();
    return ConnectResult::RetryableFailure;
  }
  return ConnectResult::Success;
#else
  (void)config;
  return ConnectResult::PermanentFailure;
#endif
}

bool LibsshBackend::write_all(const std::vector<std::uint8_t>& bytes) {
#ifdef HYPERTRON_WITH_LIBSSH
  if (impl_->channel == nullptr) return false;
  std::size_t offset{};
  while (offset < bytes.size()) {
    const auto count = ssh_channel_write(impl_->channel, bytes.data() + offset,
                                         bytes.size() - offset);
    if (count < 0 || count == 0) return false;
    offset += static_cast<std::size_t>(count);
  }
  return true;
#else
  (void)bytes;
  return false;
#endif
}

std::optional<std::vector<std::uint8_t>> LibsshBackend::read_some(
    std::chrono::milliseconds timeout) {
#ifdef HYPERTRON_WITH_LIBSSH
  if (impl_->channel == nullptr) return std::nullopt;

  // Always drain agent stderr, including when stdout is idle. Otherwise a
  // verbose diagnostic stream can fill the SSH window and stall binary data.
  while (true) {
    const auto stderr_available =
        ssh_channel_poll_timeout(impl_->channel, 0, 1);
    if (stderr_available <= 0) break;
    std::vector<std::uint8_t> discarded(
        std::min<std::size_t>(static_cast<std::size_t>(stderr_available),
                              4096U));
    if (ssh_channel_read_nonblocking(impl_->channel, discarded.data(),
                                     discarded.size(), 1) <= 0) {
      break;
    }
  }
  const auto available = ssh_channel_poll_timeout(
      impl_->channel, static_cast<int>(timeout.count()), 0);
  if (available < 0) return std::nullopt;
  if (available == 0) {
    return ssh_channel_is_eof(impl_->channel) != 0
               ? std::optional<std::vector<std::uint8_t>>{}
               : std::optional<std::vector<std::uint8_t>>(
                     std::vector<std::uint8_t>{});
  }
  std::vector<std::uint8_t> bytes(
      std::min<std::size_t>(static_cast<std::size_t>(available), 64U * 1024U));
  const auto count = ssh_channel_read_nonblocking(
      impl_->channel, bytes.data(), bytes.size(), 0);
  switch (detail::classify_libssh_read(count)) {
    case detail::LibsshReadDisposition::Data:
      break;
    case detail::LibsshReadDisposition::Again:
      return std::vector<std::uint8_t>{};
    case detail::LibsshReadDisposition::Eof:
    case detail::LibsshReadDisposition::Error:
      return std::nullopt;
  }
  bytes.resize(static_cast<std::size_t>(count));

  return bytes;
#else
  (void)timeout;
  return std::nullopt;
#endif
}

bool LibsshBackend::send_keepalive() {
#ifdef HYPERTRON_WITH_LIBSSH
  // Client-side SSH_MSG_IGNORE traffic keeps NAT/session state active on
  // libssh 0.9.x; HTBR PING/PONG separately proves the agent is responsive.
  return impl_->session != nullptr &&
         ssh_send_ignore(impl_->session, "hypertron-keepalive") == SSH_OK;
#else
  return false;
#endif
}

void LibsshBackend::disconnect() noexcept {
#ifdef HYPERTRON_WITH_LIBSSH
  if (impl_->channel != nullptr) {
    ssh_channel_send_eof(impl_->channel);
    ssh_channel_close(impl_->channel);
    ssh_channel_free(impl_->channel);
    impl_->channel = nullptr;
  }
  if (impl_->session != nullptr) {
    ssh_disconnect(impl_->session);
    ssh_free(impl_->session);
    impl_->session = nullptr;
  }
#endif
}

bool InterruptibleSleeper::sleep_for(std::chrono::milliseconds delay,
                                     const std::atomic_bool& stop) {
  using namespace std::chrono_literals;
  auto remaining = delay;
  while (remaining > 0ms && !stop.load()) {
    const auto slice = std::min(remaining, 50ms);
    std::this_thread::sleep_for(slice);
    remaining -= slice;
  }
  return !stop.load();
}

SshTunnel::SshTunnel(SshConfig config, ISshBackend& backend,
                     ISleeper& sleeper)
    : config_(std::move(config)),
      backend_(backend),
      sleeper_(sleeper),
      priority_outgoing_(config_.queue_capacity, OverflowPolicy::RejectNew),
      regular_outgoing_(config_.queue_capacity, OverflowPolicy::RejectNew) {
  if (const char* password = std::getenv("HYPERTRON_SSH_PASSWORD");
      password != nullptr && *password != '\0') {
    config_.password = password;
  }
  if (config_.host.empty() || config_.username.empty() ||
      config_.remote_command.empty()) {
    throw std::invalid_argument(
        "SSH host, username and remote command are required");
  }
  if (!config_.strict_host_key_checking) {
    throw std::invalid_argument(
        "strict SSH host-key checking cannot be disabled");
  }
  if (config_.connect_timeout.count() <= 0 ||
      config_.keepalive_interval.count() <= 0 ||
      config_.ping_interval.count() <= 0 ||
      config_.application_timeout <= config_.ping_interval ||
      config_.agent_startup_timeout < config_.application_timeout ||
      config_.reconnect_initial_delay.count() <= 0 ||
      config_.reconnect_max_delay < config_.reconnect_initial_delay ||
      config_.max_payload == 0U) {
    throw std::invalid_argument("SSH timeout, retry, or payload settings invalid");
  }
}

SshTunnel::~SshTunnel() { stop(); }

bool SshTunnel::start(FrameCallback on_frame, StateCallback on_state) {
  if (stop_invoked_.load() || started_.exchange(true) || worker_.joinable()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    on_frame_ = std::move(on_frame);
    on_state_ = std::move(on_state);
  }
  stop_.store(false);
  stop_invoked_.store(false);
  worker_ = std::thread([this] {
    try {
      run();
    } catch (...) {
      backend_.disconnect();
      try {
        drop_pending_commands();
      } catch (...) {
      }
      state_.store(ConnectionState::Disconnected);
      try {
        set_state(ConnectionState::Disconnected,
                  "SSH worker stopped after an unhandled exception");
      } catch (...) {
      }
    }
  });
  return true;
}

bool SshTunnel::send(Frame frame) {
  if (state_.load() != ConnectionState::Connected || stop_.load()) {
    return false;
  }
  if (frame.type == MessageType::CmdEstop) {
    return priority_outgoing_.push(std::move(frame));
  }
  if (frame.type == MessageType::CmdVelocity) {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    latest_velocity_ = std::move(frame);
    return true;
  }
  return regular_outgoing_.push(std::move(frame));
}

void SshTunnel::stop() {
  if (stop_invoked_.exchange(true)) return;
  stop_.store(true);
  priority_outgoing_.close();
  regular_outgoing_.close();
  if (worker_.joinable()) worker_.join();
  // The backend is worker-owned while the worker exists. Disconnect only
  // after join to avoid freeing a libssh session during read/write.
  backend_.disconnect();
  set_state(ConnectionState::Stopped, "SSH tunnel stopped");
}

bool SshTunnel::run_until_connected() {
  auto delay = config_.reconnect_initial_delay;
  while (!stop_.load()) {
    set_state(ConnectionState::Connecting, "connecting to SSH agent");
    const auto result = backend_.connect(config_);
    if (result == ConnectResult::Success) {
      set_state(ConnectionState::Connected, "SSH agent channel connected");
      return true;
    }
    if (result == ConnectResult::PermanentFailure) {
      set_state(ConnectionState::Failed,
                "SSH authentication, host key, or configuration failed");
      return false;
    }
    set_state(ConnectionState::Disconnected, "SSH connection failed; retrying");
    if (!sleeper_.sleep_for(delay, stop_)) return false;
    delay = std::min(delay * 2, config_.reconnect_max_delay);
  }
  return false;
}

ConnectionState SshTunnel::state() const noexcept { return state_.load(); }

std::uint32_t SshTunnel::protocol_drops() const noexcept {
  return protocol_drops_.load();
}

void SshTunnel::run() {
  auto reconnect_delay = config_.reconnect_initial_delay;
  while (!stop_.load()) {
    try {
    if (!run_until_connected()) {
      if (state_.load() == ConnectionState::Failed) return;
      continue;
    }
    reconnect_delay = config_.reconnect_initial_delay;
    ProtocolHandler decoder(config_.max_payload);
    auto last_pong = std::chrono::steady_clock::now();
    auto last_ping = last_pong;
    auto last_keepalive = last_pong;
    bool received_pong = false;
    bool channel_ok = true;
    std::string channel_failure_detail;
    while (!stop_.load() && channel_ok) {
      while (auto frame = priority_outgoing_.try_pop()) {
        if (!backend_.write_all(ProtocolHandler::encode(*frame))) {
          channel_ok = false;
          break;
        }
      }
      std::optional<Frame> velocity;
      {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        velocity.swap(latest_velocity_);
      }
      if (channel_ok && velocity &&
          !backend_.write_all(ProtocolHandler::encode(*velocity))) {
        channel_ok = false;
      }
      if (channel_ok) {
        if (auto frame = regular_outgoing_.try_pop(); frame &&
            !backend_.write_all(ProtocolHandler::encode(*frame))) {
          channel_ok = false;
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (channel_ok && now - last_ping >= config_.ping_interval) {
        const Frame ping{MessageType::Ping, ping_sequence_.fetch_add(1),
                         static_cast<std::uint64_t>(
                             std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 now.time_since_epoch())
                                 .count()),
                         {}};
        channel_ok = backend_.write_all(ProtocolHandler::encode(ping));
        last_ping = now;
      }
      if (channel_ok && now - last_keepalive >= config_.keepalive_interval) {
        channel_ok = backend_.send_keepalive();
        last_keepalive = now;
      }
      if (channel_ok) {
        const auto bytes = backend_.read_some(std::chrono::milliseconds(20));
        if (!bytes) {
          channel_ok = false;
        } else if (!bytes->empty()) {
          try {
            for (auto& frame : decoder.feed(*bytes)) {
              if (frame.type == MessageType::Pong) {
                last_pong = now;
                received_pong = true;
              }
              FrameCallback callback;
              {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                callback = on_frame_;
              }
              if (callback) {
                try {
                  callback(std::move(frame));
                } catch (const std::exception& error) {
                  protocol_drops_.fetch_add(1);
                  channel_failure_detail =
                      std::string("frame callback rejected payload: ") +
                      error.what();
                  channel_ok = false;
                  break;
                } catch (...) {
                  protocol_drops_.fetch_add(1);
                  channel_failure_detail =
                      "frame callback rejected payload: unknown failure";
                  channel_ok = false;
                  break;
                }
              }
            }
          } catch (const ProtocolError&) {
            protocol_drops_.fetch_add(1);
            channel_failure_detail = "HTBR frame decoding failed";
            channel_ok = false;
          }
        }
      }
      const auto liveness_timeout =
          received_pong ? config_.application_timeout
                        : config_.agent_startup_timeout;
      if (now - last_pong > liveness_timeout) channel_ok = false;
    }
    state_.store(ConnectionState::Disconnected);
    backend_.disconnect();
    drop_pending_commands();
    set_state(ConnectionState::Disconnected,
              channel_failure_detail.empty()
                  ? "SSH agent channel disconnected; commands cleared"
                  : channel_failure_detail);
    if (!stop_.load() &&
        !sleeper_.sleep_for(reconnect_delay, stop_)) {
      break;
    }
    reconnect_delay =
        std::min(reconnect_delay * 2, config_.reconnect_max_delay);
    } catch (const std::exception& error) {
      backend_.disconnect();
      drop_pending_commands();
      set_state(ConnectionState::Disconnected,
                std::string("SSH worker exception: ") + error.what());
      if (!stop_.load()) {
        if (!sleeper_.sleep_for(reconnect_delay, stop_)) break;
        reconnect_delay =
            std::min(reconnect_delay * 2, config_.reconnect_max_delay);
      }
    } catch (...) {
      backend_.disconnect();
      drop_pending_commands();
      set_state(ConnectionState::Disconnected,
                "SSH worker exception: unknown failure");
      stop_.store(true);
    }
  }
}

void SshTunnel::set_state(ConnectionState state, const std::string& detail) {
  state_.store(state);
  StateCallback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = on_state_;
  }
  if (callback) {
    try {
      callback(state, detail);
    } catch (...) {
      // User callbacks must not terminate the transport worker.
    }
  }
}

void SshTunnel::drop_pending_commands() {
  while (priority_outgoing_.try_pop()) {
  }
  while (regular_outgoing_.try_pop()) {
  }
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_velocity_.reset();
}

}  // namespace hypertron_ros2_bridge
