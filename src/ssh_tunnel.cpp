#include "hypertron_ros2_bridge/ssh_tunnel.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef HYPERTRON_WITH_LIBSSH
#include <libssh/libssh.h>
#endif

namespace hypertron_ros2_bridge {
namespace {

std::string expand_user_path(const std::string& path) {
  if (path.size() < 2U || path[0] != '~' || path[1] != '/') {
    return path;
  }
  const char* home = std::getenv("HOME");
  return home == nullptr ? path : std::string(home) + path.substr(1U);
}

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
    if (config.strict_host_key_checking ||
        (known != SSH_KNOWN_HOSTS_UNKNOWN &&
         known != SSH_KNOWN_HOSTS_NOT_FOUND)) {
      disconnect();
      return ConnectResult::PermanentFailure;
    }
    if (ssh_session_update_known_hosts(impl_->session) != SSH_OK) {
      disconnect();
      return ConnectResult::PermanentFailure;
    }
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
    if (count == SSH_ERROR || count == 0) return false;
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
  const auto available = ssh_channel_poll_timeout(
      impl_->channel, static_cast<int>(timeout.count()), 0);
  if (available == SSH_ERROR) return std::nullopt;
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
  if (count == SSH_ERROR || count == 0) return std::nullopt;
  bytes.resize(static_cast<std::size_t>(count));

  // Drain agent stderr separately. It is diagnostic only and must never be
  // mixed into the binary HTBR stdout stream.
  const auto stderr_available = ssh_channel_poll_timeout(impl_->channel, 0, 1);
  if (stderr_available > 0) {
    std::vector<std::uint8_t> discarded(
        std::min<std::size_t>(static_cast<std::size_t>(stderr_available), 4096U));
    ssh_channel_read_nonblocking(impl_->channel, discarded.data(),
                                 discarded.size(), 1);
  }
  return bytes;
#else
  (void)timeout;
  return std::nullopt;
#endif
}

bool LibsshBackend::send_keepalive() {
#ifdef HYPERTRON_WITH_LIBSSH
  return impl_->session != nullptr && ssh_send_keepalive(impl_->session) == SSH_OK;
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
      outgoing_(config_.queue_capacity, OverflowPolicy::RejectNew) {
  if (const char* password = std::getenv("HYPERTRON_SSH_PASSWORD");
      password != nullptr && *password != '\0') {
    config_.password = password;
  }
  if (config_.host.empty() || config_.username.empty() ||
      config_.remote_command.empty()) {
    throw std::invalid_argument(
        "SSH host, username and remote command are required");
  }
}

SshTunnel::~SshTunnel() { stop(); }

bool SshTunnel::start(FrameCallback on_frame, StateCallback on_state) {
  if (worker_.joinable()) return false;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    on_frame_ = std::move(on_frame);
    on_state_ = std::move(on_state);
  }
  stop_.store(false);
  worker_ = std::thread([this] { run(); });
  return true;
}

bool SshTunnel::send(Frame frame) {
  if (state_.load() != ConnectionState::Connected || stop_.load()) {
    return false;
  }
  return outgoing_.push(std::move(frame));
}

void SshTunnel::stop() {
  stop_.store(true);
  outgoing_.close();
  backend_.disconnect();
  if (worker_.joinable()) worker_.join();
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

void SshTunnel::run() {
  auto reconnect_delay = config_.reconnect_initial_delay;
  while (!stop_.load()) {
    if (!run_until_connected()) {
      if (state_.load() == ConnectionState::Failed) return;
      continue;
    }
    reconnect_delay = config_.reconnect_initial_delay;
    ProtocolHandler decoder(config_.max_payload);
    auto last_pong = std::chrono::steady_clock::now();
    auto last_ping = last_pong;
    auto last_keepalive = last_pong;
    bool channel_ok = true;
    while (!stop_.load() && channel_ok) {
      while (auto frame = outgoing_.try_pop()) {
        if (!backend_.write_all(ProtocolHandler::encode(*frame))) {
          channel_ok = false;
          break;
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
              if (frame.type == MessageType::Pong) last_pong = now;
              FrameCallback callback;
              {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                callback = on_frame_;
              }
              if (callback) callback(std::move(frame));
            }
          } catch (const ProtocolError&) {
            channel_ok = false;
          }
        }
      }
      if (now - last_pong > config_.application_timeout) channel_ok = false;
    }
    state_.store(ConnectionState::Disconnected);
    backend_.disconnect();
    drop_pending_commands();
    set_state(ConnectionState::Disconnected,
              "SSH agent channel disconnected; commands cleared");
    if (!stop_.load() &&
        !sleeper_.sleep_for(reconnect_delay, stop_)) {
      break;
    }
    reconnect_delay =
        std::min(reconnect_delay * 2, config_.reconnect_max_delay);
  }
}

void SshTunnel::set_state(ConnectionState state, const std::string& detail) {
  state_.store(state);
  StateCallback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = on_state_;
  }
  if (callback) callback(state, detail);
}

void SshTunnel::drop_pending_commands() {
  while (outgoing_.try_pop()) {
  }
}

}  // namespace hypertron_ros2_bridge
