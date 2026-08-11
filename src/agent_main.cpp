#include <arpa/inet.h>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "hypertron_ros2_bridge/astrall_sdk_adapter.hpp"

namespace hypertron_ros2_bridge {
namespace {

volatile std::sig_atomic_t g_signal_received = 0;

extern "C" void handle_signal(int) { g_signal_received = 1; }

class StdioByteStream final : public IByteStream {
 public:
  std::vector<std::uint8_t> read_some() override {
    std::vector<std::uint8_t> bytes(64U * 1024U);
    const auto count = ::read(STDIN_FILENO, bytes.data(), bytes.size());
    if (count <= 0) {
      return {};
    }
    bytes.resize(static_cast<std::size_t>(count));
    return bytes;
  }

  bool write_all(const std::vector<std::uint8_t>& bytes) override {
    std::size_t offset{};
    while (offset < bytes.size()) {
      const auto count = ::write(STDOUT_FILENO, bytes.data() + offset,
                                 bytes.size() - offset);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        return false;
      }
      offset += static_cast<std::size_t>(count);
    }
    return true;
  }

  void close() noexcept override {
    if (!closed_.exchange(true)) {
      ::close(STDIN_FILENO);
      ::close(STDOUT_FILENO);
    }
  }

 private:
  std::atomic_bool closed_{false};
};

class UdpSocketSource final : public IUdpSource {
 public:
  UdpSocketSource(const std::string& bind_address, std::uint16_t port) {
    const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
      throw std::runtime_error("socket() failed for UDP " +
                               std::to_string(port));
    }
    fd_.store(socket_fd);
    int reuse = 1;
    ::setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1) {
      close();
      throw std::runtime_error("invalid UDP bind address: " + bind_address);
    }
    if (::bind(socket_fd, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0) {
      const auto detail = std::string(std::strerror(errno));
      close();
      throw std::runtime_error("bind UDP " + std::to_string(port) +
                               " failed: " + detail);
    }
  }

  ~UdpSocketSource() override { close(); }

  std::optional<UdpDatagram> receive(
      std::chrono::milliseconds timeout) override {
    const int socket_fd = fd_.load();
    if (socket_fd < 0) {
      return std::nullopt;
    }
    pollfd descriptor{socket_fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
    if (ready <= 0 || (descriptor.revents & POLLIN) == 0) {
      return std::nullopt;
    }
    UdpDatagram datagram;
    datagram.bytes.resize(65535U);
    const auto count =
        ::recv(socket_fd, datagram.bytes.data(), datagram.bytes.size(), 0);
    if (count <= 0) {
      return std::nullopt;
    }
    datagram.bytes.resize(static_cast<std::size_t>(count));
    datagram.receive_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    return datagram;
  }

  void close() noexcept override {
    const int socket_fd = fd_.exchange(-1);
    if (socket_fd >= 0) {
      ::shutdown(socket_fd, SHUT_RDWR);
      ::close(socket_fd);
    }
  }

 private:
  std::atomic<int> fd_{-1};
};

struct AgentFileSettings {
  AgentConfig agent;
  std::string lidar_bind{"0.0.0.0"};
  std::uint16_t point_cloud_port{6100};
  std::uint16_t odometry_port{6101};
  std::string camera_bind{"0.0.0.0"};
  std::uint16_t camera_port{6000};
  bool odometry_enabled{true};
};

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  value = value.substr(first, last - first + 1U);
  if (value.size() >= 2U &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1U, value.size() - 2U);
  }
  return value;
}

bool parse_bool(const std::string& value) {
  if (value == "true" || value == "True" || value == "1") return true;
  if (value == "false" || value == "False" || value == "0") return false;
  throw std::runtime_error("invalid boolean in agent config: " + value);
}

std::chrono::milliseconds period_from_hz(const std::string& value) {
  const double hz = std::stod(value);
  if (!std::isfinite(hz) || hz <= 0.0) {
    throw std::runtime_error("frequency must be positive");
  }
  return std::chrono::milliseconds(
      static_cast<std::int64_t>(std::llround(1000.0 / hz)));
}

std::uint16_t parse_port(const std::string& value) {
  const auto port = std::stoul(value);
  if (port == 0U || port > 65535U) {
    throw std::runtime_error("UDP port must be in [1, 65535]");
  }
  return static_cast<std::uint16_t>(port);
}

std::uint32_t parse_u32(const std::string& value, const char* field,
                        std::uint32_t maximum =
                            std::numeric_limits<std::uint32_t>::max()) {
  const auto parsed = std::stoull(value);
  if (parsed == 0U || parsed > maximum) {
    throw std::runtime_error(std::string(field) + " is outside its valid range");
  }
  return static_cast<std::uint32_t>(parsed);
}

AgentFileSettings load_config(const std::string& path) {
  AgentFileSettings settings;
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open agent config: " + path);
  }
  std::string section;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const auto key = trim(line.substr(0, colon));
    const auto value = trim(line.substr(colon + 1U));
    if (value.empty()) {
      if (key == "ssh" || key == "astrall" || key == "safety" ||
          key == "odometry" || key == "camera") {
        section = key;
      }
      continue;
    }
    if (section == "ssh" && key == "application_timeout_ms") {
      settings.agent.application_timeout =
          std::chrono::milliseconds(std::stoll(value));
    } else if (section == "astrall" && key == "sdk_timeout_ms") {
      settings.agent.sdk_call_timeout_ms = parse_u32(value, "sdk_timeout_ms");
    } else if (section == "astrall" && key == "initialization_timeout_ms") {
      settings.agent.init_timeout_ms =
          parse_u32(value, "initialization_timeout_ms");
    } else if (section == "astrall" && key == "heartbeat_hz") {
      settings.agent.heartbeat_period = period_from_hz(value);
    } else if (section == "astrall" && key == "motion_refresh_hz") {
      settings.agent.motion_period = period_from_hz(value);
    } else if (section == "astrall" && key == "state_poll_hz") {
      settings.agent.state_period = period_from_hz(value);
    } else if (section == "astrall" && key == "auto_acquire_control") {
      settings.agent.auto_acquire_control = parse_bool(value);
    } else if (section == "safety" && key == "command_deadman_ms") {
      settings.agent.command_deadman =
          std::chrono::milliseconds(std::stoll(value));
    } else if (section == "safety" && key == "mode_timeout_ms") {
      settings.agent.mode_timeout =
          std::chrono::milliseconds(std::stoll(value));
    } else if (section == "safety" && key == "telemetry_queue_capacity") {
      settings.agent.output_queue_capacity =
          parse_u32(value, "telemetry_queue_capacity", 1000000U);
    } else if (section == "safety" && key == "max_payload_bytes") {
      settings.agent.max_payload =
          parse_u32(value, "max_payload_bytes", 64U * 1024U * 1024U);
    } else if (section == "odometry" && key == "enabled") {
      settings.odometry_enabled = parse_bool(value);
    } else if (section == "odometry" && key == "bind_address") {
      settings.lidar_bind = value;
    } else if (section == "odometry" && key == "point_cloud_port") {
      settings.point_cloud_port = parse_port(value);
    } else if (section == "odometry" && key == "odometry_port") {
      settings.odometry_port = parse_port(value);
    } else if (section == "odometry" && key == "packing") {
      settings.agent.lidar_packing =
          value == "packed"     ? PackingMode::Packed
          : value == "aligned"  ? PackingMode::NaturalAligned
                                : PackingMode::Auto;
    } else if (section == "odometry" && key == "position_scale") {
      settings.agent.odometry_position_scale = std::stod(value);
    } else if (section == "odometry" && key == "quaternion_scale") {
      settings.agent.odometry_quaternion_scale = std::stod(value);
    } else if (section == "odometry" && key == "point_position_scale") {
      settings.agent.point_coordinate_scale = std::stod(value);
    } else if (section == "odometry" && key == "scale_verified") {
      settings.agent.odometry_scale_verified = parse_bool(value);
    } else if (section == "camera" && key == "enabled") {
      settings.agent.camera_enabled = parse_bool(value);
    } else if (section == "camera" && key == "bind_address") {
      settings.camera_bind = value;
    } else if (section == "camera" && key == "port") {
      settings.camera_port = parse_port(value);
    }
  }
  return settings;
}

void install_signal_handlers() {
  struct sigaction action {};
  action.sa_handler = handle_signal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  struct sigaction ignore_pipe {};
  ignore_pipe.sa_handler = SIG_IGN;
  sigemptyset(&ignore_pipe.sa_mask);
  sigaction(SIGPIPE, &ignore_pipe, nullptr);
}

}  // namespace
}  // namespace hypertron_ros2_bridge

int main(int argc, char** argv) {
  using namespace hypertron_ros2_bridge;
  std::string config_path{"/opt/hypertron/config/bridge_config.yaml"};
  bool stdio{};
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--stdio") {
      stdio = true;
    } else if (argument == "--config" && index + 1 < argc) {
      config_path = argv[++index];
    } else if (argument == "--help") {
      std::cerr << "Usage: hypertron_bridge_agent --stdio [--config PATH]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << argument << '\n';
      return 64;
    }
  }
  if (!stdio) {
    std::cerr << "--stdio is required; stdout is reserved for HTBR frames\n";
    return 64;
  }

  try {
    const auto settings = load_config(config_path);
    if (!settings.agent.odometry_scale_verified) {
      std::cerr << "WARNING: UDP 6101 odometry scale is not vendor-verified\n";
    }
    install_signal_handlers();
    StdioByteStream stream;
    SteadyMonotonicClock clock;
    AstrallSdkAdapter sdk;
    std::unique_ptr<UdpSocketSource> lidar;
    std::unique_ptr<UdpSocketSource> odometry;
    std::unique_ptr<UdpSocketSource> camera;
    if (settings.odometry_enabled) {
      lidar = std::make_unique<UdpSocketSource>(settings.lidar_bind,
                                                settings.point_cloud_port);
      odometry = std::make_unique<UdpSocketSource>(settings.lidar_bind,
                                                   settings.odometry_port);
    }
    if (settings.agent.camera_enabled) {
      camera = std::make_unique<UdpSocketSource>(settings.camera_bind,
                                                 settings.camera_port);
    }
    AgentUdpSources sources{camera.get(), lidar.get(), odometry.get()};
    AgentRuntime runtime(settings.agent, sdk, stream, clock, sources);
    return runtime.run();
  } catch (const std::exception& error) {
    std::cerr << "hypertron_bridge_agent: " << error.what() << '\n';
    return 1;
  }
}
