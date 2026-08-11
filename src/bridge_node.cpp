#include "hypertron_ros2_bridge/bridge_node.hpp"

#ifdef HYPERTRON_WITH_ROS2

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "hypertron_ros2_bridge/data_receiver.hpp"
#include "hypertron_ros2_bridge/protocol_handler.hpp"
#include "hypertron_ros2_bridge/robot_controller.hpp"
#include "hypertron_ros2_bridge/ssh_tunnel.hpp"

namespace hypertron_ros2_bridge {
namespace {

std::uint64_t steady_now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

TimestampSource timestamp_source(const std::string& value) {
  if (value == "device_ns") return TimestampSource::DeviceNanoseconds;
  if (value == "device_us") return TimestampSource::DeviceMicroseconds;
  if (value == "device_ms") return TimestampSource::DeviceMilliseconds;
  return TimestampSource::Receive;
}

std::array<double, 3> covariance3(const std::vector<double>& values,
                                  const std::array<double, 3>& fallback) {
  if (values.size() != 3U) return fallback;
  return {values[0], values[1], values[2]};
}

struct RequestResult {
  bool success{};
  std::uint16_t result_code{};
  std::string message;
};

}  // namespace

struct HypertronBridgeNode::Impl {
  explicit Impl(HypertronBridgeNode& node_value)
      : node(node_value), backend(), sleeper(), clock() {
    const auto ssh_config = read_ssh_config();
    const auto receiver_config = read_receiver_config();
    const auto deadman_ms = node.declare_parameter<int>(
        "safety.command_deadman_ms", 100);
    mode_timeout = std::chrono::milliseconds(
        node.declare_parameter<int>("safety.mode_timeout_ms", 10000));
    controller = std::make_unique<RobotController>(
        ControllerConfig{std::chrono::milliseconds(deadman_ms)}, clock);
    receiver = std::make_unique<DataReceiver>(node, receiver_config);
    tunnel = std::make_unique<SshTunnel>(ssh_config, backend, sleeper);

    const auto cmd_vel_topic = node.declare_parameter<std::string>(
        "topics.cmd_vel", "/cmd_vel");
    const auto joint_topic = node.declare_parameter<std::string>(
        "topics.joint_commands", "/joint_commands");
    const auto mode_topic = node.declare_parameter<std::string>(
        "topics.robot_mode", "/robot_mode");
    const auto estop_service = node.declare_parameter<std::string>(
        "topics.emergency_stop", "/emergency_stop");

    cmd_vel = node.create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          on_velocity(*message);
        });
    joint_commands = node.create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          on_joint_command(*message);
        });
    robot_mode = node.create_subscription<std_msgs::msg::String>(
        mode_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          on_mode(message->data);
        });
    emergency_stop = node.create_service<std_srvs::srv::SetBool>(
        estop_service,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          on_emergency_stop(request->data, *response);
        });

    publish_connection_state();
    if (!receiver_config.odometry_scale_verified) {
      RCLCPP_WARN(node.get_logger(),
                  "UDP 6101 odometry scale is not vendor-verified; "
                  "RobotState.odometry_scale_verified remains false");
    }
    if (!tunnel->start(
            [this](Frame frame) { on_frame(std::move(frame)); },
            [this](ConnectionState state, const std::string& detail) {
              on_tunnel_state(state, detail);
            })) {
      throw std::runtime_error("failed to start SSH tunnel worker");
    }
  }

  ~Impl() {
    if (tunnel) tunnel->stop();
    fail_pending("bridge node is shutting down");
  }

  HypertronBridgeNode& node;
  LibsshBackend backend;
  InterruptibleSleeper sleeper;
  SteadyMonotonicClock clock;
  std::unique_ptr<RobotController> controller;
  std::unique_ptr<DataReceiver> receiver;
  std::unique_ptr<SshTunnel> tunnel;
  std::chrono::milliseconds mode_timeout{10000};
  std::atomic<std::uint32_t> next_sequence{1};
  std::mutex state_mutex;
  ReceiverConnectionState connection;
  std::mutex pending_mutex;
  std::unordered_map<std::uint32_t,
                     std::shared_ptr<std::promise<RequestResult>>>
      pending;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_mode;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop;

  SshConfig read_ssh_config() {
    SshConfig config;
    config.host = node.declare_parameter<std::string>("ssh.host", "");
    const auto port = node.declare_parameter<int>("ssh.port", 22);
    if (port <= 0 || port > 65535) {
      throw std::invalid_argument("ssh.port must be in [1, 65535]");
    }
    config.port = static_cast<std::uint16_t>(port);
    config.username =
        node.declare_parameter<std::string>("ssh.username", "robot");
    config.password =
        node.declare_parameter<std::string>("ssh.password", "");
    config.private_key = node.declare_parameter<std::string>(
        "ssh.private_key", "~/.ssh/id_ed25519");
    config.known_hosts = node.declare_parameter<std::string>(
        "ssh.known_hosts", "~/.ssh/known_hosts");
    config.strict_host_key_checking = node.declare_parameter<bool>(
        "ssh.strict_host_key_checking", true);
    config.remote_command = node.declare_parameter<std::string>(
        "ssh.remote_command",
        "/opt/hypertron/bin/hypertron_bridge_agent --stdio --config "
        "/opt/hypertron/config/bridge_config.yaml");
    config.connect_timeout = std::chrono::milliseconds(
        node.declare_parameter<int>("ssh.connect_timeout_ms", 5000));
    config.keepalive_interval = std::chrono::milliseconds(
        node.declare_parameter<int>("ssh.keepalive_interval_ms", 1000));
    config.ping_interval = std::chrono::milliseconds(
        node.declare_parameter<int>("ssh.ping_interval_ms", 200));
    config.application_timeout = std::chrono::milliseconds(
        node.declare_parameter<int>("ssh.application_timeout_ms", 500));
    config.reconnect_initial_delay = std::chrono::milliseconds(
        node.declare_parameter<int>("ssh.reconnect_initial_delay_ms", 1000));
    config.reconnect_max_delay = std::chrono::milliseconds(
        node.declare_parameter<int>("ssh.reconnect_max_delay_ms", 30000));
    config.queue_capacity = static_cast<std::size_t>(
        node.declare_parameter<int>("safety.queue_capacity", 64));
    config.max_payload = static_cast<std::uint32_t>(
        node.declare_parameter<int>("safety.max_payload_bytes", 8388608));
    return config;
  }

  DataReceiverConfig read_receiver_config() {
    DataReceiverConfig config;
    config.imu_frame =
        node.declare_parameter<std::string>("frames.imu", "imu_link");
    config.odom_frame =
        node.declare_parameter<std::string>("frames.odom", "odom");
    config.base_frame =
        node.declare_parameter<std::string>("frames.base", "base_link");
    config.camera_frame = node.declare_parameter<std::string>(
        "frames.camera", "camera_link");
    config.imu_topic =
        node.declare_parameter<std::string>("topics.imu", "/imu/data");
    config.joint_states_topic = node.declare_parameter<std::string>(
        "topics.joint_states", "/joint_states");
    config.odom_topic =
        node.declare_parameter<std::string>("topics.odom", "/odom");
    config.robot_state_topic = node.declare_parameter<std::string>(
        "topics.robot_state", "/robot_state");
    config.camera_topic = node.declare_parameter<std::string>(
        "topics.camera", "/camera/image_raw");
    config.timestamp_source = timestamp_source(node.declare_parameter<std::string>(
        "imu.timestamp_source", "receive"));
    config.quaternion_order =
        node.declare_parameter<std::string>("imu.quaternion_order", "xyzw") ==
                "wxyz"
            ? QuaternionOrder::Wxyz
            : QuaternionOrder::Xyzw;
    config.orientation_covariance_diagonal = covariance3(
        node.declare_parameter<std::vector<double>>(
            "imu.orientation_covariance_diagonal", {0.02, 0.02, 0.05}),
        config.orientation_covariance_diagonal);
    config.angular_velocity_covariance_diagonal = covariance3(
        node.declare_parameter<std::vector<double>>(
            "imu.angular_velocity_covariance_diagonal", {0.01, 0.01, 0.02}),
        config.angular_velocity_covariance_diagonal);
    config.linear_acceleration_covariance_diagonal = covariance3(
        node.declare_parameter<std::vector<double>>(
            "imu.linear_acceleration_covariance_diagonal", {0.10, 0.10, 0.20}),
        config.linear_acceleration_covariance_diagonal);
    config.camera_enabled =
        node.declare_parameter<bool>("camera.enabled", false);
    config.odometry_scale_verified =
        node.declare_parameter<bool>("odometry.scale_verified", false);
    return config;
  }

  Frame command_frame(MessageType type, std::vector<std::uint8_t> payload) {
    return {type, next_sequence.fetch_add(1), steady_now_ns(),
            std::move(payload)};
  }

  void on_velocity(const geometry_msgs::msg::Twist& message) {
    const auto decision = controller->accept_velocity(
        {static_cast<float>(message.linear.x),
         static_cast<float>(message.linear.y),
         static_cast<float>(message.angular.z)});
    if (!decision.accepted) {
      update_error(decision.reason);
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                           "Rejected /cmd_vel: %s", decision.reason.c_str());
      return;
    }
    if (!tunnel->send(command_frame(MessageType::CmdVelocity,
                                    encode_velocity(decision.value)))) {
      update_error("/cmd_vel dropped because SSH agent is disconnected");
    }
  }

  void on_joint_command(const sensor_msgs::msg::JointState&) {
    // TODO:参考手册第23-26页补充实现：厂家提供关节SDK后定义名称、单位、限位和反馈。
    controller->reject_joint_command("ASTRALL 1.0.7 has no joint API");
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      connection.rejected_joint_commands =
          controller->rejected_joint_commands();
      connection.last_error =
          "joint command rejected: vendor low-level SDK is unavailable";
    }
    publish_connection_state();
    RCLCPP_WARN_THROTTLE(
        node.get_logger(), *node.get_clock(), 5000,
        "Rejected /joint_commands: ASTRALL SDK 1.0.7 exposes no joint API");
  }

  void on_mode(const std::string& name) {
    const auto decision = controller->request_mode(name);
    if (!decision.accepted) {
      update_error(decision.reason);
      RCLCPP_WARN(node.get_logger(), "Rejected /robot_mode '%s': %s",
                  name.c_str(), decision.reason.c_str());
      return;
    }
    if (!tunnel->send(command_frame(MessageType::CmdMode,
                                    encode_mode({decision.mode})))) {
      update_error("robot mode dropped because SSH agent is disconnected");
    }
  }

  void on_emergency_stop(bool engage,
                         std_srvs::srv::SetBool::Response& response) {
    if (engage) controller->trigger_estop();
    auto promise = std::make_shared<std::promise<RequestResult>>();
    auto future = promise->get_future();
    auto frame = command_frame(MessageType::CmdEstop, encode_estop({engage}));
    const auto request_sequence = frame.sequence;
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      pending.emplace(frame.sequence, promise);
    }
    if (!tunnel->send(std::move(frame))) {
      std::lock_guard<std::mutex> lock(pending_mutex);
      pending.erase(request_sequence);
      response.success = false;
      response.message = "SSH agent is disconnected";
      return;
    }
    if (future.wait_for(mode_timeout) != std::future_status::ready) {
      std::lock_guard<std::mutex> lock(pending_mutex);
      for (auto it = pending.begin(); it != pending.end(); ++it) {
        if (it->second == promise) {
          pending.erase(it);
          break;
        }
      }
      response.success = false;
      response.message = "timed out waiting for agent emergency-stop ACK";
      return;
    }
    const auto result = future.get();
    response.success = result.success;
    response.message = result.message;
    if (result.success && !engage) controller->clear_estop();
    publish_connection_state();
  }

  void on_frame(Frame frame) {
    try {
      if (frame.type == MessageType::HelloAck) {
        const auto hello = decode_hello_ack(frame.payload);
        if (hello.selected_version != kBridgeProtocolVersion) {
          throw ProtocolError("agent selected an incompatible HTBR version");
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          connection.agent_connected = true;
          connection.last_error.clear();
        }
        publish_connection_state();
        RCLCPP_INFO(node.get_logger(), "Hypertron agent ready; SDK %s",
                    hello.sdk_version.c_str());
        return;
      }
      if (frame.type == MessageType::Ack) {
        const auto ack = decode_ack(frame.payload);
        resolve_pending(ack.request_sequence,
                        {true, ack.result_code, ack.text});
        return;
      }
      if (frame.type == MessageType::Error) {
        const auto error = decode_error(frame.payload);
        resolve_pending(error.request_sequence,
                        {false, static_cast<std::uint16_t>(error.error),
                         error.text});
        update_error(error.text);
        return;
      }
      if (frame.type == MessageType::Ping) {
        tunnel->send(command_frame(MessageType::Pong, {}));
        return;
      }
      if (frame.type == MessageType::RobotState) {
        const auto state = decode_robot_state(frame.payload);
        controller->update_robot_state(
            {state.sdk_linked, state.control_authority, state.sport_status,
             state.error_code});
      }
      const auto now = node.get_clock()->now().nanoseconds();
      receiver->handle(frame, now < 0 ? 0U : static_cast<std::uint64_t>(now));
    } catch (const std::exception& error) {
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        ++connection.protocol_rx_drops;
        connection.last_error = error.what();
      }
      publish_connection_state();
    }
  }

  void on_tunnel_state(ConnectionState state, const std::string& detail) {
    if (state == ConnectionState::Connected) {
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        connection.ssh_connected = true;
        connection.agent_connected = false;
        connection.last_error.clear();
      }
      publish_connection_state();
      const HelloPayload hello{kBridgeProtocolVersion, kBridgeProtocolVersion,
                               CapabilityImu | CapabilitySport |
                                   CapabilityOdometry | CapabilitySystemState |
                                   CapabilityCamera,
                               next_sequence.load() ^ 0x48544252U};
      tunnel->send(command_frame(MessageType::Hello, encode_hello(hello)));
      return;
    }
    if (state == ConnectionState::Disconnected ||
        state == ConnectionState::Failed || state == ConnectionState::Stopped) {
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        connection.ssh_connected = false;
        connection.agent_connected = false;
        connection.last_error = detail;
      }
      controller->update_robot_state({});
      fail_pending(detail);
      publish_connection_state();
    }
  }

  void resolve_pending(std::uint32_t sequence, RequestResult result) {
    std::shared_ptr<std::promise<RequestResult>> promise;
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      const auto found = pending.find(sequence);
      if (found == pending.end()) return;
      promise = found->second;
      pending.erase(found);
    }
    promise->set_value(std::move(result));
  }

  void fail_pending(const std::string& reason) {
    std::unordered_map<std::uint32_t,
                       std::shared_ptr<std::promise<RequestResult>>>
        local;
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      local.swap(pending);
    }
    for (auto& item : local) {
      item.second->set_value({false, 0, reason});
    }
  }

  void update_error(const std::string& error) {
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      connection.last_error = error;
    }
    publish_connection_state();
  }

  void publish_connection_state() {
    ReceiverConnectionState copy;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      copy = connection;
    }
    receiver->set_connection_state(copy);
    if (!copy.agent_connected) receiver->publish_disconnected_state();
  }
};

HypertronBridgeNode::HypertronBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hypertron_bridge", options),
      impl_(std::make_unique<Impl>(*this)) {}

HypertronBridgeNode::~HypertronBridgeNode() = default;

}  // namespace hypertron_ros2_bridge

#endif  // HYPERTRON_WITH_ROS2
