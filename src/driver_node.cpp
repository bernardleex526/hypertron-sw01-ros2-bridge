// ROS 2 driver node for the direct ASTRALL 1.0.7 SDK path.
//
// Owns the DirectDriverRuntime as the single consumer of the SDK and bridges
// it onto the ROS graph: /cmd_vel, /robot_mode and /joint_commands in;
// sensor_msgs/Imu, nav_msgs/Odometry and RobotState out; and a software
// /emergency_stop plus /control_authority SetBool service pair.
//
// Threading: construction is non-blocking (runtime_->start() is called but no
// SDK or network call runs on the constructor's thread). The runtime worker
// and vendor SDK threads invoke the NodeObserver callbacks, which must never
// block and never call back into the runtime. They only copy values and use
// thread-safe ROS publishes (all throttled logging goes through
// *node.get_clock(), which is thread-safe) plus de-bounced state publishes.
// The observer is a member declared before runtime_ so it outlives the
// runtime; the destructor stops the runtime before the observer goes away, so
// no callback can ever fire against destroyed publishers.

#include "hypertron_ros2_bridge/driver_node.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"
#include "hypertron_ros2_bridge/direct_driver_runtime.hpp"
#include "hypertron_ros2_bridge/lidar_stream.hpp"
#include "hypertron_ros2_bridge/msg/robot_state.hpp"
#include "hypertron_ros2_bridge/network_preflight.hpp"
#include "hypertron_ros2_bridge/robot_controller.hpp"

namespace hypertron_ros2_bridge {

namespace {

using RobotStateMsg = hypertron_ros2_bridge::msg::RobotState;

// Names accepted by /robot_mode, compared case-insensitively after trimming.
const std::vector<std::string>& robot_mode_names() {
  static const std::vector<std::string> names = {
      "damping",  "stand",       "down",         "move",
      "auto_charge", "exit_charge", "recover",    "recovery",
  };
  return names;
}

std::string trimmed_lower(const std::string& in) {
  const auto first = in.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::string{};
  }
  const auto last = in.find_last_not_of(" \t\r\n");
  std::string out = in.substr(first, last - first + 1);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

bool finite_float(float v) { return std::isfinite(v); }

}  // namespace

struct HypertronDriverNode::Impl {
  // Concrete RuntimeObserver that forwards to Impl's private handlers. Held
  // by value and declared before runtime_ so it outlives the runtime (the
  // runtime holds a raw pointer to it).
  struct NodeObserver : public RuntimeObserver {
    Impl& impl;
    explicit NodeObserver(Impl& owner) : impl(owner) {}
    void on_state(const SdkSnapshot& snapshot) override { impl.on_state(snapshot); }
    void on_imu(const ImuSample& sample) override { impl.on_imu(sample); }
    void on_sport(const SportSample& sample) override { impl.on_sport(sample); }
    void on_status(bool linked, bool control_authority) override {
      impl.on_status(linked, control_authority);
    }
    void on_event(int severity, std::string message) override {
      impl.on_event(severity, std::move(message));
    }
  };

  explicit Impl(HypertronDriverNode& node, std::unique_ptr<IAstrallSdk> sdk,
                std::unique_ptr<INetworkPreflight> preflight,
                std::unique_ptr<ILidarDatagramSource> point_cloud_source,
                std::unique_ptr<ILidarDatagramSource> odometry_source)
      : node(node), clock(), observer(*this) {
    declare_parameters();
    validate_parameters();
    create_graph_interfaces();
    decide_tf_broadcaster();
    create_receivers(std::move(point_cloud_source), std::move(odometry_source));
    assemble_runtime(std::move(sdk), std::move(preflight));
    // Receivers are started only after the runtime serializes the session;
    // order within construction does not matter for safety but starting them
    // last means a bind failure never gates the SDK session.
    start_receivers();
  }

  ~Impl() {
    // Stop the LiDAR receive threads before tearing down the runtime so no
    // receiver callback can fire against a destroyed publisher or a stopped
    // runtime. Order is explicit: receivers first, then runtime.
    if (point_cloud_receiver_) {
      point_cloud_receiver_->stop();
    }
    if (odometry_receiver_) {
      odometry_receiver_->stop();
    }
    if (runtime_) {
      runtime_->stop();
    }
    log_lidar_stats();
  }

  // ------------------------------------------------------------------
  // RuntimeObserver handlers (may arrive on the runtime worker or a vendor
  // SDK thread). Never block; never call back into runtime_.
  // ------------------------------------------------------------------
  void on_state(const SdkSnapshot& snapshot) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      snapshot_ = snapshot;
      status_linked_ = snapshot.sdk_linked;
      status_authority_ = snapshot.control_authority;
    }
    publish_robot_state();
    if (tf_broadcaster_ && last_pose_indicator()) {
      publish_tf();
    }
  }

  void on_imu(const ImuSample& sample) {
    if (!imu_sample_finite(sample)) {
      update_last_error(
          "dropped sample: IMU telemetry contained a non-finite value");
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                           "Dropped IMU sample: non-finite value");
      return;
    }
    const auto now = node.get_clock()->now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = frames_.imu;
    if (!orientation_quaternion(sample, imu.orientation)) {
      update_last_error("dropped sample: IMU orientation norm was zero");
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                           "Dropped IMU sample: zero-norm orientation");
      return;
    }
    std::copy(orientation_covariance_.begin(), orientation_covariance_.end(),
              imu.orientation_covariance.begin());
    std::copy(angular_velocity_covariance_.begin(),
              angular_velocity_covariance_.end(),
              imu.angular_velocity_covariance.begin());
    std::copy(linear_acceleration_covariance_.begin(),
              linear_acceleration_covariance_.end(),
              imu.linear_acceleration_covariance.begin());
    imu.angular_velocity.x = sample.gyroscope[0];
    imu.angular_velocity.y = sample.gyroscope[1];
    imu.angular_velocity.z = sample.gyroscope[2];
    imu.linear_acceleration.x = sample.accelerometer[0];
    imu.linear_acceleration.y = sample.accelerometer[1];
    imu.linear_acceleration.z = sample.accelerometer[2];
    imu_pub_->publish(imu);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frames_.odom;
    odom.child_frame_id = frames_.base;
    odom.pose.pose.position.x = sample.odom_x;
    odom.pose.pose.position.y = sample.odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(sample.yaw * 0.5);
    odom.pose.pose.orientation.w = std::cos(sample.yaw * 0.5);
    odom_pub_->publish(odom);

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      last_pose_ = odom.pose.pose;
    }
    if (tf_broadcaster_) {
      publish_tf();
    }
  }

  void on_sport(const SportSample& sample) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot_.wheel_speed = sample.wheel_speed;
  }

  void on_status(bool linked, bool control_authority) {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (status_linked_ != linked || status_authority_ != control_authority) {
        status_linked_ = linked;
        status_authority_ = control_authority;
        changed = true;
      }
    }
    if (changed) {
      publish_robot_state();
    }
  }

  void on_event(int severity, std::string message) {
    if (severity >= kRuntimeEventWarning) {
      update_last_error_owned(message);
    }
    switch (severity) {
      case kRuntimeEventError:
        RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                              "runtime error: %s", message.c_str());
        break;
      case kRuntimeEventWarning:
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                             "runtime warning: %s", message.c_str());
        break;
      default:
      case kRuntimeEventInfo:
        RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                              "runtime: %s", message.c_str());
        break;
    }
  }

  // ------------------------------------------------------------------
  // ROS interface entry points.
  // ------------------------------------------------------------------
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!runtime_) {
      return;
    }
    // Reject any non-finite component before clamping: clamp_unit() would
    // otherwise turn NaN into +1.0f and slip past the controller's non-finite
    // rejection. The public /cmd_vel callback must never submit a NaN/Inf.
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
        !std::isfinite(msg->angular.z)) {
      rejected_nonfinite_cmd_vel_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
          node.get_logger(), *node.get_clock(), 1000,
          "Rejected /cmd_vel with non-finite component (linear.x=%g "
          "linear.y=%g angular.z=%g)",
          static_cast<double>(msg->linear.x),
          static_cast<double>(msg->linear.y),
          static_cast<double>(msg->angular.z));
      return;
    }
    Velocity v;
    v.vx = clamp_unit(msg->linear.x);
    v.vy = clamp_unit(msg->linear.y);
    v.vyaw = clamp_unit(msg->angular.z);
    runtime_->submit_velocity(v);
  }

  void on_robot_mode(const std_msgs::msg::String::SharedPtr msg) {
    if (!runtime_) {
      return;
    }
    const std::string name = trimmed_lower(msg->data);
    const auto& names = robot_mode_names();
    if (std::find(names.begin(), names.end(), name) == names.end()) {
      RCLCPP_WARN_THROTTLE(
          node.get_logger(), *node.get_clock(), 1000,
          "Rejected /robot_mode '%s': not one of the documented mode names",
          msg->data.c_str());
      return;
    }
    std::future<Result> future = runtime_->request_mode(name);
    if (future.wait_for(std::chrono::milliseconds(50)) ==
        std::future_status::ready) {
      const Result result = future.get();
      if (result.success()) {
        RCLCPP_INFO(node.get_logger(), "Robot mode '%s' confirmed", name.c_str());
      } else {
        RCLCPP_WARN(node.get_logger(), "Robot mode '%s' failed: %s",
                    name.c_str(), result.message.c_str());
      }
    } else {
      RCLCPP_DEBUG(node.get_logger(), "Robot mode '%s' in progress", name.c_str());
    }
  }

  // Token-bearing rejection path for joint commands (ASTRALL exposes no joint
  // API). Counts rejections, records the reason, re-publishes state, throttled
  // warning.
  void reject_joint_command(const std::string& reason) {
    rejected_joint_commands_.fetch_add(1, std::memory_order_relaxed);
    update_last_error(reason);
    publish_robot_state();
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "%s",
                         reason.c_str());
  }

  void on_joint_commands(const sensor_msgs::msg::JointState::SharedPtr /*msg*/) {
    reject_joint_command(
        "joint command rejected: ASTRALL 1.0.7 exposes no joint API");
  }

  void on_emergency_stop(const std_srvs::srv::SetBool::Request::SharedPtr req,
                         std_srvs::srv::SetBool::Response::SharedPtr resp) {
    if (!runtime_) {
      resp->success = false;
      resp->message = "runtime not available";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      estop_latched_ = req->data;
    }
    std::future<Result> future = runtime_->trigger_estop(req->data);
    const auto timeout = std::chrono::milliseconds(2 * sdk_call_timeout_ms_) +
                         std::chrono::milliseconds(2000);
    if (future.wait_for(timeout) == std::future_status::ready) {
      const Result result = future.get();
      resp->success = result.success();
      resp->message = result.message;
    } else {
      resp->success = false;
      resp->message =
          "estop request latched locally but SDK execution was not confirmed "
          "within the service timeout";
    }
    publish_robot_state();
  }

  void on_control_authority(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr resp) {
    if (!runtime_) {
      resp->success = false;
      resp->message = "runtime not available";
      return;
    }
    std::future<Result> future = runtime_->request_authority(req->data);
    const auto timeout = std::chrono::milliseconds(2 * sdk_call_timeout_ms_) +
                         std::chrono::milliseconds(2000);
    if (future.wait_for(timeout) == std::future_status::ready) {
      const Result result = future.get();
      resp->success = result.success();
      resp->message = result.message;
    } else {
      resp->success = false;
      resp->message =
          "control authority request not confirmed within the service timeout";
    }
    publish_robot_state();
  }

 private:
  // ------------------------------------------------------------------
  // Parameter handling.
  // ------------------------------------------------------------------
  void declare_parameters() {
    node.declare_parameter("interface", "eno1");
    node.declare_parameter("host_address", "10.18.0.200/24");
    node.declare_parameter("robot_address", "10.18.0.100");

    node.declare_parameter("topics.cmd_vel", "/cmd_vel");
    node.declare_parameter("topics.robot_mode", "/robot_mode");
    node.declare_parameter("topics.joint_commands", "/joint_commands");
    node.declare_parameter("topics.imu", "/imu/data");
    node.declare_parameter("topics.odom", "/odom");
    node.declare_parameter("topics.robot_state", "/robot_state");

    node.declare_parameter("frames.imu", "imu_link");
    node.declare_parameter("frames.odom", "odom");
    node.declare_parameter("frames.base", "base_link");

    node.declare_parameter("sdk.init_timeout_ms", 60000);
    node.declare_parameter("sdk.call_timeout_ms", 5000);
    node.declare_parameter("sdk.heartbeat_call_timeout_ms", 500);
    node.declare_parameter("sdk.heartbeat_max_failures", 1);

    node.declare_parameter("timing.heartbeat_period_ms", 100);
    node.declare_parameter("timing.motion_refresh_period_ms", 20);
    node.declare_parameter("timing.state_poll_period_ms", 500);
    node.declare_parameter("timing.mode_timeout_ms", 5000);
    node.declare_parameter("timing.reconnect_initial_delay_ms", 1000);
    node.declare_parameter("timing.reconnect_max_delay_ms", 30000);

    node.declare_parameter("safety.command_deadman_ms", 100);
    node.declare_parameter("safety.queue_capacity", 16);

    node.declare_parameter("subscriptions.imu_frequency_hz", 50);
    node.declare_parameter("subscriptions.sport_frequency_hz", 50);
    node.declare_parameter("subscriptions.lidar_enabled", true);

    node.declare_parameter("topics.points", "/points");
    node.declare_parameter("topics.odom_lidar", "/odom_lidar");

    node.declare_parameter("lidar.point_cloud_port", 6100);
    node.declare_parameter("lidar.odometry_port", 6101);
    node.declare_parameter("lidar.bind_ip", "0.0.0.0");
    node.declare_parameter("lidar.source_ip", "10.18.0.100");
    node.declare_parameter("lidar.point_position_scale", 1.0e-3);
    node.declare_parameter("lidar.odom_position_scale", 1.0e-6);
    node.declare_parameter("lidar.odom_quaternion_scale", 1.0);
    node.declare_parameter("lidar.frame_id", "lidar");
    node.declare_parameter("lidar.frame_timeout_ms", 300);
    node.declare_parameter("lidar.max_parallel_frames", 4);
    node.declare_parameter("lidar.max_points_per_frame", 2000000);
    node.declare_parameter("lidar.max_packets_per_frame", 4096);

    node.declare_parameter("odom_lidar.parent_frame", "odom");
    node.declare_parameter("odom_lidar.child_frame", "lidar");
    node.declare_parameter("odom_lidar.pose_covariance_diagonal",
                           std::vector<double>{0.05, 0.05, 0.05, 0.01, 0.01, 0.01});

    node.declare_parameter("imu.quaternion_order", "xyzw");
    node.declare_parameter("imu.orientation_covariance_diagonal",
                           std::vector<double>{0.02, 0.02, 0.05});
    node.declare_parameter("imu.angular_velocity_covariance_diagonal",
                           std::vector<double>{0.01, 0.01, 0.02});
    node.declare_parameter("imu.linear_acceleration_covariance_diagonal",
                           std::vector<double>{0.10, 0.10, 0.20});

    node.declare_parameter("odom.publish_tf", false);
    node.declare_parameter("odometry.scale_verified", false);
  }

  void validate_parameters() {
    // Parameter range validation: an invalid parameter is a configuration
    // error, so the whole node construction fails (main exits non-zero) rather
    // than silently clamping, wrapping to a negative value, or allocating an
    // unbounded buffer. This runs before any receiver or runtime is built.
    auto fail = [this](const std::string& message) {
      throw rclcpp::exceptions::InvalidParametersException(message);
    };

    // Ports within [1, 65535].
    for (const char* p :
         {"lidar.point_cloud_port", "lidar.odometry_port"}) {
      const int port = node.get_parameter(p).as_int();
      if (port < 1 || port > 65535) {
        fail(std::string(p) + "=" + std::to_string(port) +
             " is outside the valid port range [1, 65535]");
      }
    }

    // Every *_ms timing / timeout must be strictly positive.
    for (const char* p :
         {"sdk.init_timeout_ms",       "sdk.call_timeout_ms",
          "sdk.heartbeat_call_timeout_ms", "timing.heartbeat_period_ms",
          "timing.motion_refresh_period_ms", "timing.state_poll_period_ms",
          "timing.mode_timeout_ms",    "timing.reconnect_initial_delay_ms",
          "timing.reconnect_max_delay_ms", "safety.command_deadman_ms",
          "lidar.frame_timeout_ms"}) {
      const int ms = node.get_parameter(p).as_int();
      if (ms <= 0) {
        fail(std::string(p) + "=" + std::to_string(ms) +
             " must be strictly positive");
      }
    }

    // Bounded command queue and concurrency / allocation caps.
    const int queue_capacity =
        node.get_parameter("safety.queue_capacity").as_int();
    if (queue_capacity < 1 || queue_capacity > 1024) {
      fail("safety.queue_capacity=" + std::to_string(queue_capacity) +
           " is outside [1, 1024]");
    }
    const int max_parallel =
        node.get_parameter("lidar.max_parallel_frames").as_int();
    if (max_parallel < 1 || max_parallel > 64) {
      fail("lidar.max_parallel_frames=" + std::to_string(max_parallel) +
           " is outside [1, 64]");
    }
    const int max_points =
        node.get_parameter("lidar.max_points_per_frame").as_int();
    if (max_points < 1 || max_points > 50000000) {
      fail("lidar.max_points_per_frame=" + std::to_string(max_points) +
           " is outside [1, 50000000]");
    }
    const int max_packets =
        node.get_parameter("lidar.max_packets_per_frame").as_int();
    if (max_packets < 1 || max_packets > 65536) {
      fail("lidar.max_packets_per_frame=" + std::to_string(max_packets) +
           " is outside [1, 65536]");
    }

    // Scale coefficients: finite and within (0, 1e9].
    for (const char* p :
         {"lidar.point_position_scale", "lidar.odom_position_scale",
          "lidar.odom_quaternion_scale"}) {
      const double scale = node.get_parameter(p).as_double();
      if (!std::isfinite(scale) || scale <= 0.0 || scale > 1e9) {
        fail(std::string(p) + "=" + std::to_string(scale) +
             " must be finite and within (0, 1e9]");
      }
    }

    // Heartbeat failure threshold and reconnect backoff ordering.
    const int heartbeat_max_failures =
        node.get_parameter("sdk.heartbeat_max_failures").as_int();
    if (heartbeat_max_failures < 1) {
      fail("sdk.heartbeat_max_failures=" +
           std::to_string(heartbeat_max_failures) +
           " must be >= 1");
    }
    const int reconnect_initial =
        node.get_parameter("timing.reconnect_initial_delay_ms").as_int();
    const int reconnect_max =
        node.get_parameter("timing.reconnect_max_delay_ms").as_int();
    if (reconnect_initial > reconnect_max) {
      fail("timing.reconnect_initial_delay_ms (" +
           std::to_string(reconnect_initial) + ") exceeds " +
           "timing.reconnect_max_delay_ms (" +
           std::to_string(reconnect_max) + ")");
    }

    const std::string robot_address =
        node.get_parameter("robot_address").as_string();
    if (robot_address != "10.18.0.100") {
      RCLCPP_ERROR(
          node.get_logger(),
          "robot_address=%s differs from the SDK-hardcoded 10.18.0.100:3600; "
          "the SDK still targets 10.18.0.100:3600 but preflight will use the "
          "configured robot_address",
          robot_address.c_str());
    }

    imu_freq_ = frequency_from_hz(
        node.get_parameter("subscriptions.imu_frequency_hz").as_int());
    sport_freq_ = frequency_from_hz(
        node.get_parameter("subscriptions.sport_frequency_hz").as_int());

    const std::string order =
        node.get_parameter("imu.quaternion_order").as_string();
    if (order == "wxyz") {
      quaternion_order_wxyz_ = true;
    } else if (order == "xyzw") {
      quaternion_order_wxyz_ = false;
    } else {
      throw rclcpp::exceptions::InvalidParametersException(
          "imu.quaternion_order='" + order +
          "' is not 'xyzw' or 'wxyz'");
    }

    orientation_covariance_ =
        diagonal_covariance(node.get_parameter("imu.orientation_covariance_diagonal")
                                .as_double_array());
    angular_velocity_covariance_ = diagonal_covariance(
        node.get_parameter("imu.angular_velocity_covariance_diagonal")
            .as_double_array());
    linear_acceleration_covariance_ = diagonal_covariance(
        node.get_parameter("imu.linear_acceleration_covariance_diagonal")
            .as_double_array());
    validate_covariance_diagonal(
        "imu.orientation_covariance_diagonal",
        node.get_parameter("imu.orientation_covariance_diagonal")
            .as_double_array());
    validate_covariance_diagonal(
        "imu.angular_velocity_covariance_diagonal",
        node.get_parameter("imu.angular_velocity_covariance_diagonal")
            .as_double_array());
    validate_covariance_diagonal(
        "imu.linear_acceleration_covariance_diagonal",
        node.get_parameter("imu.linear_acceleration_covariance_diagonal")
            .as_double_array());

    // LiDAR stream configuration resolved at construction.
    lidar_enabled_ = node.get_parameter("subscriptions.lidar_enabled").as_bool();
    points_topic_ = node.get_parameter("topics.points").as_string();
    odom_lidar_topic_ = node.get_parameter("topics.odom_lidar").as_string();
    lidar_.point_cloud_port = static_cast<std::uint16_t>(
        node.get_parameter("lidar.point_cloud_port").as_int());
    lidar_.odometry_port = static_cast<std::uint16_t>(
        node.get_parameter("lidar.odometry_port").as_int());
    lidar_.bind_ip = node.get_parameter("lidar.bind_ip").as_string();
    lidar_.source_ip = node.get_parameter("lidar.source_ip").as_string();
    lidar_.frame_id = node.get_parameter("lidar.frame_id").as_string();
    lidar_.frame_timeout_ms = static_cast<int>(
        node.get_parameter("lidar.frame_timeout_ms").as_int());
    lidar_.max_parallel_frames = static_cast<std::size_t>(
        node.get_parameter("lidar.max_parallel_frames").as_int());
    lidar_.max_points_per_frame = static_cast<std::size_t>(
        node.get_parameter("lidar.max_points_per_frame").as_int());
    lidar_.max_packets_per_frame = static_cast<std::size_t>(
        node.get_parameter("lidar.max_packets_per_frame").as_int());
    lidar_.point_position_scale =
        node.get_parameter("lidar.point_position_scale").as_double();
    lidar_.odom_position_scale =
        node.get_parameter("lidar.odom_position_scale").as_double();
    lidar_.odom_quaternion_scale =
        node.get_parameter("lidar.odom_quaternion_scale").as_double();
    odom_lidar_.parent_frame =
        node.get_parameter("odom_lidar.parent_frame").as_string();
    odom_lidar_.child_frame =
        node.get_parameter("odom_lidar.child_frame").as_string();
    odom_lidar_.pose_covariance_diagonal =
        node.get_parameter("odom_lidar.pose_covariance_diagonal")
            .as_double_array();
    validate_covariance_diagonal(
        "odom_lidar.pose_covariance_diagonal",
        odom_lidar_.pose_covariance_diagonal);
  }

  SubscriptionFrequency frequency_from_hz(int hz) {
    switch (hz) {
      case 0:
        return SubscriptionFrequency::Disabled;
      case 1:
        return SubscriptionFrequency::Hz1;
      case 25:
        return SubscriptionFrequency::Hz25;
      case 50:
        return SubscriptionFrequency::Hz50;
      case 125:
        return SubscriptionFrequency::Hz125;
      case 250:
        return SubscriptionFrequency::Hz250;
      default:
        throw rclcpp::exceptions::InvalidParametersException(
            "subscription frequency " + std::to_string(hz) +
            " Hz is not valid (must be one of 0/1/25/50/125/250)");
    }
  }

  static std::array<double, 9> diagonal_covariance(
      const std::vector<double>& diag) {
    std::array<double, 9> cov{};
    for (std::size_t i = 0; i < 3; ++i) {
      double value =
          (i < diag.size() && std::isfinite(diag[i])) ? diag[i] : 0.0;
      cov[i * 4] = value;
    }
    return cov;
  }

  // Every diagonal covariance coefficient must be finite and >= 0; an
  // invalid coefficient is a configuration error that fails construction.
  void validate_covariance_diagonal(const std::string& name,
                                    const std::vector<double>& diag) {
    for (std::size_t i = 0; i < diag.size(); ++i) {
      if (!std::isfinite(diag[i]) || diag[i] < 0.0) {
        throw rclcpp::exceptions::InvalidParametersException(
            name + " coefficient " + std::to_string(i) + " (" +
            std::to_string(diag[i]) + ") must be finite and >= 0");
      }
    }
  }

  // ------------------------------------------------------------------
  // ROS graph construction.
  // ------------------------------------------------------------------
  void create_graph_interfaces() {
    const std::string cmd_vel_topic =
        node.get_parameter("topics.cmd_vel").as_string();
    const std::string mode_topic =
        node.get_parameter("topics.robot_mode").as_string();
    const std::string joint_topic =
        node.get_parameter("topics.joint_commands").as_string();
    const std::string imu_topic = node.get_parameter("topics.imu").as_string();
    const std::string odom_topic = node.get_parameter("topics.odom").as_string();
    const std::string state_topic =
        node.get_parameter("topics.robot_state").as_string();

    imu_pub_ = node.create_publisher<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS());
    odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS());
    robot_state_pub_ = node.create_publisher<RobotStateMsg>(
        state_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    // LiDAR bypass publishers. Created only when the stream is enabled so a
    // disabled node exposes exactly zero lidar publishers and zero receivers.
    if (lidar_enabled_) {
      points_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
          points_topic_, rclcpp::SensorDataQoS());
      odom_lidar_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
          odom_lidar_topic_, rclcpp::SensorDataQoS());
    }

    cmd_vel_sub_ = node.create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          on_cmd_vel(msg);
        });
    mode_sub_ = node.create_subscription<std_msgs::msg::String>(
        mode_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) { on_robot_mode(msg); });
    joint_sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          on_joint_commands(msg);
        });

    estop_srv_ = node.create_service<std_srvs::srv::SetBool>(
        "/emergency_stop",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr resp) {
          on_emergency_stop(req, resp);
        });
    authority_srv_ = node.create_service<std_srvs::srv::SetBool>(
        "/control_authority",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr resp) {
          on_control_authority(req, resp);
        });
  }

  void decide_tf_broadcaster() {
    const bool publish_tf = node.get_parameter("odom.publish_tf").as_bool();
    const bool scale_verified =
        node.get_parameter("odometry.scale_verified").as_bool();
    if (publish_tf && !scale_verified) {
      RCLCPP_ERROR(node.get_logger(),
                   "invalid configuration: odom.publish_tf requires "
                   "odometry.scale_verified");
      tf_broadcaster_.reset();
      return;
    }
    if (publish_tf) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }
  }

  // ------------------------------------------------------------------
  // LiDAR bypass stream receivers (UDP 6100 / 6101).
  // ------------------------------------------------------------------

  // Binds the LiDAR UDP sockets and constructs the receivers. A preferred
  // injected source (tests) replaces the production UdpLidarDatagramSource;
  // a null source defaults to binding the configured port. A bind failure
  // (port in use) logs an error and skips that stream without taking the node
  // down: the SDK, motion, and /odom diagnostics stay fully functional.
  void create_receivers(std::unique_ptr<ILidarDatagramSource> point_cloud_source,
                        std::unique_ptr<ILidarDatagramSource> odometry_source) {
    if (!lidar_enabled_) {
      RCLCPP_DEBUG(node.get_logger(), "LiDAR stream disabled; no UDP receivers");
      return;
    }
    LidarParseConfig parse_cfg;
    parse_cfg.point_position_scale = lidar_.point_position_scale;
    parse_cfg.odom_position_scale = lidar_.odom_position_scale;
    parse_cfg.odom_quaternion_scale = lidar_.odom_quaternion_scale;

    AssemblerConfig assembler_cfg;
    assembler_cfg.frame_timeout = std::chrono::milliseconds(lidar_.frame_timeout_ms);
    assembler_cfg.max_parallel_frames = lidar_.max_parallel_frames;
    assembler_cfg.max_points_per_frame = lidar_.max_points_per_frame;
    assembler_cfg.max_packets_per_frame = lidar_.max_packets_per_frame;

    LidarSink sink;
    sink.on_frame = [this](const PointCloudFrame& frame) { on_lidar_frame(frame); };
    sink.on_odometry = [this](const LidarOdometry& odom) { on_lidar_odometry(odom); };
    const auto parse_warning = [this](const std::string& reason) {
      // The receiver already throttles to ~1 Hz; record directly.
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                           "LiDAR parse warning: %s", reason.c_str());
    };

    if (!point_cloud_source) {
      auto udp = std::make_unique<UdpLidarDatagramSource>(
          lidar_.point_cloud_port, lidar_.bind_ip, lidar_.source_ip);
      if (!udp->valid()) {
        RCLCPP_ERROR(node.get_logger(),
                     "LiDAR point-cloud UDP port %u bind failed; skipping "
                     "the point-cloud stream (node stays alive)",
                     static_cast<unsigned>(lidar_.point_cloud_port));
      } else {
        point_cloud_receiver_ = std::make_unique<LidarStreamReceiver>(
            std::move(udp), LidarStreamReceiver::LidarStreamKind::PointCloud,
            parse_cfg, assembler_cfg, sink, parse_warning);
      }
    } else {
      point_cloud_receiver_ = std::make_unique<LidarStreamReceiver>(
          std::move(point_cloud_source),
          LidarStreamReceiver::LidarStreamKind::PointCloud, parse_cfg,
          assembler_cfg, sink, parse_warning);
    }

    if (!odometry_source) {
      auto udp = std::make_unique<UdpLidarDatagramSource>(
          lidar_.odometry_port, lidar_.bind_ip, lidar_.source_ip);
      if (!udp->valid()) {
        RCLCPP_ERROR(node.get_logger(),
                     "LiDAR odometry UDP port %u bind failed; skipping "
                     "the odometry stream (node stays alive)",
                     static_cast<unsigned>(lidar_.odometry_port));
      } else {
        odometry_receiver_ = std::make_unique<LidarStreamReceiver>(
            std::move(udp), LidarStreamReceiver::LidarStreamKind::Odometry,
            parse_cfg, assembler_cfg, sink, parse_warning);
      }
    } else {
      odometry_receiver_ = std::make_unique<LidarStreamReceiver>(
          std::move(odometry_source),
          LidarStreamReceiver::LidarStreamKind::Odometry, parse_cfg,
          assembler_cfg, sink, parse_warning);
    }
  }

  void start_receivers() {
    // Start the receive threads only after the runtime has started the SDK
    // session; the reprocessing order within construction does not gate
    // safety, but starting last means a bind failure never blocks motion.
    if (point_cloud_receiver_) {
      point_cloud_receiver_->start();
      RCLCPP_DEBUG(node.get_logger(), "LiDAR point-cloud stream started");
    }
    if (odometry_receiver_) {
      odometry_receiver_->start();
      RCLCPP_DEBUG(node.get_logger(), "LiDAR odometry stream started");
    }
  }

  // Receiver thread -> publisher (thread-safe, never blocks, never calls
  // runtime). Converts one complete frame to a sensor_msgs/PointCloud2 with
  // the standard PointXYZRGBA layout.
  void on_lidar_frame(const PointCloudFrame& frame) {
    if (!points_pub_) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node.get_clock()->now();
    msg.header.frame_id = lidar_.frame_id;
    msg.height = 1U;
    msg.width = static_cast<std::uint32_t>(frame.points.size());
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.point_step = 16U;
    msg.row_step = msg.point_step * msg.width;

    sensor_msgs::msg::PointField field;
    sensor_msgs::msg::PointField x;
    x.name = "x"; x.offset = 0U; x.datatype = sensor_msgs::msg::PointField::FLOAT32; x.count = 1U;
    sensor_msgs::msg::PointField y;
    y.name = "y"; y.offset = 4U; y.datatype = sensor_msgs::msg::PointField::FLOAT32; y.count = 1U;
    sensor_msgs::msg::PointField z;
    z.name = "z"; z.offset = 8U; z.datatype = sensor_msgs::msg::PointField::FLOAT32; z.count = 1U;
    sensor_msgs::msg::PointField rgba;
    rgba.name = "rgba"; rgba.offset = 12U; rgba.datatype = sensor_msgs::msg::PointField::UINT32; rgba.count = 1U;
    msg.fields = {x, y, z, rgba};

    msg.data.resize(msg.point_step * frame.points.size());
    for (std::size_t i = 0; i < frame.points.size(); ++i) {
      const LidarPoint& p = frame.points[i];
      std::uint8_t* dst = msg.data.data() + i * msg.point_step;
      float f;
      f = p.x;
      std::memcpy(dst + 0, &f, sizeof(float));
      f = p.y;
      std::memcpy(dst + 4, &f, sizeof(float));
      f = p.z;
      std::memcpy(dst + 8, &f, sizeof(float));
      std::memcpy(dst + 12, &p.rgba, sizeof(p.rgba));
    }
    points_pub_->publish(msg);
  }

  // Receiver thread -> publisher (thread-safe, never blocks, never calls
  // runtime). Normalizes the quaternion defensively; a ~zero norm is dropped
  // with a throttled warning (a point cloud can still flow).
  void on_lidar_odometry(const LidarOdometry& odom) {
    if (!odom_lidar_pub_) {
      return;
    }
    const double norm = std::sqrt(odom.qx * odom.qx + odom.qy * odom.qy +
                                  odom.qz * odom.qz + odom.qw * odom.qw);
    if (!finite_double(norm) ||
        norm < std::numeric_limits<double>::epsilon() * 4.0) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                           "Dropped LiDAR odometry: zero-norm orientation");
      return;
    }

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node.get_clock()->now();
    msg.header.frame_id = odom_lidar_.parent_frame;
    msg.child_frame_id = odom_lidar_.child_frame;
    msg.pose.pose.position.x = odom.x;
    msg.pose.pose.position.y = odom.y;
    msg.pose.pose.position.z = odom.z;
    msg.pose.pose.orientation.x = odom.qx / norm;
    msg.pose.pose.orientation.y = odom.qy / norm;
    msg.pose.pose.orientation.z = odom.qz / norm;
    msg.pose.pose.orientation.w = odom.qw / norm;
    // Pose covariance: package the configured diagonal into the 6x6 matrix.
    const auto& diag = odom_lidar_.pose_covariance_diagonal;
    for (std::size_t i = 0; i < 6; ++i) {
      const double value = (i < diag.size() && std::isfinite(diag[i])) ? diag[i] : 0.0;
      msg.pose.covariance[i * 7] = value;
    }
    // No twist is reported by the lidar stream; leave the 6x6 twist
    // covariance and the twist at their all-zero defaults.
    odom_lidar_pub_->publish(msg);
  }

  static bool finite_double(double v) { return std::isfinite(v); }

  // Reads the receiver assembler stats safely only after stop(); always
  // present, never unsafe to call from teardown.
  void log_lidar_stats() {
    if (!point_cloud_receiver_) {
      if (lidar_enabled_) {
        RCLCPP_INFO(node.get_logger(), "LiDAR point-cloud stream not active");
      }
      return;
    }
    const auto pc = point_cloud_receiver_->assembler_stats();
    RCLCPP_INFO(
        node.get_logger(),
        "LiDAR point-cloud stats: frames=%llu accepted=%llu rejected=%llu "
        "dup=%llu timeout_drops=%llu conflict_drops=%llu capacity_drops=%llu",
        static_cast<unsigned long long>(pc.frames_published),
        static_cast<unsigned long long>(pc.packets_accepted),
        static_cast<unsigned long long>(pc.packets_rejected),
        static_cast<unsigned long long>(pc.packets_duplicated),
        static_cast<unsigned long long>(pc.frames_dropped_timeout),
        static_cast<unsigned long long>(pc.frames_dropped_conflict),
        static_cast<unsigned long long>(pc.frames_dropped_capacity));
  }

  void assemble_runtime(std::unique_ptr<IAstrallSdk> sdk,
                        std::unique_ptr<INetworkPreflight> preflight) {
    RuntimeConfig cfg;
    cfg.interface = node.get_parameter("interface").as_string();
    cfg.host_address = node.get_parameter("host_address").as_string();
    cfg.robot_address = node.get_parameter("robot_address").as_string();

    cfg.init_timeout_ms = static_cast<std::uint32_t>(
        node.get_parameter("sdk.init_timeout_ms").as_int());
    cfg.sdk_call_timeout_ms = static_cast<std::uint32_t>(
        node.get_parameter("sdk.call_timeout_ms").as_int());
    cfg.heartbeat_call_timeout_ms = static_cast<std::uint32_t>(
        node.get_parameter("sdk.heartbeat_call_timeout_ms").as_int());
    cfg.heartbeat_max_failures = static_cast<std::uint32_t>(
        node.get_parameter("sdk.heartbeat_max_failures").as_int());

    cfg.heartbeat_period = std::chrono::milliseconds(
        node.get_parameter("timing.heartbeat_period_ms").as_int());
    cfg.motion_refresh_period = std::chrono::milliseconds(
        node.get_parameter("timing.motion_refresh_period_ms").as_int());
    cfg.state_poll_period = std::chrono::milliseconds(
        node.get_parameter("timing.state_poll_period_ms").as_int());
    cfg.mode_timeout_ms = std::chrono::milliseconds(
        node.get_parameter("timing.mode_timeout_ms").as_int());
    cfg.reconnect_initial_delay_ms = std::chrono::milliseconds(
        node.get_parameter("timing.reconnect_initial_delay_ms").as_int());
    cfg.reconnect_max_delay_ms = std::chrono::milliseconds(
        node.get_parameter("timing.reconnect_max_delay_ms").as_int());

    cfg.deadman_ms = std::chrono::milliseconds(
        node.get_parameter("safety.command_deadman_ms").as_int());
    cfg.queue_capacity =
        static_cast<std::size_t>(node.get_parameter("safety.queue_capacity").as_int());

    cfg.imu_freq = imu_freq_;
    cfg.sport_freq = sport_freq_;
    cfg.enable_lidar_stream = lidar_enabled_;

    if (!sdk) {
      sdk = std::make_unique<DirectAstrallSdk>();
    }
    if (!preflight) {
      preflight = std::make_unique<LinuxNetworkPreflight>();
    }
    sdk_call_timeout_ms_ = cfg.sdk_call_timeout_ms;
    runtime_.emplace(std::move(sdk), std::move(preflight), clock, cfg,
                     &observer);
    runtime_->start();
  }

  // ------------------------------------------------------------------
  // Publishing helpers.
  // ------------------------------------------------------------------
  void publish_robot_state() {
    if (!robot_state_pub_) {
      return;
    }
    RobotStateMsg msg;
    SdkSnapshot snapshot;
    bool linked = false;
    bool authority = false;
    bool estop = false;
    std::string last_error;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      snapshot = snapshot_;
      linked = status_linked_;
      authority = status_authority_;
      estop = estop_latched_;
      last_error = last_error_;
    }
    msg.header.stamp = node.get_clock()->now();
    msg.ssh_connected = false;
    msg.agent_connected = false;
    msg.sdk_linked = linked;
    msg.control_authority = authority;
    msg.emergency_stop = estop;
    msg.joint_interface_available = false;
    msg.camera_available = false;
    msg.odometry_scale_verified =
        node.get_parameter("odometry.scale_verified").as_bool();
    msg.system_status = snapshot.system_status;
    msg.error_code = snapshot.error_code;
    msg.warning_code = snapshot.warning_code;
    msg.sport_status = snapshot.sport_status;
    msg.battery_percentage = snapshot.battery_percentage;
    msg.battery_temperature = snapshot.battery_temperature;
    msg.battery_voltage = snapshot.battery_voltage;
    msg.battery_cycle_count = snapshot.battery_cycle_count;
    msg.charge_status = snapshot.charge_status;
    msg.wheel_speed = snapshot.wheel_speed;
    msg.protocol_rx_drops = 0;
    msg.rejected_joint_commands =
        rejected_joint_commands_.load(std::memory_order_relaxed);
    msg.last_error = std::move(last_error);
    robot_state_pub_->publish(msg);
  }

  void publish_tf() {
    std::optional<geometry_msgs::msg::Pose> pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose = last_pose_;
    }
    if (!pose) {
      return;
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node.get_clock()->now();
    t.header.frame_id = frames_.odom;
    t.child_frame_id = frames_.base;
    t.transform.translation.x = pose->position.x;
    t.transform.translation.y = pose->position.y;
    t.transform.translation.z = pose->position.z;
    t.transform.rotation = pose->orientation;
    tf_broadcaster_->sendTransform(t);
  }

  // ------------------------------------------------------------------
  // IMU helpers.
  // ------------------------------------------------------------------
  bool imu_sample_finite(const ImuSample& sample) const {
    for (std::size_t i = 0; i < 3; ++i) {
      if (!finite_float(sample.accelerometer[i]) ||
          !finite_float(sample.gyroscope[i])) {
        return false;
      }
    }
    for (std::size_t i = 0; i < 4; ++i) {
      if (!finite_float(sample.quaternion[i])) {
        return false;
      }
    }
    return finite_float(sample.pitch) && finite_float(sample.roll) &&
           finite_float(sample.yaw) && finite_float(sample.odom_x) &&
           finite_float(sample.odom_y);
  }

  // Fills target (xyzw) from the configured source order; normalizes unless
  // the input norm is ~0. Returns false on a zero norm.
  bool orientation_quaternion(const ImuSample& sample,
                              geometry_msgs::msg::Quaternion& target) {
    float x, y, z, w;
    if (quaternion_order_wxyz_) {
      x = sample.quaternion[1];
      y = sample.quaternion[2];
      z = sample.quaternion[3];
      w = sample.quaternion[0];
    } else {
      x = sample.quaternion[0];
      y = sample.quaternion[1];
      z = sample.quaternion[2];
      w = sample.quaternion[3];
    }
    const float norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!finite_float(norm) ||
        norm < std::numeric_limits<float>::epsilon() * 4.0f) {
      return false;
    }
    target.x = x / norm;
    target.y = y / norm;
    target.z = z / norm;
    target.w = w / norm;
    return true;
  }

  void update_last_error(const std::string& message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = message;
  }

  void update_last_error_owned(std::string message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = std::move(message);
  }

  bool last_pose_indicator() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return last_pose_.has_value();
  }

  static float clamp_unit(float v) {
    return std::max(-1.0f, std::min(1.0f, v));
  }

  // ------------------------------------------------------------------
  // Members. Declaration order matters: clock, observer and node must outlive
  // runtime_; runtime_ is declared last and emplaced in the ctor body.
  // ------------------------------------------------------------------
  HypertronDriverNode& node;
  SteadyMonotonicClock clock;
  NodeObserver observer;

  // Parameters resolved at construction.
  SubscriptionFrequency imu_freq_{SubscriptionFrequency::Hz50};
  SubscriptionFrequency sport_freq_{SubscriptionFrequency::Hz50};
  bool quaternion_order_wxyz_{false};
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
  std::uint32_t sdk_call_timeout_ms_{5000U};
  struct Frames {
    std::string imu{"imu_link"};
    std::string odom{"odom"};
    std::string base{"base_link"};
  } frames_;

  // LiDAR bypass stream configuration (resolved from parameters).
  bool lidar_enabled_{true};
  std::string points_topic_{"/points"};
  std::string odom_lidar_topic_{"/odom_lidar"};
  struct LidarParams {
    std::uint16_t point_cloud_port{6100};
    std::uint16_t odometry_port{6101};
    std::string bind_ip{"0.0.0.0"};
    std::string source_ip;
    std::string frame_id{"lidar"};
    int frame_timeout_ms{300};
    std::size_t max_parallel_frames{4};
    std::size_t max_points_per_frame{2000000};
    std::size_t max_packets_per_frame{4096};
    double point_position_scale{1.0e-3};
    double odom_position_scale{1.0e-6};
    double odom_quaternion_scale{1.0};
  } lidar_;
  struct OdomLidarParams {
    std::string parent_frame{"odom"};
    std::string child_frame{"lidar"};
    std::vector<double> pose_covariance_diagonal{0.05, 0.05, 0.05, 0.01, 0.01, 0.01};
  } odom_lidar_;

  // Publishers / subscribers / services.
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<RobotStateMsg>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_lidar_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr authority_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Telemetry/state guarded by state_mutex_. Callbacks and the publishing
  // helpers share it; never call runtime_ while holding it.
  std::mutex state_mutex_;
  SdkSnapshot snapshot_{};
  bool status_linked_{false};
  bool status_authority_{false};
  bool estop_latched_{false};
  std::string last_error_;

  std::atomic<std::uint32_t> rejected_joint_commands_{0};
  std::atomic<std::uint32_t> rejected_nonfinite_cmd_vel_{0};

  // Latest odometry pose, guarded by pose_mutex_, used for TF.
  std::mutex pose_mutex_;
  std::optional<geometry_msgs::msg::Pose> last_pose_;

  // LiDAR bypass stream receivers. Declared before runtime_ and explicitly
  // stopped before runtime_->stop() in the destructor, so no receiver thread
  // can publish after the runtime (or their publishers) is torn down. Each
  // owns its receive thread and its UDP socket.
  std::unique_ptr<LidarStreamReceiver> point_cloud_receiver_;
  std::unique_ptr<LidarStreamReceiver> odometry_receiver_;

  // The runtime owns the SDK. Declared last; emplaced in the ctor body and
  // stopped in the destructor before observer_ disappears.
  std::optional<DirectDriverRuntime> runtime_;
};

HypertronDriverNode::HypertronDriverNode(
    rclcpp::NodeOptions options, std::unique_ptr<IAstrallSdk> sdk,
    std::unique_ptr<INetworkPreflight> preflight,
    std::unique_ptr<ILidarDatagramSource> point_cloud_source,
    std::unique_ptr<ILidarDatagramSource> odometry_source)
    : rclcpp::Node("hypertron_driver", options),
      impl_(std::make_unique<Impl>(*this, std::move(sdk), std::move(preflight),
                                   std::move(point_cloud_source),
                                   std::move(odometry_source))) {}

HypertronDriverNode::~HypertronDriverNode() = default;

}  // namespace hypertron_ros2_bridge
