#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"
#include "hypertron_ros2_bridge/lidar_stream.hpp"
#include "hypertron_ros2_bridge/network_preflight.hpp"

namespace hypertron_ros2_bridge {

// ROS 2 node that owns the direct-driver runtime and bridges it onto the ROS
// graph: /cmd_vel, /robot_mode and /joint_commands in; /imu/data, /odom and
// /robot_state out; /emergency_stop and /control_authority services. The
// node owns the DirectDriverRuntime as the single consumer of the SDK, so
// construction is non-blocking: runtime_->start() is called before the
// constructor returns but no SDK or network call happens on this thread.
//
// Dependencies run one way (node -> runtime): the observer is a member that
// outlives the runtime, and the runtime is stopped (and later destroyed)
// before the observer goes away, so no callback can fire against destroyed
// publishers.
class HypertronDriverNode : public rclcpp::Node {
 public:
  // sdk / preflight default to the production DirectAstrallSdk and
  // LinuxNetworkPreflight when null; tests inject fakes. point_cloud_source /
  // odometry_source inject the LiDAR UDP datagram transports; when null the
  // node binds its own UdpLidarDatagramSource sockets to the configured
  // lidar.point_cloud_port / lidar.odometry_port (tests inject a programmatic
  // fake).
  HypertronDriverNode(rclcpp::NodeOptions options = rclcpp::NodeOptions(),
                      std::unique_ptr<IAstrallSdk> sdk = nullptr,
                      std::unique_ptr<INetworkPreflight> preflight = nullptr,
                      std::unique_ptr<ILidarDatagramSource> point_cloud_source = nullptr,
                      std::unique_ptr<ILidarDatagramSource> odometry_source = nullptr);
  ~HypertronDriverNode() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hypertron_ros2_bridge
