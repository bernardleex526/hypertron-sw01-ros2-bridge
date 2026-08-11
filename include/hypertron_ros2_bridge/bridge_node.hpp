#pragma once

#ifdef HYPERTRON_WITH_ROS2

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace hypertron_ros2_bridge {

class HypertronBridgeNode final : public rclcpp::Node {
 public:
  explicit HypertronBridgeNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HypertronBridgeNode() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hypertron_ros2_bridge

#endif
