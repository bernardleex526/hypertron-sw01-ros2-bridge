// Entry point for the hypertron_driver_node executable: spins the
// HypertronDriverNode over the direct ASTRALL SDK runtime.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hypertron_ros2_bridge/driver_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const auto node =
      std::make_shared<hypertron_ros2_bridge::HypertronDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
