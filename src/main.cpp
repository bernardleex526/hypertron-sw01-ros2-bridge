#include <rclcpp/rclcpp.hpp>

#include "hypertron_ros2_bridge/bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hypertron_ros2_bridge::HypertronBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
