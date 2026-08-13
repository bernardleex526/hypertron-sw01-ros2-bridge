#include <rclcpp/rclcpp.hpp>

#include "hypertron_ros2_bridge/bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hypertron_ros2_bridge::HypertronBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  // /emergency_stop 服务由桥接节点内部的专用单线程执行器处理（阻塞式服务
  // 回调最多等待 agent ACK），这里只把默认回调组交给多线程执行器，避免同一
  // 回调组被两个执行器争用（rclcpp 会直接抛异常）。
  executor.add_callback_group(
      node->get_node_base_interface()->get_default_callback_group(),
      node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
