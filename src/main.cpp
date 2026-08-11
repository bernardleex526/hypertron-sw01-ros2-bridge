#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hypertron_bridge");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
