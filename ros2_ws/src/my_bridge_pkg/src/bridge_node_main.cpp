// bridge_node_main.cpp
#include <rclcpp/rclcpp.hpp>
#include "my_bridge_pkg/bridge_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
