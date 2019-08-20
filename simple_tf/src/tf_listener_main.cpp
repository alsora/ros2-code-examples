#include "rclcpp/rclcpp.hpp"

#include "simple_tf/simple_tf_listener_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleTFListenerNode> node = std::make_shared<SimpleTFListenerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


