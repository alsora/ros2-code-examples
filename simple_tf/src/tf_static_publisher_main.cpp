#include "rclcpp/rclcpp.hpp"

#include "simple_tf/simple_tf_static_publisher_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleTFStaticPublisherNode> node = std::make_shared<SimpleTFStaticPublisherNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


