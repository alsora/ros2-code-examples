#include "rclcpp/rclcpp.hpp"

#include "simple_publisher/simple_publisher_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimplePublisherNode> node = std::make_shared<SimplePublisherNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


