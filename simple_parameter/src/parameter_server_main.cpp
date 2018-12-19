#include "rclcpp/rclcpp.hpp"

#include "simple_parameter/simple_parameter_server_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleParameterServerNode> node = std::make_shared<SimpleParameterServerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


