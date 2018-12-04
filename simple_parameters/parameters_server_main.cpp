#include "rclcpp/rclcpp.hpp"

#include "simple_parameters_server_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleParametersServerNode> node = std::make_shared<SimpleParametersServerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


