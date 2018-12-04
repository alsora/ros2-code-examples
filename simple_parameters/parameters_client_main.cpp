#include "rclcpp/rclcpp.hpp"

#include "simple_parameters_client_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleParametersClientNode> node = std::make_shared<SimpleParametersClientNode>();
  node->parameters_init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


