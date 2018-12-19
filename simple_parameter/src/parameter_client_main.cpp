#include "rclcpp/rclcpp.hpp"

#include "simple_parameter/simple_parameter_client_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleParameterClientNode> node = std::make_shared<SimpleParameterClientNode>();

  rclcpp::shutdown();
  return 0;
}


