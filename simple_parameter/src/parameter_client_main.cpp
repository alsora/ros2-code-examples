#include "rclcpp/rclcpp.hpp"

#include "simple_parameter/simple_parameter_client_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleParameterClientNode> node = std::make_shared<SimpleParameterClientNode>();

  node->get_parameters();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  node->set_wrong_parameters();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  node->set_correct_parameters();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


