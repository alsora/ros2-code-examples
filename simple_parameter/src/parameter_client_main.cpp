#include "rclcpp/rclcpp.hpp"

#include "simple_parameter/simple_parameter_client_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleParameterClientNode> node = std::make_shared<SimpleParameterClientNode>();

  node->get_parameters();
  node->set_wrong_parameters();
  node->set_correct_parameters();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
  return 0;
}


