#include "rclcpp/rclcpp.hpp"

#include "simple_client_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleClientNode> node = std::make_shared<SimpleClientNode>();

  node->run_request_loop();

  rclcpp::shutdown();

  return 0;
}

