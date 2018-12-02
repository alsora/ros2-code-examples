

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "simple_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimplePublisherNode> node = std::make_shared<SimplePublisherNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}


