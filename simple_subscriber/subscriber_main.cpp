

#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "simple_subscriber.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleSubscriberNode> node = std::make_shared<SimpleSubscriberNode>();

  rclcpp::spin(node);


  rclcpp::shutdown();
  return 0;
}


