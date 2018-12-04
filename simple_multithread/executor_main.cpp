#include <chrono>
#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "simple_publisher_node.hpp"
#include "simple_subscriber_node.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimplePublisherNode> pub_node = std::make_shared<SimplePublisherNode>();
    std::shared_ptr<SimpleSubscriberNode> sub_node = std::make_shared<SimpleSubscriberNode>();

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor->add_node(pub_node);
    executor->add_node(sub_node);

    executor->spin();

    rclcpp::shutdown();

    return 0;

}