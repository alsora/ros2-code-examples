#include <chrono>
#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "simple_publisher.hpp"
#include "simple_subscriber.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimplePublisherNode> pub_node = std::make_shared<SimplePublisherNode>();
    std::shared_ptr<SimpleSubscriberNode> sub_node = std::make_shared<SimpleSubscriberNode>();

    std::thread pub_thread(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), pub_node);
    std::thread sub_thread(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), sub_node);


    std::this_thread::sleep_for(10s);


    rclcpp::shutdown();

    return 0;

}