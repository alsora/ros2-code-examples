#include <chrono>
#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "simple_publisher/simple_publisher_node.hpp"
#include "simple_subscriber/simple_subscriber_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimplePublisherNode> pub_node = std::make_shared<SimplePublisherNode>();
    std::shared_ptr<SimpleSubscriberNode> sub_node = std::make_shared<SimpleSubscriberNode>();

    std::thread pub_thread(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), pub_node);
    std::thread sub_thread(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), sub_node);

    // setting threads priority is not mandatory
    sched_param sch;
    int policy;
    pthread_getschedparam(pub_thread.native_handle(), &policy, &sch);
    sch.sched_priority = 10;
    pthread_setschedparam(pub_thread.native_handle(), SCHED_FIFO, &sch);

    pub_thread.detach();
    sub_thread.detach();

    std::this_thread::sleep_for(10s);

    rclcpp::shutdown();

    return 0;
}