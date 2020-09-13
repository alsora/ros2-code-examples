#include <memory>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "simple_latency_test/msg/array750kb.hpp"
#include "simple_latency_test/pub.hpp"
#include "simple_latency_test/sub.hpp"

using namespace std::chrono_literals;

using MsgType = simple_latency_test::msg::Array750kb;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto sub_node = std::make_shared<SimpleSubscriberNode<MsgType>>();
    auto pub_node = std::make_shared<SimplePublisherNode<MsgType>>();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::thread t_s([=]() { rclcpp::spin(sub_node); });
    t_s.detach();

    std::thread t_p([=]() { rclcpp::spin(pub_node); });
    t_p.detach();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    rclcpp::shutdown();

    std::cout<<"Mean latency: "<< sub_node->mean<<std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
