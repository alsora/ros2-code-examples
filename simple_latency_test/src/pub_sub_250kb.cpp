#include <memory>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "simple_latency_test/msg/array750kb.hpp"

using namespace std::chrono_literals;

using MsgType = simple_latency_test::msg::Array750kb;

class SimplePublisherNode : public rclcpp::Node {

public:

    SimplePublisherNode() : Node("publisher_node")
    {
        _publisher = this->create_publisher<MsgType>("my_topic");

        auto period = 500ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));

        msg_count = 0;

    }

    void publish()
    {
        RCLCPP_INFO(this->get_logger(), "Publish new message %d", msg_count);

        auto message = std::make_shared<MsgType>();
        message->header.stamp = this->now();

        _publisher->publish(message);

        msg_count++;

    }



private:
    rclcpp::Publisher<MsgType>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    int msg_count;

};

class SimpleSubscriberNode : public rclcpp::Node {

public:

    SimpleSubscriberNode() : Node("subscriber_node")
    {
        _subscriber = this->create_subscription<MsgType>("my_topic",
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));

        msg_count = 0;
    }

    int mean;

private:

    void simple_callback(const MsgType::SharedPtr msg)
    {
        auto now = this->now();

        // Compute latency
        rclcpp::Time stamp(msg->header.stamp);
        auto lat = std::chrono::nanoseconds((now - stamp).nanoseconds());
        int lat_us = lat.count() / 1000;

        RCLCPP_INFO(this->get_logger(), "Received msg %d with latency %d", msg_count, lat_us);

        int delta = lat_us - mean;
        mean += (delta/msg_count + 1);

        msg_count ++;
    }

    rclcpp::Subscription<MsgType>::SharedPtr _subscriber;
    int msg_count;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto sub_node = std::make_shared<SimpleSubscriberNode>();

    auto pub_node = std::make_shared<SimplePublisherNode>();


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