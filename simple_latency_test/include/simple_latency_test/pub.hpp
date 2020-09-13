#pragma once

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

template<typename MsgType>
class SimplePublisherNode : public rclcpp::Node
{
public:
    SimplePublisherNode() : Node("publisher_node")
    {
        _publisher = this->create_publisher<MsgType>("my_topic", 10);

        auto period = 500ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));

        msg_count = 0;
    }

    void publish()
    {
        RCLCPP_DEBUG(this->get_logger(), "Publish new message %d", msg_count);

        auto message = std::make_unique<MsgType>();
        message->header.stamp = this->now();

        _publisher->publish(std::move(message));

        msg_count++;
    }

private:
    typename rclcpp::Publisher<MsgType>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    int msg_count;
};
