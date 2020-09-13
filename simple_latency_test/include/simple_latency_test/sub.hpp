#pragma once

#include "rclcpp/rclcpp.hpp"

template<typename MsgType>
class SimpleSubscriberNode : public rclcpp::Node
{
public:

    SimpleSubscriberNode() : Node("subscriber_node")
    {
        _subscriber = this->create_subscription<MsgType>("my_topic", 10,
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));

        msg_count = 0;
        mean = 0;
    }

    int mean;

private:
    void simple_callback(const typename MsgType::SharedPtr msg)
    {
        auto now = this->now();

        // Compute latency
        rclcpp::Time stamp(msg->header.stamp);
        auto lat = std::chrono::nanoseconds((now - stamp).nanoseconds());
        int lat_us = lat.count() / 1000;

        RCLCPP_INFO(this->get_logger(), "Received msg %d with latency %d", msg_count, lat_us);

        int delta = lat_us - mean;
        mean += delta/(msg_count + 1);

        msg_count ++;
    }

    typename rclcpp::Subscription<MsgType>::SharedPtr _subscriber;
    int msg_count;
};
