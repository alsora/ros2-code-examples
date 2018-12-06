#ifndef __TIMER_CLIENT_NODE_HPP__
#define __TIMER_CLIENT_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/srv/get_image.hpp"

class TimerClientNode : public rclcpp::Node
{

public:

    TimerClientNode(std::string name = "timer_client");

private:

    void make_request();

    void response_received_callback(rclcpp::Client<simple_interfaces::srv::GetImage>::SharedFuture result_future);

    rclcpp::Client<simple_interfaces::srv::GetImage>::SharedPtr _client;
    rclcpp::TimerBase::SharedPtr _request_timer;

    std::chrono::high_resolution_clock::time_point _request_sent_time;

};

#endif // __TIMER_CLIENT_NODE_HPP__