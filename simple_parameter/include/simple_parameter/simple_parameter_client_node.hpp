#ifndef __SIMPLE_PARAMETER_CLIENT_NODE_HPP__
#define __SIMPLE_PARAMETER_CLIENT_NODE_HPP__

#include "rclcpp/rclcpp.hpp"


class SimpleParameterClientNode : public rclcpp::Node {

public:

    SimpleParameterClientNode();

    void get_parameters();
    void set_wrong_parameters();
    void set_correct_parameters();

private:
    void parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    double _wheel_radius;

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr _param_events_subscriber;
    rclcpp::AsyncParametersClient::SharedPtr _param_client;
};

#endif // __SIMPLE_PARAMETER_CLIENT_NODE_HPP__