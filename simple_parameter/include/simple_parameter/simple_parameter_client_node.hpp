#ifndef __SIMPLE_PARAMETER_CLIENT_NODE_HPP__
#define __SIMPLE_PARAMETER_CLIENT_NODE_HPP__

#include "rclcpp/rclcpp.hpp"


class SimpleParameterClientNode : public rclcpp::Node {

public:

    SimpleParameterClientNode();

private:

    void parameters_init();

    void parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    double wheel_radius;

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr _param_events_subscriber;
    rclcpp::AsyncParametersClient::SharedPtr _param_client;

};

#endif // __SIMPLE_PARAMETER_CLIENT_NODE_HPP__