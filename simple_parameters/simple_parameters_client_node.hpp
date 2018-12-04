
#include "rclcpp/rclcpp.hpp"


class SimpleParametersClientNode : public rclcpp::Node {

public:

    SimpleParametersClientNode();
    void parameters_init();

private:

    void parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr _param_events_subscriber;
    rclcpp::AsyncParametersClient::SharedPtr _param_client;

};