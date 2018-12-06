#include "simple_parameters/simple_parameters_client_node.hpp"


SimpleParametersClientNode::SimpleParametersClientNode() : Node("simple_parameters_client")
{

    rmw_qos_profile_t parameter_events_qos_profile = rmw_qos_profile_default;
    parameter_events_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    parameter_events_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    _param_events_subscriber = this->create_subscription<rcl_interfaces::msg::ParameterEvent>("parameter_events",
        std::bind(&SimpleParametersClientNode::parameter_events_callback, this, std::placeholders::_1),
        parameter_events_qos_profile);


    _param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "simple_parameters_server");

    while (!_param_client->wait_for_service(std::chrono::milliseconds(250))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            assert(0);
        }
        RCLCPP_INFO(this->get_logger(), "Parameters server not available, waiting again...");
    }

    wheel_radius = 0;

    RCLCPP_INFO(this->get_logger(), "Parameters Client created!!");

}


void SimpleParametersClientNode::parameters_init()
{

    std::shared_future<std::vector<rclcpp::Parameter> > get_parameters_result = _param_client->get_parameters({
        "wheels.radius"
    });

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_parameters_result) != rclcpp::executor::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(this->get_logger(), "get_parameters service call failed.");
        assert(0);
    }

    std::vector<rclcpp::Parameter> params = get_parameters_result.get();
    rclcpp::Parameter wheel_radius_parameter = params[0];

    if (wheel_radius_parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
        RCLCPP_ERROR(this->get_logger(), "Parameter \"%s\" not set on the server!", wheel_radius_parameter.get_name().c_str());
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Got Paramter \"%s\" with value %f", wheel_radius_parameter.get_name().c_str(), wheel_radius_parameter.as_double());
        wheel_radius = wheel_radius_parameter.as_double();
    }



}


void SimpleParametersClientNode::parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{

    std::stringstream ss;
    ss << "\nParameter event:\n new parameters:";
    for (rcl_interfaces::msg::Parameter & new_parameter : event->new_parameters) {
        ss << "\n  " << new_parameter.name << ": "<< new_parameter.value.double_value;
        if (new_parameter.name == "wheels.radius"){
            wheel_radius = new_parameter.value.double_value;
        }
    }
    ss << "\n changed parameters:";
    for (rcl_interfaces::msg::Parameter & changed_parameter : event->changed_parameters) {
        ss << "\n  " << changed_parameter.name << ": "<< changed_parameter.value.double_value;
        if (changed_parameter.name == "wheels.radius"){
            wheel_radius = changed_parameter.value.double_value;
        }
    }
    ss << "\n deleted parameters:";
    for (rcl_interfaces::msg::Parameter & deleted_parameter : event->deleted_parameters) {
        ss << "\n  " << deleted_parameter.name;
        if (deleted_parameter.name == "wheels.radius"){
            wheel_radius = 0;
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str())

}