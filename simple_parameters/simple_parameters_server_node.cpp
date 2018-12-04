#include "simple_parameters_server_node.hpp"


SimpleParametersServerNode::SimpleParametersServerNode() : Node("simple_parameters_server")
{

    std::vector< rcl_interfaces::msg::SetParametersResult> set_parameters_results = this->set_parameters({
        rclcpp::Parameter("wheels.radius", 1.1),
        rclcpp::Parameter("wheels.radius.dummy", 2.77),
        rclcpp::Parameter("wheels.weight", 0.5)
    });

    for ( const rcl_interfaces::msg::SetParametersResult& result : set_parameters_results) {
        if (!result.successful) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            assert(0);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Parameters Server created!!");

}