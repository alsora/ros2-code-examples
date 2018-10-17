
#include <chrono>
#include <iostream>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/parameter.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("parameters_node");

  // create the parameters client
  rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  // initialize the parameters client
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  // set some parameters
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("wheels.radius", 1),
    rclcpp::Parameter("wheels.weight", 0.5),
    rclcpp::Parameter("use_odometry", true)});

  for (rcl_interfaces::msg::SetParametersResult &result : set_parameters_results)
  {
    if (!result.successful)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    }
  }

  // get some parameters
  std::vector< rclcpp::Parameter > get_parameters_results = parameters_client->get_parameters({
    "use_odometry",
    "wheels",
    "wheels.radius"});

  std::stringstream ss;
  for (rclcpp::Parameter &parameter : get_parameters_results)
  {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
      ss << "\nThis parameter does not exist!!";
    }
    ss << "\n---------";
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str());

  // get one parameter (you MUST provide a default value in case the parameter is not present on the server)
  double param = parameters_client->get_parameter("wheels.weight", 10.0);
  RCLCPP_INFO(node->get_logger(), "received parameter %f", param);


  rclcpp::shutdown();
  return 0;
}
