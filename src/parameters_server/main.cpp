
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

  rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

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

  std::stringstream ss;
  // Get a few of the parameters just set.

  std::vector< rclcpp::Parameter > get_parameters_results = parameters_client->get_parameters({
    "use_odometry",
    "wheels",
    "wheels.weight"});

  for (rclcpp::Parameter &parameter : get_parameters_results)
  {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str());

  rclcpp::shutdown();
  return 0;
}
