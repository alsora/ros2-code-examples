#include "simple_parameter/simple_parameter_client_node.hpp"


SimpleParameterClientNode::SimpleParameterClientNode() : Node("simple_parameters_client")
{
  _param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "simple_parameters_server");

  while (!_param_client->wait_for_service(std::chrono::milliseconds(250))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      assert(0);
    }
    RCLCPP_INFO(this->get_logger(), "Parameters server not available, waiting again...");
  }

  _param_events_subscriber = _param_client->on_parameter_event(
    std::bind(&SimpleParameterClientNode::parameter_events_callback, this, std::placeholders::_1));

  _wheel_radius = 0;

  RCLCPP_INFO(this->get_logger(), "Parameters Client created!!");
}


void SimpleParameterClientNode::get_parameters()
{
  std::shared_future<std::vector<rclcpp::Parameter> > get_parameters_result = _param_client->get_parameters({
    "wheels.radius"
  });

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_parameters_result) != rclcpp::executor::FutureReturnCode::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "get_parameters service call failed.");
    throw std::runtime_error("Not able to get parameters from server");
  }

  std::vector<rclcpp::Parameter> params = get_parameters_result.get();
  rclcpp::Parameter wheel_radius_parameter = params[0];

  if (wheel_radius_parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    RCLCPP_ERROR(this->get_logger(), "Parameter \"%s\" not set on the server!", wheel_radius_parameter.get_name().c_str());
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Got Paramter \"%s\" with value %f", wheel_radius_parameter.get_name().c_str(), wheel_radius_parameter.as_double());
    _wheel_radius = wheel_radius_parameter.as_double();
  }
}


void SimpleParameterClientNode::set_wrong_parameters()
{
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_parameters_result = _param_client->set_parameters({
    rclcpp::Parameter("not_declared_param", 42),
    rclcpp::Parameter("speed", 42),
    rclcpp::Parameter("wheels.radius.dummy", "Hello new World"),
    rclcpp::Parameter("wheels.magic", 18.2)
  });

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_parameters_result) != rclcpp::executor::FutureReturnCode::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "set_parameters service call failed.");
    throw std::runtime_error("Not able to set parameters on the server");
  }

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters = set_parameters_result.get();

  for (const rcl_interfaces::msg::SetParametersResult& result : set_parameters) {
    if (!result.successful) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    }
  }
}

void SimpleParameterClientNode::set_correct_parameters()
{
  std::vector<int64_t> two_integers = {42, 42};

  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_parameters_result = _param_client->set_parameters({
    rclcpp::Parameter("speed", 8),
    rclcpp::Parameter("wheels.radius", 1.6),
    rclcpp::Parameter("wheels.magic", 84),
    rclcpp::Parameter("two_integers", two_integers),
  });

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_parameters_result) != rclcpp::executor::FutureReturnCode::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "set_parameters service call failed.");
    throw std::runtime_error("Not able to set parameters on the server");
  }

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters = set_parameters_result.get();

  for (const rcl_interfaces::msg::SetParametersResult& result : set_parameters) {
    if (!result.successful) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    }
  }
}

void SimpleParameterClientNode::parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::stringstream ss;
  ss << "\nParameter event callback:";
  if (event->new_parameters.size() > 0) {
    ss << "\n new parameters:";
  }
  for (rcl_interfaces::msg::Parameter & new_parameter : event->new_parameters) {
    ss << "\n  " << new_parameter.name;
    if (new_parameter.name == "wheels.radius"){
      _wheel_radius = new_parameter.value.double_value;
    }
  }
  if (event->changed_parameters.size() > 0) {
    ss << "\n changed parameters:";
  }
  for (rcl_interfaces::msg::Parameter & changed_parameter : event->changed_parameters) {
    ss << "\n  " << changed_parameter.name;
    if (changed_parameter.name == "wheels.radius"){
      _wheel_radius = changed_parameter.value.double_value;
    }
  }
  if (event->deleted_parameters.size() > 0) {
    ss << "\n deleted parameters:";
  }
  for (rcl_interfaces::msg::Parameter & deleted_parameter : event->deleted_parameters) {
    ss << "\n  " << deleted_parameter.name;
    if (deleted_parameter.name == "wheels.radius"){
      _wheel_radius = 0;
    }
  }

  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}