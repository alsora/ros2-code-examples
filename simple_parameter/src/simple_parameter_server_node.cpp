#include "simple_parameter/simple_parameter_server_node.hpp"


SimpleParameterServerNode::SimpleParameterServerNode() : Node("simple_parameters_server")
{

    RCLCPP_INFO(this->get_logger(), "Parameters Server created!!");

    this->parameters_init();

    auto existing_callback = this->set_on_parameters_set_callback(nullptr);
    auto param_change_callback =
      [this, existing_callback](std::vector<rclcpp::Parameter> parameters)
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        // first call the existing callback, if there was one
        if (nullptr != existing_callback) {
          result = existing_callback(parameters);
          // if the existing callback failed, go ahead and return the result
          if (!result.successful) {
            return result;
          }
        }
        result.successful = true;
        for (auto parameter : parameters) {
          rclcpp::ParameterType parameter_type = parameter.get_type();
          if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
            RCLCPP_WARN(this->get_logger(),
              "Trying to delete parameter '%s'. Deleting parameters is not allowed",
              parameter.get_name().c_str()
            );
            result.successful = false;
            result.reason = "parameter \'"+parameter.get_name()+"\' cannot be deleted";
          } else {
            // Here I handle special parameters with special conditions
            if (parameter.get_name().compare("wheels.magic") == 0) {
              if (rclcpp::ParameterType::PARAMETER_INTEGER != parameter_type) {
                RCLCPP_WARN(this->get_logger(),
                  "Trying to set parameter '%s' with wrong type",
                  parameter.get_name().c_str()
                );
                result.successful = false;
                result.reason = "parameter \'"+parameter.get_name()+"\' must be an integer";
              } else if (parameter.as_int() != 2 && parameter.as_int() % 42 !=0) {
                RCLCPP_WARN(this->get_logger(),
                  "Trying to set parameter '%s' to a not allowed value",
                  parameter.get_name().c_str()
                );
                result.successful = false;
                result.reason = "parameter \'"+parameter.get_name()+"\' accepts only 2 or multiple of 42 values";
              } else {
                result.successful &= true;
              }
            } else if (parameter.get_name().compare("two_integers") == 0) {
              if (rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY != parameter_type) {
                RCLCPP_WARN(this->get_logger(),
                  "Trying to set parameter '%s' with wrong type",
                  parameter.get_name().c_str()
                );
                result.successful = false;
                result.reason = "parameter \'"+parameter.get_name()+"\' must be an integer array";
              } else if (parameter.as_integer_array().size() != 2) {
                RCLCPP_WARN(this->get_logger(),
                  "Trying to set parameter '%s' not with 2 values",
                  parameter.get_name().c_str()
                );
                result.successful = false;
                result.reason = "parameter \'"+parameter.get_name()+"\' requires 2 integer values to be specified";
              } else if (parameter.as_integer_array()[0] != 42 && parameter.as_integer_array()[1] != 42) {
                RCLCPP_WARN(this->get_logger(),
                  "Trying to set parameter '%s' with none of the values being 42",
                  parameter.get_name().c_str()
                );
                result.successful = false;
                result.reason = "parameter \'"+parameter.get_name()+"\' one of the two values must be 42";
              } else {
                result.successful &= true;
              }
            }
          }
        }
        return result;
      };
    this->set_on_parameters_set_callback(param_change_callback);
}


void SimpleParameterServerNode::parameters_init()
{
    // Declare an integer parameter that can only range from 1 to 10
    std::string speed_param_name = "speed";
    rcl_interfaces::msg::ParameterDescriptor speed_param_descriptor;
    speed_param_descriptor.name = speed_param_name;
    speed_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    speed_param_descriptor.description = "The speed coefficient: ranges from 0 to 10";
    rcl_interfaces::msg::IntegerRange speed_param_range;
    speed_param_range.from_value = 0;
    speed_param_range.to_value = 10;
    speed_param_range.step = 1;
    speed_param_descriptor.integer_range.push_back(speed_param_range);
    int speed_default = 5;
    this->declare_parameter(speed_param_name, speed_default, speed_param_descriptor);

    // Declare a double parameter
    std::string wheels_radius_param_name = "wheels.radius";
    rcl_interfaces::msg::ParameterDescriptor wheels_radius_param_descriptor;
    wheels_radius_param_descriptor.name = wheels_radius_param_name;
    wheels_radius_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    wheels_radius_param_descriptor.description = "The radius of the wheels";
    double wheels_radius_default = 1.1;
    this->declare_parameter(wheels_radius_param_name, wheels_radius_default, wheels_radius_param_descriptor);

    // Declare a string parameter that is read only
    std::string wheels_radius_dummy_param_name = "wheels.radius.dummy";
    rcl_interfaces::msg::ParameterDescriptor wheels_radius_dummy_param_descriptor;
    wheels_radius_dummy_param_descriptor.name = wheels_radius_dummy_param_name;
    wheels_radius_dummy_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    wheels_radius_dummy_param_descriptor.description = "This is a dummy read-only parameter";
    wheels_radius_dummy_param_descriptor.read_only = true;
    std::string wheels_radius_dummy_default = "Hello world";
    this->declare_parameter(wheels_radius_dummy_param_name, wheels_radius_dummy_default, wheels_radius_dummy_param_descriptor);

    // Declare an integer parameter with a complex constraint (this has to be manually handled in the callback)
    std::string wheels_magic_param_name = "wheels.magic";
    rcl_interfaces::msg::ParameterDescriptor wheels_magic_param_descriptor;
    wheels_magic_param_descriptor.name = wheels_magic_param_name;
    wheels_magic_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    wheels_magic_param_descriptor.description = "This is the wheel magic number";
    wheels_magic_param_descriptor.additional_constraints =
      "This parameter has complex constraints that must be explained in English. It must be 2 or a multiple of 42";
    int wheels_magic_default = 2;
    this->declare_parameter(wheels_magic_param_name, wheels_magic_default, wheels_magic_param_descriptor);

    // Declare an integer array parameter (it's actually implemented through a vector)
    std::string two_integers_param_name = "two_integers";
    rcl_interfaces::msg::ParameterDescriptor two_integers_param_descriptor;
    two_integers_param_descriptor.name = two_integers_param_name;
    two_integers_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    two_integers_param_descriptor.description = "This is a parameter made of two values";
    two_integers_param_descriptor.additional_constraints = "Both two values must be specified. One of the two must be 42";
    std::vector<int64_t> two_integers_value = {24, 42};
    this->declare_parameter(two_integers_param_name, two_integers_value, two_integers_param_descriptor);

    RCLCPP_INFO(this->get_logger(), "Parameters correctly set on the server");
}