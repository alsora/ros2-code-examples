
#include "rclcpp/rclcpp.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char ** argv)
{

    // initialize ros2 and create a node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("simple_node");

    std::cout<<"Printing some messages with default verbosity level INFO"<<std::endl;

    // print some log messages
    RCLCPP_DEBUG(node->get_logger(), "This is a RCLCPP_DEBUG message");
    RCLCPP_INFO(node->get_logger(), "This is a RCLCPP_INFO message");
    RCLCPP_WARN(node->get_logger(), "This is a RCLCPP_WARN message");
    RCLCPP_ERROR(node->get_logger(), "This is a RCLCPP_ERROR message");

    // change the logger verbosity level at runtime
    std::cout<<"Changing verbosity level to WARN"<<std::endl;
    auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(node->get_logger(), "Error setting severity: %s", rcutils_get_error_string());
        rcutils_reset_error();
    }

    // print again some log messages to see the differences
    RCLCPP_DEBUG(node->get_logger(), "This is a RCLCPP_DEBUG message");
    RCLCPP_INFO(node->get_logger(), "This is a RCLCPP_INFO message");
    RCLCPP_WARN(node->get_logger(), "This is a RCLCPP_WARN message");
    RCLCPP_ERROR(node->get_logger(), "This is a RCLCPP_ERROR message");

    // create a logger not related to any node
    rclcpp::Logger logger = rclcpp::get_logger("my_logger");

    RCLCPP_WARN(logger, "This is a RCLCPP_WARN message from a logger not related to any node");


    rclcpp::shutdown();
    node = nullptr;
    return 0;
}