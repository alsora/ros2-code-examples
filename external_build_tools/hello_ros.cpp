#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{

	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("test_node");

	RCLCPP_INFO(node->get_logger(), "Hello World from this ROS2 node!");

	return 0;
}
    
