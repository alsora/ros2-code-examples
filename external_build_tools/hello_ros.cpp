#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{

	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("test_node");

	auto publisher = node->create_publisher<std_msgs::msg::String>("topic");

	RCLCPP_INFO(node->get_logger(), "Hello World from this ROS2 node!");

	
	rclcpp::Rate loop_rate(4);

	int count=0;
	RCLCPP_INFO(node->get_logger(), "Publising messages on \\topic");
	while (rclcpp::ok()){

	auto message = std_msgs::msg::String();
	message.data = "Hello, world! " + std::to_string(count++);

	publisher->publish(message);

	rclcpp::spin_some(node);
    loop_rate.sleep();

	}

	return 0;
}
    
