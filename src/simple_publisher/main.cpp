

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


rclcpp::Node::SharedPtr g_node = nullptr;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_publisher");


  //rmw_qos_profile_t custom_qos_profile;
  //custom_qos_profile.depth = 1;
  //custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = g_node->create_publisher<std_msgs::msg::String>("my_topic", rmw_qos_profile_default);

  int msg_count = 0;

  rclcpp::WallRate loop_rate(2);


  while (rclcpp::ok()){

    std_msgs::msg::String message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(msg_count++);

    publisher->publish(message);

    RCLCPP_INFO(g_node->get_logger(), "Published: '%s'", message.data.c_str());

    loop_rate.sleep();

  }


  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}


