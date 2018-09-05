

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


rclcpp::Node::SharedPtr g_node = nullptr;


void my_callback(const std_msgs::msg::String::SharedPtr msg)
{

  std::cout<<"Received msg: "<< msg->data<<std::endl;

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_subscriber");


  //rmw_qos_profile_t custom_qos_profile;
  //custom_qos_profile.depth = 1;
  //custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber = g_node->create_subscription<std_msgs::msg::String>("my_topic", my_callback, rmw_qos_profile_default);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}


