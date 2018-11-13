

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "my_interfaces/msg/stamped_string.hpp"
#include "my_interfaces/msg/stamped_boolean.hpp"

rclcpp::Node::SharedPtr g_node = nullptr;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_publisher");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  rclcpp::Publisher<my_interfaces::msg::StampedString>::SharedPtr stamped_string_publisher = g_node->create_publisher<my_interfaces::msg::StampedString>("stamped_string_topic", custom_qos_profile);
  rclcpp::Publisher<my_interfaces::msg::StampedBoolean>::SharedPtr stamped_boolean_publisher = g_node->create_publisher<my_interfaces::msg::StampedBoolean>("stamped_boolean_topic", custom_qos_profile);

  int msg_count = 0;

  rclcpp::WallRate loop_rate(2);


  while (rclcpp::ok()){

    msg_count++;

    rclcpp::Time t = g_node->now();

    my_interfaces::msg::StampedString message2 = my_interfaces::msg::StampedString();
    message2.header.stamp = t;
    message2.data = "Hello, world " + std::to_string(msg_count);
    stamped_string_publisher->publish(message2);

    if (msg_count % 3 == 0){

      rclcpp::Time t2;

      if (msg_count % 6 == 0){
        t2 = t;
        RCLCPP_INFO(g_node->get_logger(), "Published string and bool synced msg");
      }
      else{
        t2 = g_node->now();
        RCLCPP_INFO(g_node->get_logger(), "Published string and bool msg");
      }

      my_interfaces::msg::StampedBoolean message3 = my_interfaces::msg::StampedBoolean();
      message3.header.stamp = t2;
      message3.data = (msg_count % 2);
      stamped_boolean_publisher->publish(message3);
    }
    else {
        RCLCPP_INFO(g_node->get_logger(), "Published string msg");
    }




    loop_rate.sleep();

  }


  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}


