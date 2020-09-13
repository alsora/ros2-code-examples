

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "simple_interfaces/msg/stamped_string.hpp"
#include "simple_interfaces/msg/stamped_boolean.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("publisher")
  {
    msg_count = 0;

    rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(1)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .avoid_ros_namespace_conventions(false);

    _stamped_string_publisher = this->create_publisher<simple_interfaces::msg::StampedString>("stamped_string_topic", custom_qos_profile);
    _stamped_boolean_publisher = this->create_publisher<simple_interfaces::msg::StampedBoolean>("stamped_boolean_topic", custom_qos_profile);

    _pub_string_timer = this->create_wall_timer(500ms, std::bind(&MyNode::publish_string, this));
    _pub_approximate_timer = this->create_wall_timer(1500ms, std::bind(&MyNode::publish_approximate, this));
    _pub_exact_timer = this->create_wall_timer(3000ms, std::bind(&MyNode::publish_exact, this));
  }

private:
  void publish_string()
  {
    if ((msg_count+1) % 3 == 0){
      return;
    }

    msg_count++;
    simple_interfaces::msg::StampedString message = simple_interfaces::msg::StampedString();
    message.header.stamp = this->now();
    message.data = "Hello, world " + std::to_string(msg_count);
    _stamped_string_publisher->publish(message);
  }

  void publish_approximate()
  {
    if ((msg_count+1) % 6 == 0){
      return;
    }

    msg_count++;

    simple_interfaces::msg::StampedString message = simple_interfaces::msg::StampedString();
    message.header.stamp = this->now();
    message.data = "Hello, world " + std::to_string(msg_count);
    _stamped_string_publisher->publish(message);

    simple_interfaces::msg::StampedBoolean message2 = simple_interfaces::msg::StampedBoolean();
    message2.header.stamp = this->now();
    message2.data = (msg_count % 2);
    _stamped_boolean_publisher->publish(message2);

    RCLCPP_INFO(this->get_logger(), "Published string and bool approximate msg");
  }

  void publish_exact()
  {
    msg_count++;
    rclcpp::Time t = this->now();

    simple_interfaces::msg::StampedString message = simple_interfaces::msg::StampedString();
    message.header.stamp = t;
    message.data = "Hello, world " + std::to_string(msg_count);
    _stamped_string_publisher->publish(message);

    simple_interfaces::msg::StampedBoolean message2 = simple_interfaces::msg::StampedBoolean();
    message2.header.stamp = t;
    message2.data = (msg_count % 2);
    _stamped_boolean_publisher->publish(message2);

    RCLCPP_INFO(this->get_logger(), "Published string and bool exact msg");
  }

  rclcpp::Publisher<simple_interfaces::msg::StampedString>::SharedPtr _stamped_string_publisher;
  rclcpp::Publisher<simple_interfaces::msg::StampedBoolean>::SharedPtr _stamped_boolean_publisher;

  rclcpp::TimerBase::SharedPtr _pub_string_timer;
  rclcpp::TimerBase::SharedPtr _pub_approximate_timer;
  rclcpp::TimerBase::SharedPtr _pub_exact_timer;

  int msg_count;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
