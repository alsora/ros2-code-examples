#include "simple_publisher/simple_publisher_node.hpp"


using namespace std::chrono_literals;

SimplePublisherNode::SimplePublisherNode(std::string name) : Node(name)
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(10)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

    _publisher = this->create_publisher<std_msgs::msg::String>(
      "my_topic",
      qos_profile);

    auto period = 500ms;
    _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));

    _count = 0;

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");
}


void SimplePublisherNode::publish()
{
    _count++;

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(_count);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    _publisher->publish(message);
}