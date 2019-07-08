#include "simple_subscriber/simple_subscriber_node.hpp"


using namespace std::chrono_literals;

SimpleSubscriberNode::SimpleSubscriberNode(std::string name) : Node(name)
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(10)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

    _subscriber = this->create_subscription<std_msgs::msg::String>(
        "my_topic",
        qos_profile,
        std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriber created!!");
}


void SimpleSubscriberNode::simple_callback(const std_msgs::msg::String::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "Received msg: '%s'", msg->data.c_str());

}