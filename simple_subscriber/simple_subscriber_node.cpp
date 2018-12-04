#include "simple_subscriber_node.hpp"


using namespace std::chrono_literals;

SimpleSubscriberNode::SimpleSubscriberNode(std::string name) : Node(name)
{

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 1;
    custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    _subscriber = this->create_subscription<std_msgs::msg::String>("my_topic",
        std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1),
        custom_qos_profile);

    RCLCPP_INFO(this->get_logger(), "Subscriber created!!");

}


void SimpleSubscriberNode::simple_callback(const std_msgs::msg::String::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "Received msg: '%s'", msg->data.c_str());

}