#include "simple_tf/simple_tf_static_publisher_node.hpp"


using namespace std::chrono_literals;

SimpleTFStaticPublisherNode::SimpleTFStaticPublisherNode(std::string name) : Node(name)
{

  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .keep_last(10)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .avoid_ros_namespace_conventions(false);


  _broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this, qos_profile);


  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = 1.0;

  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = "camera";

  _broadcaster->sendTransform(msg);


  RCLCPP_INFO(this->get_logger(), "Publisher created!!");
}


