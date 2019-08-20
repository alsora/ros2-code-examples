#include "simple_tf/simple_tf_listener_node.hpp"

#include "tf2_ros/create_timer_ros.h"


using namespace std::chrono_literals;

SimpleTFListenerNode::SimpleTFListenerNode(std::string name) : Node(name)
{
  /*
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .keep_last(10)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .avoid_ros_namespace_conventions(false);
  */

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  RCLCPP_INFO(this->get_logger(), "Listener created!!");
}


