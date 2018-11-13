#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

#include "my_interfaces/msg/stamped_string.hpp"
#include "my_interfaces/msg/stamped_boolean.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

using StampedStringMsg = my_interfaces::msg::StampedString;
using StampedBooleanMsg = my_interfaces::msg::StampedBoolean;

rclcpp::Node::SharedPtr g_node = nullptr;



void exact_sync_callback(const StampedStringMsg::SharedPtr string_msg, const StampedBooleanMsg::SharedPtr bool_msg)
{

  std::cout<<"Received EXACT msg: "<< string_msg->data<< " and msg " << bool_msg->data<<std::endl;

}

void approximate_sync_callback(const StampedStringMsg::SharedPtr string_msg, const StampedBooleanMsg::SharedPtr bool_msg)
{

  std::cout<<"Received APPROXIMATE msg: "<< string_msg->data<< " and msg " << bool_msg->data<<std::endl;

}


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_time_sync");

  // define quality of service: all messages that you want to receive must have the same
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  // create subscribers to the topics of interest
  message_filters::Subscriber<StampedStringMsg> string_sub(g_node, "stamped_string_topic", custom_qos_profile);
  message_filters::Subscriber<StampedBooleanMsg> bool_sub(g_node, "stamped_boolean_topic", custom_qos_profile);


  /**
   * create an exact time filter. 
   * this can be done in two different ways
   */ 

  // method 1: exact time policy simplified API
  //message_filters::TimeSynchronizer<StampedStringMsg, StampedBooleanMsg> syncExact(string_sub, bool_sub, 10);

  // method 2: exact time policy
  typedef message_filters::sync_policies::ExactTime<StampedStringMsg, StampedBooleanMsg> exact_policy;
  message_filters::Synchronizer<exact_policy>syncExact(exact_policy(10), string_sub, bool_sub);

  // register the exact time callback
  syncExact.registerCallback(exact_sync_callback);

  /**
   * create an approximate time filter. 
   */ 

  typedef message_filters::sync_policies::ApproximateTime<StampedStringMsg, StampedBooleanMsg> approximate_policy;
  message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(10), string_sub, bool_sub);
  
  // register the approximate time callback
  syncApproximate.registerCallback(approximate_sync_callback);

  rclcpp::spin(g_node);

  return 0;

}

