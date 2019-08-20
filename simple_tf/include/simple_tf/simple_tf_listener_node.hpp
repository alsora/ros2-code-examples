#ifndef __SIMPLE_TF_LISTENER_NODE_HPP__
#define __SIMPLE_TF_LISTENER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class SimpleTFListenerNode : public rclcpp::Node {

public:

  SimpleTFListenerNode(std::string name = "simple_tf_listener");

private:

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif // __SIMPLE_TF_LISTENER_NODE_HPP__