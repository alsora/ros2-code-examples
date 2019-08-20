#ifndef __SIMPLE_TF_STATIC_PUBLISHER_NODE_HPP__
#define __SIMPLE_TF_STATIC_PUBLISHER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class SimpleTFStaticPublisherNode : public rclcpp::Node {

public:

    SimpleTFStaticPublisherNode(std::string name = "simple_tf_static_publisher");

private:

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _broadcaster;

};

#endif // __SIMPLE_TF_STATIC_PUBLISHER_NODE_HPP__