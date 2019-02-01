#include <memory>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "simple_backward_compatible/msg/optional_fields.hpp"

using OptionalFieldsMsg = simple_backward_compatible::msg::OptionalFields;

class SimpleSubscriberNode : public rclcpp::Node {

public:

    SimpleSubscriberNode() : Node("subscriber_node")
    {
        _subscriber = this->create_subscription<OptionalFieldsMsg>("my_topic",
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));
    }


private:

    void simple_callback(const OptionalFieldsMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received msg: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<OptionalFieldsMsg>::SharedPtr _subscriber;
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleSubscriberNode> node = std::make_shared<SimpleSubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


