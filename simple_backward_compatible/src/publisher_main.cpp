#include <memory>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "simple_backward_compatible/msg/optional_fields.hpp"

using namespace std::chrono_literals;
using OptionalFieldsMsg = simple_backward_compatible::msg::OptionalFields;

class SimplePublisherNode : public rclcpp::Node {

public:

    SimplePublisherNode() : Node("publisher_node")
    {
        _publisher = this->create_publisher<OptionalFieldsMsg>("my_topic");

        auto period = 500ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));
        _count = 0;
    }

    void publish()
    {
        OptionalFieldsMsg::SharedPtr message(new OptionalFieldsMsg());

        _count ++;
        message->data = "Hello, world! " + std::to_string(_count);
        #ifdef BUILD_V2
            message->optional_boolean.push_back(true);
            message->optional_vector = { 10, 20, 30 };
        #endif

        _publisher->publish(message);

        #ifdef BUILD_V2
            RCLCPP_INFO(this->get_logger(), "Publishing VERSION2: '%s'", message->data.c_str());
        #else
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message->data.c_str());
        #endif
    }

private:
    rclcpp::Publisher<OptionalFieldsMsg>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    int _count;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimplePublisherNode> node = std::make_shared<SimplePublisherNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
