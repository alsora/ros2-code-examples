#include <memory>
#include <iostream>
#include <thread>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "simple_memory_test/msg/array2mb.hpp"



class SimpleSubscriberNode : public rclcpp::Node {

public:

    SimpleSubscriberNode(std::string name) : Node(name)
    {

        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

        _sub = this->create_subscription<simple_memory_test::msg::Array2mb>( "chatter",
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1),
            custom_qos_profile);

    }

private:

    void simple_callback(const simple_memory_test::msg::Array2mb::SharedPtr msg)
    {
        (void)msg;
    }

    rclcpp::Subscription<simple_memory_test::msg::Array2mb>::SharedPtr _sub;

};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimpleSubscriberNode> sub_node = std::make_shared<SimpleSubscriberNode>("this_node");

    rclcpp::spin(sub_node);

    rclcpp::shutdown();

    return 0;
}