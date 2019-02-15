#include <memory>
#include <iostream>
#include <thread>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "simple_memory_test/msg/array500kb.hpp"



class SimplePublisherNode : public rclcpp::Node {

public:

    SimplePublisherNode(std::string name) : Node(name)
    {

        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

        _pub = this->create_publisher<simple_memory_test::msg::Array500kb>( "chatter",
            custom_qos_profile);

    }

private:

    rclcpp::Publisher<simple_memory_test::msg::Array500kb>::SharedPtr _pub;

};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimplePublisherNode> pub_node = std::make_shared<SimplePublisherNode>("this_node");

    rclcpp::spin(pub_node);

    rclcpp::shutdown();

    return 0;
}