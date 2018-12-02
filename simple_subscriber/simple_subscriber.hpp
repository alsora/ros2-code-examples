
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SimpleSubscriberNode : public rclcpp::Node {

public:

    SimpleSubscriberNode(std::string name = "simple_subscriber");

private:

    void simple_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
};