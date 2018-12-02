
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;



class SimpleSubscriberNode : public rclcpp::Node {

public:
    SimpleSubscriberNode(std::string name = "simple_subscriber") : Node(name)
    {
        _subscriber = this->create_subscription<std_msgs::msg::String>("my_topic",
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscriber created!!");
    }

private:
    void simple_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received msg from '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimpleSubscriberNode> simple_subscriber = std::make_shared<SimpleSubscriberNode>("simple_subscriber");
    std::shared_ptr<SimpleSubscriberNode> additional_subscriber = std::make_shared<SimpleSubscriberNode>("additional_subscriber");

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(simple_subscriber);
    executor.add_node(additional_subscriber);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}


