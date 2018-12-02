
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;



class SimplePublisherNode : public rclcpp::Node {

public:

    SimplePublisherNode(std::string name = "simple_publisher") : Node(name)
    {
        _publisher = this->create_publisher<std_msgs::msg::String>("my_topic");

        auto period = 500ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));

        RCLCPP_INFO(this->get_logger(), "Publisher created!!");
    }

    void publish()
    {
        auto message = std_msgs::msg::String();
        message.data = this->get_name();
        _publisher->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing message!");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<SimplePublisherNode> simple_publisher;
    std::shared_ptr<SimplePublisherNode> other_node;

    try {
        simple_publisher = std::make_shared<SimplePublisherNode>("simple_publisher");
        executor.add_node(simple_publisher);
    }
    catch (const std::exception& e){
        std::cerr << e.what() << std::endl;
    }

    try {
        other_node = std::make_shared<SimplePublisherNode>("other_node");
        executor.add_node(other_node);
    }
    catch (const std::exception& e){
       std::cerr << e.what() << std::endl;
    }

    if (simple_publisher == nullptr && other_node == nullptr){
        rclcpp::shutdown();
        return 0;
    }

    executor.spin();

    rclcpp::shutdown();

    return 0;
}


