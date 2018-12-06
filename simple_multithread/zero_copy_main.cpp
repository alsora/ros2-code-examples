#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/msg/stamped_array4m.hpp"

using namespace std::chrono_literals;

class SimplePublisherNode : public rclcpp::Node {

public:

    SimplePublisherNode(std::string name = "simple_publisher", bool use_intra_process_comm = true) : Node(name, "", use_intra_process_comm)
    {
        _publisher = this->create_publisher<simple_interfaces::msg::StampedArray4m>("my_topic");

        auto period = 500ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));

        RCLCPP_INFO(this->get_logger(), "Publisher created!!");
    }

    void publish()
    {
        simple_interfaces::msg::StampedArray4m::UniquePtr message(new simple_interfaces::msg::StampedArray4m());
        message->header.stamp = this->now();
        RCLCPP_INFO(this->get_logger(), "Publishing message with address 0x%" PRIXPTR, reinterpret_cast<std::uintptr_t>(message.get()));
        _publisher->publish(message);
    }

private:

    rclcpp::Publisher<simple_interfaces::msg::StampedArray4m>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;

};

class SimpleSubscriberNode : public rclcpp::Node {

public:

    SimpleSubscriberNode(std::string name = "simple_subscriber", bool use_intra_process_comm = true) : Node(name, "", use_intra_process_comm)
    {
        _subscriber = this->create_subscription<simple_interfaces::msg::StampedArray4m>("my_topic",
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscriber created!!");
    }

private:

    void simple_callback(const simple_interfaces::msg::StampedArray4m::UniquePtr msg)
    {

        rclcpp::Duration rcl_duration = this->now() - rclcpp::Time(msg->header.stamp);
        int64_t delta_microseconds = RCL_NS_TO_US(static_cast<int64_t>(rcl_duration.nanoseconds()));

        RCLCPP_INFO(this->get_logger(), "Received message with address 0x%" PRIXPTR " took %" PRId64 " microsec", reinterpret_cast<std::uintptr_t>(msg.get()), delta_microseconds);

    }

    rclcpp::Subscription<simple_interfaces::msg::StampedArray4m>::SharedPtr _subscriber;
};


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    bool use_intra_process_comm = true;

    auto pub = std::make_shared<SimplePublisherNode>("publisher", use_intra_process_comm);
    auto sub = std::make_shared<SimpleSubscriberNode>("subscriber", use_intra_process_comm);

    executor.add_node(pub);
    executor.add_node(sub);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}