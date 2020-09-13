#include <memory>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "simple_latency_test/msg/dynamic.hpp"
#include "simple_latency_test/sub.hpp"

using namespace std::chrono_literals;

using MsgType = simple_latency_test::msg::Dynamic;

class SimplePublisherNode : public rclcpp::Node
{
public:
    SimplePublisherNode(size_t size) : Node("publisher_node")
    {
        _publisher = this->create_publisher<MsgType>("my_topic", 10);

        auto period = 500ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));

        msg_count = 0;
        _size = size;
    }

    void publish()
    {
        RCLCPP_DEBUG(this->get_logger(), "Publish new message %d", msg_count);

        auto message = std::make_unique<MsgType>();

        //message->data.resize(_size);
        message->data = std::vector<uint8_t>(_size, 1);

        message->header.stamp = this->now();

        _publisher->publish(std::move(message));

        msg_count++;
    }

private:
    rclcpp::Publisher<MsgType>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    int msg_count;
    size_t _size;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    size_t size = 250000;

    if (argc > 1){
        size = std::strtol(argv[1], NULL, 0);
    }

    std::cout<<"Publishing messages of size "<< size << " byte"<<std::endl;

    auto sub_node = std::make_shared<SimpleSubscriberNode<MsgType>>();
    auto pub_node = std::make_shared<SimplePublisherNode>(size);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::thread t_s([=]() { rclcpp::spin(sub_node); });
    t_s.detach();

    std::thread t_p([=]() { rclcpp::spin(pub_node); });
    t_p.detach();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    rclcpp::shutdown();

    std::cout<<"Mean latency: "<< sub_node->mean<<std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
