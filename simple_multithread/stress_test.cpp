#include <memory>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class SimplePublisherNode : public rclcpp::Node {

public:

    SimplePublisherNode() : Node("publisher_node")
    {
        _publisher = this->create_publisher<std_msgs::msg::Header>("my_topic");

        auto period = 200ms;
        _timer = this->create_wall_timer(period, std::bind(&SimplePublisherNode::publish, this));
    }

    void publish()
    {
        std_msgs::msg::Header::SharedPtr message(new std_msgs::msg::Header());
        _publisher->publish(message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

class SimpleSubscriberNode : public rclcpp::Node {

public:

    SimpleSubscriberNode(std::string name) : Node(name)
    {
        _subscriber = this->create_subscription<std_msgs::msg::Header>("my_topic",
            std::bind(&SimpleSubscriberNode::simple_callback, this, std::placeholders::_1));

        msg_count = 0;
    }

    int get_count() {return msg_count;}

private:

    void simple_callback(const std_msgs::msg::Header::SharedPtr msg)
    {
        (void)msg;
        msg_count ++;
    }

    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _subscriber;
    int msg_count;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int n_subscribers = 10;
    int experiment_duration = 10;
    int wait_time_before_start = 1;

    if (argc > 1){
        n_subscribers = std::strtol(argv[1], NULL, 0);
        if (argc > 2){
            experiment_duration = std::strtol(argv[2], NULL, 0);
            if (argc > 3){
                wait_time_before_start = std::strtol(argv[3], NULL, 0);
            }
        }
    }

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    std::vector<std::shared_ptr<SimpleSubscriberNode>> subscriber_nodes;

    for (int i = 0; i < n_subscribers; i++){
        std::stringstream ss;
        ss << "node_";
        ss << i;
        std::shared_ptr<SimpleSubscriberNode> sub_node = std::make_shared<SimpleSubscriberNode>(ss.str());

        subscriber_nodes.push_back(sub_node);
        executor->add_node(sub_node);
    }

    std::shared_ptr<SimplePublisherNode> pub_node = std::make_shared<SimplePublisherNode>();
    executor->add_node(pub_node);

    std::this_thread::sleep_for(std::chrono::seconds(wait_time_before_start));

    std::cout<<"Starting stress test!"<<std::endl;
    std::thread thread([executor](){
        executor->spin();
    });

    thread.detach();

    std::this_thread::sleep_for(std::chrono::seconds(experiment_duration));

    rclcpp::shutdown();

    for (int i = 0; i < n_subscribers; i++){
        std::shared_ptr<SimpleSubscriberNode> sub_node = subscriber_nodes[i];
        std::cout<<"Node: "<< i<< " received " << sub_node->get_count() << " messages!"<<std::endl;
    }

    return 0;

}