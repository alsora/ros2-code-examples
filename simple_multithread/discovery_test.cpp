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
        msg_count ++;
    }

    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _subscriber;
    int msg_count;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int n_subscribers = 20;

    if (argc > 1){
        if (argc > 2){
            assert(0 && "Too many arguments");
        }
        n_subscribers = std::strtol(argv[1], NULL, 0);
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

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    std::cout<<"Starting stress test!"<<std::endl;
    std::thread thread([executor](){
        executor->spin();
    });

    thread.detach();


    int discovered = 0;
    auto query_inteval = std::chrono::milliseconds(10);
    while (true){
        std::this_thread::sleep_for(query_inteval);

        int d = pub_node->count_subscribers("my_topic");

        if (d > discovered){
            discovered = d;
            std::chrono::high_resolution_clock::time_point t_best = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t_best - t1 ).count();
            std::cout<<"Discovered --> "<<discovered <<"/"<<n_subscribers<<" after "<< duration << " milliseconds"<<std::endl;
        }

        if (discovered == n_subscribers){
            break;
        }

    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout<<"DISCOVERY TIME ----> "<< duration<<" milliseconds" <<std::endl;

    rclcpp::shutdown();

    for (int i = 0; i < n_subscribers; i++){
        std::shared_ptr<SimpleSubscriberNode> sub_node = subscriber_nodes[i];
        std::cout<<"Node: "<< i<< " received " << sub_node->get_count() << " messages!"<<std::endl;
    }

    return 0;

}
