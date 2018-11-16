#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <sstream>
#include <map>
#include <set>
#include <algorithm>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "multi-node.hpp"

using namespace std::chrono_literals;


void publish_callback(std::shared_ptr<MultiNode> node)
{
    std_msgs::msg::Header message = std_msgs::msg::Header();

    message.stamp = node->now();
    message.frame_id = node->_name;

    for (auto const& map_item : node->_publishers){

      rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher = map_item.second;
      publisher->publish(message);
    
    }
}




void executor_spin(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
{

    executor->spin();

}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int n_subscribers = 2;
    int n_publishers = 1;
    int test_duration = 5;

    if (argc >= 2){
        n_subscribers = atoi(argv[1]);

        if (argc >= 3){
            n_publishers = atoi(argv[2]);

            if (argc >= 4){
                test_duration = atoi(argv[3]);
            }
        }
    }

    std::vector<std::shared_ptr<MultiNode>> vec;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    for (int i = 0; i < n_subscribers; i ++){

        int node_id = i;
        auto node = std::make_shared<MultiNode>(node_id);

        for (int k = 0; k < n_publishers; k ++){
            
            int subscriber_id = k + n_subscribers;
            node->add_subscriber(subscriber_id);
        }

        vec.push_back(node);
        executor->add_node(node);

    }

    std::cout<<"Subscribers created!"<<std::endl;
    
    auto publish_interval = 250ms;
    for (int i = 0; i < n_publishers; i++){

        int node_id = i + n_subscribers;
        auto node = std::make_shared<MultiNode>(node_id);

        int publisher_id = node_id;
        node->add_publisher(publisher_id);
        
        node->add_timer(publish_interval, std::bind(&publish_callback, node));

        vec.push_back(node);

        executor->add_node(node);

    }

    std::cout<<"Publishers created!"<<std::endl;
    
    // temporary, until api spin for seconds comes out (15 december 2018 with crystal release)
    std::thread thread1(executor_spin, executor);
    thread1.detach();
    std::this_thread::sleep_for(std::chrono::seconds(test_duration));

    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;

    for (int i = 0; i < n_subscribers; i ++){
        
        std::string name = vec[i]->get_name();

        auto durations_map = vec[i]->stats.subscription_delta_times;

        for (auto const& map_item : durations_map){

            int topic_id = map_item.first;

            auto duration = map_item.second.first;
            int num_msgs = map_item.second.second;

            float mean = 0;
            if (num_msgs != 0)
                mean = (float)duration/(float)num_msgs;

            std::cout<<name << ": topic "<< topic_id << " --> "<< mean << "  #"<< num_msgs<<std::endl;
        }

    }

    
    std::cout<<"End test"<<std::endl;

    return 0;
}

