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
#include "std_msgs/msg/string.hpp"

#include "multi-node.hpp"



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

    for (int i = 0; i < n_subscribers; i ++){

        int node_id = i;
        auto node = std::make_shared<MultiNode>(node_id);

        for (int k = 0; k < n_publishers; k ++){
            
            int subscriber_id = k + n_subscribers;
            node->add_subscriber(subscriber_id);
        }

        vec.push_back(node);

        std::thread thread1(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), node);
        thread1.detach();

    }

    std::cout<<"Subscribers created!"<<std::endl;
    
    for (int i = 0; i < n_publishers; i++){

        int node_id = i + n_subscribers;
        auto node = std::make_shared<MultiNode>(node_id);

        int publisher_id = node_id;
        node->add_publisher(publisher_id);

        vec.push_back(node);
        std::thread thread1(&MultiNode::simple_publisher_task, node);
        thread1.detach();
    }

    std::cout<<"Publishers created!"<<std::endl;
    
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

