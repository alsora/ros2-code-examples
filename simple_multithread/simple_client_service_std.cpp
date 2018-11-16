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

#include <pthread.h>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "multi-node.hpp"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int n_clients = 2;
    int n_services = 1;
    int test_duration = 5;

    if (argc >= 2){
        n_clients = atoi(argv[1]);

        if (argc >= 3){
            n_services = atoi(argv[2]);

            if (argc >= 4){
                test_duration = atoi(argv[3]);
            }
        }
    }

    std::vector<std::shared_ptr<MultiNode>> vec;

    for (int i = 0; i < n_clients; i ++){

        int node_id = i;
        auto node = std::make_shared<MultiNode>(node_id);

        for (int k = 0; k < n_services; k ++){
            
            int client_id = k + n_clients;
            node->add_client(client_id);
        }

        vec.push_back(node);

        std::thread thread1(&MultiNode::simple_client_task, node);
        thread1.detach();

    }

    std::cout<<"Clients created!"<<std::endl;
        
    for (int i = 0; i < n_services; i++){

        int node_id = i + n_clients;
        auto node = std::make_shared<MultiNode>(node_id);

        int service_id = node_id;
        node->add_service(service_id);

        vec.push_back(node);

        std::thread thread1(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), node);
        thread1.detach();
    }

    std::cout<<"Services created!"<<std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(test_duration));

    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;

    for (int i = 0; i < n_clients; i ++){
        
        std::string name = vec[i]->get_name();

        auto durations_map = vec[i]->stats.client_requests_latency;

        for (auto const& map_item : durations_map){

            int client_id = map_item.first;

            auto duration = map_item.second.first;
            int num_requests = map_item.second.second;

            std::cout<<name << ": client "<< client_id << " --> "<< duration/num_requests << "  #"<< num_requests<<std::endl;
        }

    }
    
 
    std::cout<<"End test"<<std::endl;

    return 0;
}

