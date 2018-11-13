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


void* spin_task(void* ptr) 
{

    MultiNode* node_raw = static_cast<MultiNode*>(ptr);
    std::shared_ptr<MultiNode> node(node_raw);
    
    rclcpp::spin(node);

    return 0;
}

void* client_task(void* ptr)
{

    MultiNode* node_raw = static_cast<MultiNode*>(ptr);
    std::shared_ptr<MultiNode> node(node_raw);

    node->simple_client_task();

    return 0;
}



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

    pthread_t threads[n_clients + n_services];
    pthread_attr_t tattr;
    sched_param param;

    pthread_attr_init (&tattr);
    pthread_attr_getschedparam (&tattr, &param);

    std::vector<std::shared_ptr<MultiNode>> vec;

    for (int i = 0; i < n_clients; i ++){

        param.sched_priority = 10 + i;
        pthread_attr_setschedparam (&tattr, &param);
        
        int node_id = i;
        auto node = std::make_shared<MultiNode>(node_id);

        for (int k = 0; k < n_services; k ++){
            
            int client_id = k + n_clients;
            node->add_client(client_id);
        }

        vec.push_back(node);

        pthread_create(&threads[i], &tattr, &client_task, static_cast<void *>(node.get()));

    }

    std::cout<<"Clients created!"<<std::endl;
        
    for (int i = 0; i < n_services; i++){

        param.sched_priority = 15;
        pthread_attr_setschedparam (&tattr, &param);

        int node_id = i + n_clients;
        auto node = std::make_shared<MultiNode>(node_id);

        int service_id = node_id;
        node->add_service(service_id);

        vec.push_back(node);

        pthread_create(&threads[i], &tattr, &spin_task, static_cast<void *>(node.get()));

    }

    std::cout<<"Services created!"<<std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(test_duration));

    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;
    /*
    for (auto thread : threads){
        char *b;;
        pthread_join(thread, (void**)&b);
    }
    */

    std::cout<<"N_CLIENTS: "<<n_clients<<std::endl;

    for (int i = 0; i < n_clients; i ++){
        
        std::string name = vec[i]->get_name();

        std::cout<<name<<" --> "<<std::endl;


        auto durations_map = vec[i]->stats.requests_durations_map;

        std::cout<<name<<" --> "<<std::endl;
        for (auto const& map_item : durations_map){
            std::cout<<"ITEM --> "<<std::endl;

            int client_id = map_item.first;
            std::cout<<client_id<<" --> "<<std::endl;

            auto duration = map_item.second.first;
            int num_requests = map_item.second.second;

            std::cout<<"client "<< client_id << " --> "<< duration/num_requests << "  #"<< num_requests<<std::endl;
        }

    }
    
 
    std::cout<<"End test"<<std::endl;

    return 0;
}

