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
    
    std::this_thread::sleep_for(std::chrono::seconds(test_duration));

    rclcpp::shutdown();

    std::cout<<"rclcpp::shutdown"<<std::endl;

    std::map<std::string, int> g_counters;
    for (int i = 0; i < n_subscribers; i ++){

        std::string name = vec[i]->get_name();
        g_counters[name] = vec[i]->stats.all_msgs_counter;
    }


    // Declaring the type of Predicate that accepts 2 pairs and return a bool
	typedef std::function<bool(std::pair<std::string, int>, std::pair<std::string, int>)> Comparator;
 
	// Defining a lambda function to compare two pairs. It will compare two pairs using second field
	Comparator compFunctor =
			[](std::pair<std::string, int> elem1 ,std::pair<std::string, int> elem2)
			{
				return elem1.second > elem2.second;
			};


	// Declaring a set that will store the pairs using above comparision logic
	std::vector<std::pair<std::string, int>> orderedSet(
			g_counters.begin(), g_counters.end());

    std::sort(orderedSet.begin(), orderedSet.end(), compFunctor);
    
    for (std::pair<std::string, int> element : orderedSet)
		std::cout << element.first << " --> " << element.second << std::endl;

    
    std::cout<<"End test"<<std::endl;

    return 0;
}

