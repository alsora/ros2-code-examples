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

    int n_nodes = 2;
    int test_duration = 5;

    if (argc >= 2){
        n_nodes = atoi(argv[1]);

        if (argc >= 3){
            test_duration = atoi(argv[2]);
        }
    }

    std::vector<std::shared_ptr<MultiNode>> vec;

    for (int i = 0; i < n_nodes; i ++){

        int node_id = i;
        auto node = std::make_shared<MultiNode>(node_id);

        vec.push_back(node);

        std::thread thread1(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), node);
        thread1.detach();

    }

    std::cout<<"All nodes created"<<std::endl;


    std::this_thread::sleep_for(std::chrono::seconds(test_duration));

    rclcpp::shutdown();
    
    std::cout<<"End test"<<std::endl;

    return 0;
}

