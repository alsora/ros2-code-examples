

#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "simple_service.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimpleServiceNode>node = std::make_shared<SimpleServiceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}


