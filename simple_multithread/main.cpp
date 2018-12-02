#include <chrono>
#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::shutdown();

    return 0;

}