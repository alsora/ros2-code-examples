#include "rclcpp/rclcpp.hpp"

#include "simple_service/simple_service_node.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SimpleServiceNode>node = std::make_shared<SimpleServiceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}


