
#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/srv/get_image.hpp"



class SimpleClientNode : public rclcpp::Node
{

public:

    SimpleClientNode(std::string name = "simple_client");

    void run_request_loop();

private:

    rclcpp::Client<simple_interfaces::srv::GetImage>::SharedPtr _client;

};