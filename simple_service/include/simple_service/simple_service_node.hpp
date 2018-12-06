#ifndef __SIMPLE_SERVICE_NODE_HPP__
#define __SIMPLE_SERVICE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/srv/get_image.hpp"


class SimpleServiceNode : public rclcpp::Node
{

public:

    SimpleServiceNode(std::string name = "simple_service");

private:

    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<simple_interfaces::srv::GetImage::Request> request,
        const std::shared_ptr<simple_interfaces::srv::GetImage::Response> response);

    rclcpp::Service<simple_interfaces::srv::GetImage>::SharedPtr _service;

};


#endif //__SIMPLE_SERVICE_NODE_HPP__