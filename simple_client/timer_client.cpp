

#include "timer_client.hpp"

using GetImageSrv = simple_interfaces::srv::GetImage;
using namespace std::chrono_literals;

TimerClientNode::TimerClientNode(std::string name) : Node(name)
{

    _client = this->create_client<GetImageSrv>("get_image");

    while (!_client->wait_for_service(100ms)){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            assert(0);
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }


    _request_timer = this->create_wall_timer(500ms, std::bind(&TimerClientNode::make_request, this));


    RCLCPP_INFO(this->get_logger(), "Client created!!");
}


void TimerClientNode::make_request()
{

    _request_sent_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

    rclcpp::Client<GetImageSrv>::SharedFuture result_future = _client->async_send_request(request,
        std::bind(&TimerClientNode::response_received_callback, this, std::placeholders::_1));


}


void TimerClientNode::response_received_callback(rclcpp::Client<GetImageSrv>::SharedFuture result_future)
{

    std::shared_ptr<GetImageSrv::Response> response = result_future.get();

    std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
    auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t - _request_sent_time ).count();

    RCLCPP_INFO(this->get_logger(), "got response in %ld microseconds", request_duration);

}