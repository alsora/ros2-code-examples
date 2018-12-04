#include "simple_client_node.hpp"

using GetImageSrv = simple_interfaces::srv::GetImage;
using namespace std::chrono_literals;

SimpleClientNode::SimpleClientNode(std::string name) : Node(name)
{

    _client = this->create_client<GetImageSrv>("get_image");

    while (!_client->wait_for_service(100ms)){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            assert(0);
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    RCLCPP_INFO(this->get_logger(), "Client created!!");
}


void SimpleClientNode::run_request_loop()
{

    rclcpp::WallRate loop_rate(250ms);
    while(rclcpp::ok()){

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

        rclcpp::Client<GetImageSrv>::SharedFuture result_future = _client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "service call failed :(");
            assert(0);
        }

        std::shared_ptr<GetImageSrv::Response> response = result_future.get();

        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

        RCLCPP_INFO(this->get_logger(), "got response in %ld microseconds", request_duration);

        loop_rate.sleep();
    }

}

