
#include <chrono>
#include <iostream>
#include <cinttypes>
#include <memory>
#include "my_interfaces/srv/get_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

using GetImageSrv = my_interfaces::srv::GetImage;
using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono_literals;


class MyNode : public rclcpp::Node
{
public:

  MyNode() : Node("simple_client")
  {

    _client = this->create_client<GetImageSrv>("get_image");

    while (!_client->wait_for_service(100ms))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        assert(0);
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    _request_timer = this->create_wall_timer(500ms, std::bind(&MyNode::make_request, this));

    RCLCPP_INFO(this->get_logger(), "Client created!!");
  }

private:

  void response_received_callback(rclcpp::Client<GetImageSrv>::SharedFuture result_future)
  {

    std::shared_ptr<GetImageSrv::Response> response = result_future.get();
  
    std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
    auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t - _request_sent_time ).count();
    RCLCPP_INFO(this->get_logger(), "got response in %ld", request_duration);

  }


  void make_request()
  {

    _request_sent_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

    rclcpp::Client<GetImageSrv>::SharedFuture result_future = _client->async_send_request(request, std::bind(&MyNode::response_received_callback, this, std::placeholders::_1));
  }

  rclcpp::Client<GetImageSrv>::SharedPtr _client;
  rclcpp::Subscription<GetImageSrv>::SharedPtr _futures_subscriber;
  rclcpp::TimerBase::SharedPtr _request_timer;

  std::chrono::high_resolution_clock::time_point _request_sent_time;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<MyNode> node = std::make_shared<MyNode>();
  rclcpp::spin(node);

  return 0;

}
