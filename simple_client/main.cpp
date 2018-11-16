
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

/**
 * The goal of this class is to implement a node which makes client requests through a timer callback.
 * Unfortunately it's not easy, because the timer callback is triggered while the node is spinning, 
 * thus it's not possible to add the node again to an executor and wait for the server response.
 * ---> use the provided main, without a class 
 */

#if (0)
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
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.")
        assert(0);
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...")
    }


    _request_timer = this->create_wall_timer(500ms, std::bind(&MyNode::make_request, this));


  }

private:

  void make_request()
  {

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

    rclcpp::Client<GetImageSrv>::SharedFuture result_future = _client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "service call failed :(")
      assert(0);
    }

    std::shared_ptr<GetImageSrv::Response> response = result_future.get();

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();


    RCLCPP_INFO(this->get_logger(), "got response in %ld", request_duration)

    

  }

  rclcpp::Client<GetImageSrv>::SharedPtr _client;
  rclcpp::TimerBase::SharedPtr _request_timer;


};
#endif



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("simple_client");
  rclcpp::Client<GetImageSrv>::SharedPtr client = node->create_client<GetImageSrv>("get_image");

  while (!client->wait_for_service(500ms))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.")
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...")
  }


  rclcpp::WallRate loop_rate(10);
  while (rclcpp::ok())
  {

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    
    std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

    rclcpp::Client<GetImageSrv>::SharedFuture result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(")
      return 1;
    }


    std::shared_ptr<GetImageSrv::Response> response = result_future.get();

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();


    RCLCPP_INFO(node->get_logger(), "got response in %ld", request_duration)


    loop_rate.sleep();
  }

  return 0;
}

