
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

  std::shared_ptr<GetImageSrv::Request> request = std::make_shared<GetImageSrv::Request>();

  while (rclcpp::ok())
  {

    rclcpp::Client<GetImageSrv>::SharedFuture result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(")
      return 1;
    }


    std::shared_ptr<GetImageSrv::Response> response = result_future.get();

    RCLCPP_INFO(node->get_logger(), "got response")
    ImageMsg img  = response->image;

    response_msg_publisher->publish(img);

    rclcpp::spin_some(node);

    loop_rate.sleep();
  }


  rclcpp::shutdown();
  return 0;
}

