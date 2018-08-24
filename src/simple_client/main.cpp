
#include <chrono>
#include <iostream>
#include <cinttypes>
#include <memory>
#include "my_interfaces/srv/retrieve_frame.hpp"
#include "my_interfaces/msg/frame.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

using RetrieveFrameSrv = my_interfaces::srv::RetrieveFrame;
using Frame = my_interfaces::msg::Frame;
using Image = sensor_msgs::msg::Image;
using namespace std::chrono_literals;


Image frame2image(Frame f);


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("simple_client");
  rclcpp::Client<RetrieveFrameSrv>::SharedPtr client = node->create_client<RetrieveFrameSrv>("retrieve_frame");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  rclcpp::Publisher<Image>::SharedPtr response_msg_publisher = node->create_publisher<Image>("image", custom_qos_profile);

  std::chrono::milliseconds milis(200);

  rclcpp::WallRate loop_rate(500ms);

  while (!client->wait_for_service(500ms))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.")
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...")
  }

  std::shared_ptr<RetrieveFrameSrv::Request> request = std::make_shared<RetrieveFrameSrv::Request>();

  while (rclcpp::ok())
  {

    rclcpp::Client<RetrieveFrameSrv>::SharedFuture result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(")
      return 1;
    }

    std::shared_ptr<RetrieveFrameSrv::Response> response = result_future.get();

    RCLCPP_INFO(node->get_logger(), "got response")
    Image img = frame2image(response->frame);
    response_msg_publisher->publish(img);

    rclcpp::spin_some(node);

    loop_rate.sleep();
  }


  rclcpp::shutdown();
  return 0;
}



Image frame2image(Frame f)
{

  Image img;

  img.encoding = "mono8";

  img.height = f.height;
  img.width = f.width;

  img.data = f.data;


  return img;

}
