

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "my_interfaces/srv/get_image.hpp"
#include "rclcpp/rclcpp.hpp"


//#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using GetImageSrv = my_interfaces::srv::GetImage;
using ImageMsg = sensor_msgs::msg::Image;

rclcpp::Node::SharedPtr g_node = nullptr;

ImageMsg create_image(const std::shared_ptr<GetImageSrv::Request> request);

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetImageSrv::Request> request,
  const std::shared_ptr<GetImageSrv::Response> response)
{
  (void)request_header;

  RCLCPP_INFO(g_node->get_logger(),"request received");
  
  response->image = create_image(request);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_service");

  auto server = g_node->create_service<GetImageSrv>("get_image", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}




ImageMsg create_image(const std::shared_ptr<GetImageSrv::Request> request)
{

  ImageMsg img;

  // Generate fake frame
  img.width = request->res_w;
  img.height = request->res_h;
  img.data.resize(img.width * img.height);
  for (int i = 0; i < img.width * img.height; i++){
    img.data[i] = ( std::rand() % ( 255 + 1 ) );
  }


  return img;
}
