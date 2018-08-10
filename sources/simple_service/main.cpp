

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "irobot_interfaces/srv/retrieve_frame.hpp"
#include "irobot_interfaces/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"


//#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using RetrieveFrame = my_interfaces::srv::RetrieveFrame;
using FrameMsg = my_interfaces::msg::Frame;

rclcpp::Node::SharedPtr g_node = nullptr;

FrameMsg retrieveFrame(const std::shared_ptr<RetrieveFrame::Request> request);

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RetrieveFrame::Request> request,
  const std::shared_ptr<RetrieveFrame::Response> response)
{
  (void)request_header;

  RCLCPP_INFO(g_node->get_logger(),"request");
  
  response->frame = retrieveFrame(request);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_service");

  auto server = g_node->create_service<RetrieveFrame>("retrieve_frame", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}




FrameMsg retrieveFrame(const std::shared_ptr<RetrieveFrame::Request> request)
{

  FrameMsg fMsg;

  // Generate fake frame
  f.width = request->res_w;
  f.height = request->res_h;
  f.data.resize(f.width * f.height);
  for (int i = 0; i < f.width * f.height; i++){
    f.data[i] = ( std::rand() % ( 255 + 1 ) );
  }


  return fMsg;
}
