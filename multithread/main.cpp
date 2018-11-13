
#include <chrono>
#include <iostream>
#include <cinttypes>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;


class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    _sub = this->create_subscription<std_msgs::msg::String>("my_topic", std::bind(&MyNode::my_callback, this, std::placeholders::_1));
    _service = this->create_service<std_srvs::srv::SetBool>("set_bool", std::bind(&MyNode::my_service_handler, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3));
  }

  void process_something(std::string str)
  {
    std::cout<<"process_something: "<< str<<std::endl;
  }

private:

  void my_callback(std_msgs::msg::String::SharedPtr msg)
  {
    std::cout<<"my_callback: "<< msg->data<<std::endl;
  }

  void my_service_handler(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::cout<<"handling service request: "<< request->data<<std::endl;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _service;

};


int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  
  auto other_thread_node = std::make_shared<MyNode>();

  auto node = rclcpp::Node::make_shared("simple_publisher");
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher = node->create_publisher<std_msgs::msg::String>("my_topic");
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bool_client = node->create_client<std_srvs::srv::SetBool>("set_bool");

  while (!bool_client->wait_for_service(std::chrono::milliseconds(250))){
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "ros client interrupted while waiting for services to appear.") ;
          return -1;
      }

      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...") ;
  }


  // I need the static_cast because rclcpp::spin is an overloaded function so I have to disambiguate it and return a pointer
  std::thread thread1(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), other_thread_node);
  thread1.detach();

  std::cout<<"Started spinning node in another thread!!"<<std::endl;

  rclcpp::WallRate loop_rate(250ms);




  std_msgs::msg::Header h;

  h.stamp = node->now();

  int i =0;

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  for (i =0; i < 100000; i ++){
    rclcpp::Time my_time = rclcpp::Time(h.stamp);
  }
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

  std::cout<<"CASE 1 : "<< duration<<std::endl;

  t1 = std::chrono::high_resolution_clock::now();
  for (i =0; i < 100000; i ++){
    rclcpp::Time my_time = h.stamp;
  }
  t2 = std::chrono::high_resolution_clock::now();

  duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout<<"CASE 2 : "<< duration<<std::endl;

  /*
  int msg_count = 0;
  while (rclcpp::ok()){
    msg_count++;
    std_msgs::msg::String message1 = std_msgs::msg::String();
    
    message1.data = "Hello, world! " + std::to_string(msg_count);
    string_publisher->publish(message1);

    std_srvs::srv::SetBool::Request::SharedPtr request = std::make_shared<std_srvs::srv::SetBool::Request>();

    auto future = bool_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::executor::FutureReturnCode::SUCCESS){

      RCLCPP_ERROR(node->get_logger(), "service call failed :(") ;
      return -1;
    }


    other_thread_node->process_something("Hello, ROS2!");

    loop_rate.sleep();
  }
  */

  rclcpp::shutdown();
  return 0;
}

