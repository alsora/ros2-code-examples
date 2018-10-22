
#include <chrono>
#include <iostream>
#include <cinttypes>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>

using namespace std::chrono_literals;


class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    
    _sub = this->create_subscription<std_msgs::msg::String>("my_topic", std::bind(&MyNode::my_callback, this, std::placeholders::_1));
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


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;


};


void spin_node(rclcpp::Node::SharedPtr node)
{
  rclcpp::spin(node);
}



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MyNode>();

  auto node_pub = rclcpp::Node::make_shared("simple_publisher");
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher = node_pub->create_publisher<std_msgs::msg::String>("my_topic");


  std::thread t1(spin_node, node);

  std::cout<<"Started spinning node in another thread!!"<<std::endl;

  rclcpp::WallRate loop_rate(250ms);
  int msg_count = 0;
  while (rclcpp::ok()){
    msg_count++;
    std_msgs::msg::String message1 = std_msgs::msg::String();
    message1.data = "Hello, world! " + std::to_string(msg_count);
    string_publisher->publish(message1);

    node->process_something("Hello, ROS2!");

    loop_rate.sleep();
  }


  rclcpp::shutdown();
  return 0;
}

