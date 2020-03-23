#include <chrono>
#include <memory>
#include <iostream>
#include <thread>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


void default_usage_test()
{
    std::cout<<"DEFAULT USAGE TEST: start!"<<std::endl;

    auto pub_node = std::make_shared<rclcpp::Node>("pub_node");
    auto pub = pub_node->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [&]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(pub_node->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub->publish(message);
      };
    auto timer = pub_node->create_wall_timer(500ms, timer_callback);

    size_t count = 0;
    auto sub_node = std::make_shared<rclcpp::Node>("sub_node");
    auto sub = sub_node->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [&](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(sub_node->get_logger(), "I heard: '%s'", msg->data.c_str());
        count++;
      });

    // Create a Static Executor
    auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    // Add both node to the executor
    executor->add_node(pub_node);
    executor->add_node(sub_node);

    std::thread executor_thread([=]() { executor->spin(); });
    executor_thread.detach();

    std::this_thread::sleep_for(2s);

    executor->remove_node(pub_node);
    executor->remove_node(sub_node);

    assert(count > 0 && "No messages received!!");

    std::cout<<"DEFAULT USAGE TEST: success!"<<std::endl;
    
    std::this_thread::sleep_for(250ms);
}

void node_added_test()
{
    std::cout<<"NODE ADDED TEST: start!"<<std::endl;

    auto pub_node = std::make_shared<rclcpp::Node>("pub_node");
    auto pub = pub_node->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [&]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(pub_node->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub->publish(message);
      };
    auto timer = pub_node->create_wall_timer(500ms, timer_callback);

    size_t count = 0;
    auto sub_node = std::make_shared<rclcpp::Node>("sub_node");
    auto sub = sub_node->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [&](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(sub_node->get_logger(), "I heard: '%s'", msg->data.c_str());
        count++;
      });

    // Create a Static Executor
    auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    // Add only 1 node to the executor
    executor->add_node(pub_node);

    std::thread executor_thread([=]() { executor->spin(); });
    executor_thread.detach();

    std::this_thread::sleep_for(1s);

    // Add the other node to the executor
    executor->add_node(sub_node);

    std::this_thread::sleep_for(2s);

    executor->remove_node(pub_node);
    executor->remove_node(sub_node);

    assert(count > 0 && "No messages received!!");

    std::cout<<"NODE ADDED TEST: success!"<<std::endl;
    
    std::this_thread::sleep_for(250ms);
}

void sub_added_test()
{
    std::cout<<"SUBSCRIPTION ADDED TEST: start!"<<std::endl;

    auto pub_node = std::make_shared<rclcpp::Node>("pub_node_3");
    auto pub = pub_node->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [&]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(pub_node->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub->publish(message);
      };
    auto timer = pub_node->create_wall_timer(500ms, timer_callback);

    size_t count = 0;
    auto sub_node = std::make_shared<rclcpp::Node>("sub_node_3");

    // Create a Static Executor for each node
    auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    executor->add_node(pub_node);
    executor->add_node(sub_node);

    // Spin each executor in a different thread
    std::thread executor_thread([=]() { executor->spin(); });
    executor_thread.detach();

    std::this_thread::sleep_for(1s);

    auto sub = sub_node->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [&](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(sub_node->get_logger(), "I heard: '%s'", msg->data.c_str());
        count++;
      });

    std::this_thread::sleep_for(2s);

    executor->remove_node(pub_node);
    executor->remove_node(sub_node);

    assert(count > 0 && "No messages received!!");

    std::cout<<"SUBSCRIPTION ADDED TEST: success!"<<std::endl;
    
    std::this_thread::sleep_for(250ms);
}

void node_removed_test()
{
    std::cout<<"NODE REMOVED TEST: start!"<<std::endl;

    auto pub_node = std::make_shared<rclcpp::Node>("pub_node_3");
    auto pub = pub_node->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [&]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(pub_node->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub->publish(message);
      };
    auto timer = pub_node->create_wall_timer(500ms, timer_callback);

    size_t count = 0;
    auto sub_node = std::make_shared<rclcpp::Node>("sub_node_3");
    auto sub = sub_node->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [&](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(sub_node->get_logger(), "I heard: '%s'", msg->data.c_str());
        count++;
      });

    // Create a Static Executor for each node
    auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    executor->add_node(pub_node);
    executor->add_node(sub_node);

    // Spin each executor in a different thread
    std::thread executor_thread([=]() { executor->spin(); });
    executor_thread.detach();

    std::this_thread::sleep_for(2s);
    assert(count > 0 && "No messages received!!");
    executor->remove_node(sub_node);
    
    // Wait a little bit to make sure everything that was pending is processed
    std::this_thread::sleep_for(100ms);
    count = 0;

    std::this_thread::sleep_for(2s);

    executor->remove_node(pub_node);

    assert(count == 0 && "Messages received even if node was removed");

    std::cout<<"NODE REMOVED TEST: success!"<<std::endl;
    
    std::this_thread::sleep_for(250ms);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int test_id = 0;

    if (argc == 2) {
        test_id = atoi(argv[1]);
    }

    std::cout<<"Selected test case "<< test_id<<std::endl;

    switch(test_id) {
    case 0:
        default_usage_test();
        break;
    case 1:
        node_added_test();
        break;
    case 2:
        sub_added_test();
        break;
    case 3:
        node_removed_test();
        break;
    default:
        default_usage_test();
        break;
    }

    rclcpp::shutdown();
    
    std::this_thread::sleep_for(250ms);

    return 0;
}
