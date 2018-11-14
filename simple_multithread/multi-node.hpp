#ifndef __MULTI_NODE_HPP__
#define __MULTI_NODE_HPP__

#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/empty.hpp"


typedef std::chrono::duration<int64_t, std::ratio<1, 1000000> >::rep duration_microseconds;

struct Stats{

    std::map<int, std::pair<duration_microseconds, int> > requests_durations_map;
    std::map<int, std::pair<duration_microseconds, int> > subscriptions_durations_map;

    int all_msgs_counter;

};

class MultiNode : public rclcpp::Node {

public:

    MultiNode(int id);
    ~MultiNode();

    void add_publisher(int id);
    void add_subscriber(int id);
    void add_service(int id);
    void add_client(int id);

    void simple_publisher_task();
    void simple_client_task();


    Stats stats;

private:

    void service_handler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void topic_callback(const std_msgs::msg::Header::SharedPtr msg, int id);

    int _id;
    std::string _name;

    std::map<int, rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr> _subscribers;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr> _publishers;

    std::map<int, rclcpp::Client<std_srvs::srv::Empty>::SharedPtr> _clients;
    std::map<int, rclcpp::Service<std_srvs::srv::Empty>::SharedPtr> _services;


};


#endif