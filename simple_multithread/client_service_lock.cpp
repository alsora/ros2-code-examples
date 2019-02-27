#include <memory>
#include <iostream>
#include <thread>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


class SimpleServerNode : public rclcpp::Node {

public:

    SimpleServerNode() : Node("publisher_node")
    {
        _service = this->create_service<std_srvs::srv::Empty>("test_service",
            std::bind(&SimpleServerNode::handle_service, this, _1, _2, _3));


        RCLCPP_INFO(this->get_logger(), "Service created!!");
    }

    void handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        (void)request_header;
        (void)request;
        (void)response;
    }

private:

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _service;

};

class SimpleClientNode : public rclcpp::Node {

public:

    SimpleClientNode(std::string name) : Node(name)
    {

        _client = this->create_client<std_srvs::srv::Empty>("test_service");

        RCLCPP_INFO(this->get_logger(), "Client created!!");

        sent_count = 0;
        received_count = 0;
        found = false;
    }

    void get_count(std::chrono::time_point<std::chrono::system_clock> start_time)
    {

        auto last_sent_duration = std::chrono::duration_cast<std::chrono::milliseconds>( last_sent - start_time ).count();
        auto last_received_duration = std::chrono::duration_cast<std::chrono::milliseconds>( last_received - start_time ).count();

        RCLCPP_INFO(this->get_logger(), "Sent %d", sent_count);
        RCLCPP_INFO(this->get_logger(), "Received %d", received_count);
        RCLCPP_INFO(this->get_logger(), "Last sent %lu", last_sent_duration);
        RCLCPP_INFO(this->get_logger(), "Last received %lu", last_received_duration);

    }

    bool get_found()
    {
        return found;
    }

    void wait()
    {

        while (!_client->wait_for_service(100ms)){
            if (!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
                assert(0);
            }
            //RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }


        RCLCPP_INFO(this->get_logger(), "Service found!");
        found = true;

    }


    void run_request_loop(int request_frequency)
    {

        int period_ms = (1000/request_frequency);
        auto period = std::chrono::milliseconds(period_ms);

        rclcpp::WallRate loop_rate(period);
        while(rclcpp::ok()){

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::shared_ptr<std_srvs::srv::Empty::Request> request = std::make_shared<std_srvs::srv::Empty::Request>();

            sent_count ++;

            last_sent = std::chrono::system_clock::now();
            last_received = last_sent;


            rclcpp::Client<std_srvs::srv::Empty>::SharedFuture result_future = _client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
                rclcpp::executor::FutureReturnCode::SUCCESS){
                //RCLCPP_ERROR(this->get_logger(), "Service call failed :(");
            }

            if (rclcpp::ok()){
                received_count ++;
                last_received = std::chrono::system_clock::now();
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

            //RCLCPP_INFO(this->get_logger(), "Got response in %ld microseconds", request_duration);

            loop_rate.sleep();
        }

    }

private:

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _client;

    std::chrono::time_point<std::chrono::system_clock> last_sent;
    std::chrono::time_point<std::chrono::system_clock> last_received;

    bool found;
    int sent_count;
    int received_count;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int n_clients = 20;
    int request_frequency = 1000;

    if (argc > 1){
        n_clients = std::strtol(argv[1], NULL, 0);
        if (argc > 2){
            request_frequency = std::strtol(argv[2], NULL, 0);
        }
    }


    std::vector<std::shared_ptr<SimpleClientNode>> client_nodes;

    for (int i = 0; i < n_clients; i++){
        std::stringstream ss;
        ss << "node_";
        ss << i;
        std::shared_ptr<SimpleClientNode> node = std::make_shared<SimpleClientNode>(ss.str());

        client_nodes.push_back(node);

        std::thread thread = std::thread(&SimpleClientNode::wait, node);
        thread.detach();

    }

    std::shared_ptr<SimpleServerNode> node = std::make_shared<SimpleServerNode>();
    std::thread thread = std::thread(static_cast<void (*)(rclcpp::Node::SharedPtr)>(rclcpp::spin), node);
    thread.detach();


    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout<<"Starting stress test!"<<std::endl;

    auto start_time = std::chrono::system_clock::now();

    for (auto node : client_nodes){
        std::thread thread = std::thread(&SimpleClientNode::run_request_loop, node, request_frequency);
        thread.detach();
    }


    std::this_thread::sleep_for(std::chrono::seconds(15));


    rclcpp::shutdown();

    for (auto node : client_nodes){
        node->get_count(start_time);
    }


    return 0;

}
