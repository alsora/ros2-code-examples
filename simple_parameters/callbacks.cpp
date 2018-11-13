
#include <chrono>
#include <iostream>
#include <cinttypes>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/parameter.hpp"

using namespace std::chrono_literals;



void my_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::stringstream ss;
  ss << "\nParameter event:\n new parameters:";
  for (rcl_interfaces::msg::Parameter & new_parameter : event->new_parameters) {
    ss << "\n  " << new_parameter.name;
  }
  ss << "\n changed parameters:";
  for (rcl_interfaces::msg::Parameter & changed_parameter : event->changed_parameters) {
    ss << "\n  " << changed_parameter.name;
  }
  ss << "\n deleted parameters:";
  for (rcl_interfaces::msg::Parameter & deleted_parameter : event->deleted_parameters) {
    ss << "\n  " << deleted_parameter.name;
  }
  ss << "\n";

  std::cout<< ss.str() << std::endl;

}


void node_loop(const std::string& name)
{

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(name);


    rclcpp::WallRate loop_rate(250ms);
    while (rclcpp::ok()){

        rclcpp::spin_some(node);

        //node->set_parameters({rclcpp::Parameter("dummy", 1)});

        loop_rate.sleep();

    }



}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node1 = rclcpp::Node::make_shared("node1");

    std::thread t2(node_loop, "node2");
    t2.detach();

    std::thread t3(node_loop, "node3");
    t3.detach();

    // create the parameters client
    rclcpp::SyncParametersClient::SharedPtr parameters_client2 = std::make_shared<rclcpp::SyncParametersClient>(node1, "node2");
    // initialize the parameters client
    while (!parameters_client2->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node1->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node1->get_logger(), "service not available, waiting again...");
    }

    rclcpp::SyncParametersClient::SharedPtr parameters_client3 = std::make_shared<rclcpp::SyncParametersClient>(node1, "node3");
    // initialize the parameters client
    while (!parameters_client3->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node1->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node1->get_logger(), "service not available, waiting again...");
    }



    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub = parameters_client2->on_parameter_event(my_callback);


    // set some parameters

    // the callback should be triggered since I'm changin parameters on node2
    std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_results2 = parameters_client2->set_parameters({rclcpp::Parameter("wheels.radius", 1),
                                                                                                                      rclcpp::Parameter("wheels.weight", 0.5),
                                                                                                                      rclcpp::Parameter("use_odometry", true)});
    for (rcl_interfaces::msg::SetParametersResult &result : set_parameters_results2)
    {
        if (!result.successful)
        {
            RCLCPP_ERROR(node1->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
        }
    }

    // the callback should not be triggered since I'm changin parameters on node3
    std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_results3 = parameters_client3->set_parameters({rclcpp::Parameter("pippo.radius", 1),
                                                                                                                      rclcpp::Parameter("pippo.weight", 0.5),
                                                                                                                      rclcpp::Parameter("pippo", true)});
    for (rcl_interfaces::msg::SetParametersResult &result : set_parameters_results3)
    {
        if (!result.successful)
        {
            RCLCPP_ERROR(node1->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
        }
    }


    rclcpp::shutdown();
    return 0;
}
