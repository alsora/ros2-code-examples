#ifndef __SIMPLE_PARAMETER_SERVER_NODE_HPP__
#define __SIMPLE_PARAMETER_SERVER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"


class SimpleParameterServerNode : public rclcpp::Node {

public:

    SimpleParameterServerNode();

private:

    void parameters_init();


};

#endif // __SIMPLE_PARAMETER_SERVER_NODE_HPP__