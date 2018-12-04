

#include "simple_service_node.hpp"


using GetImageSrv = simple_interfaces::srv::GetImage;
using ImageMsg = sensor_msgs::msg::Image;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

SimpleServiceNode::SimpleServiceNode(std::string name) : Node(name)
{

    _service = this->create_service<GetImageSrv>("get_image", std::bind(&SimpleServiceNode::handle_service, this, _1, _2, _3));


    RCLCPP_INFO(this->get_logger(), "Service created!!");
}


void SimpleServiceNode::handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<GetImageSrv::Request> request,
        const std::shared_ptr<GetImageSrv::Response> response)
{


    (void)request_header;

    RCLCPP_INFO(this->get_logger(),"request received");

    auto img = ImageMsg();

    // Generate fake frame
    img.width = request->res_w;
    img.height = request->res_h;
    img.data.resize(img.width * img.height);
    for (size_t i = 0; i < img.width * img.height; i++){
        img.data[i] = ( std::rand() % ( 255 + 1 ) );
    }

    response->image = img;

}


