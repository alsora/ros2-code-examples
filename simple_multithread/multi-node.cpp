#include "multi-node.hpp"

#include <string>
#include <sstream>

std::string id_to_node_name(int id)
{
  std::stringstream ss;
  ss << "node_";
  ss << id;
  return ss.str();
}

std::string id_to_service_name(int id)
{
  std::stringstream ss;
  ss << "service_";
  ss << id;
  return ss.str();
}

std::string id_to_topic_name(int id)
{
  std::stringstream ss;
  ss << "topic_";
  ss << id;
  return ss.str();
}


MultiNode::MultiNode(int  id) 
  : Node(id_to_node_name(id))
{

  _id = id;
  _name = id_to_node_name(id);

  this->stats = Stats();

}

MultiNode::~MultiNode()
{

  this->_subscribers.clear();
  this->_publishers.clear();
  this->_clients.clear();
  this->_services.clear();

}


void MultiNode::add_publisher(int id)
{

  std::string topic_name = id_to_topic_name(id);

  std::stringstream ss;
  ss<< "Created publisher from " << _name << " to "<< topic_name << "\n";
  std::cout<< ss.str();

  auto it = _publishers.find(id); 
  if (it == _publishers.end()){
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher = this->create_publisher<std_msgs::msg::Header>(
      topic_name);

    _publishers[id] = publisher;
  }

}


void MultiNode::add_subscriber(int id)
{

  std::string topic_name = id_to_topic_name(id);

  std::stringstream ss;
  ss<< "Created subscriber from " << _name << " to "<< topic_name << "\n";
  std::cout<< ss.str();

  auto it = _subscribers.find(id); 
  if (it == _subscribers.end()){
    
    std::function<void(const std_msgs::msg::Header::SharedPtr msg)> fcn = std::bind(&MultiNode::topic_callback, this, std::placeholders::_1, id);

    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscriber = this->create_subscription<std_msgs::msg::Header>(
      topic_name, 
      fcn );

    _subscribers[id] = subscriber;

    this->stats.subscription_delta_times[id] = std::make_pair(0, 0);
  }

}

void MultiNode::add_service(int id)
{

  std::string service_name = id_to_service_name(id);

  std::stringstream ss;
  ss<< "Created service from " << _name << " to "<< service_name << "\n";
  std::cout<< ss.str();

  auto it = _services.find(id); 
  if (it == _services.end()){
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service = this->create_service<std_srvs::srv::Empty>(
    service_name, 
    std::bind(&MultiNode::service_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

      _services[id] = service;
  }

}


void MultiNode::add_client(int id)
{

  std::string service_name = id_to_service_name(id);

  std::stringstream ss;
  ss<< "Created client from " << _name << " to "<< service_name << "\n";
  std::cout<< ss.str();

  auto it = _clients.find(id); 
  if (it == _clients.end()){

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client = this->create_client<std_srvs::srv::Empty>(
      service_name);

    _clients[id] = client;

    this->stats.client_requests_latency[id] = std::make_pair(0, 0);
  }

}

void MultiNode::add_timer(std::chrono::milliseconds time, std::function< void() > callback)
{

  rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(time, callback);

  _timers.push_back(timer);

}




void MultiNode::service_handler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;

}

void MultiNode::topic_callback(const std_msgs::msg::Header::SharedPtr msg, int id)
{
  rclcpp::Duration rcl_duration = this->now() - rclcpp::Time(msg->stamp);
  int64_t delta_microseconds = RCL_NS_TO_US(static_cast<int64_t>(rcl_duration.nanoseconds()));
 
  auto chrono_duration = duration_microseconds(delta_microseconds);

  this->stats.subscription_delta_times[id].first += chrono_duration;
  this->stats.subscription_delta_times[id].second ++;

}



void MultiNode::simple_publisher_task()
{

  rclcpp::WallRate loop_rate(2);
  while(rclcpp::ok()){

    std_msgs::msg::Header message = std_msgs::msg::Header();

    message.stamp = this->now();
    message.frame_id = _name;

    for (auto const& map_item : _publishers){

      rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher = map_item.second;
      publisher->publish(message);
    
    }

    loop_rate.sleep();
  }

  return;
}


void MultiNode::simple_client_task()
{

  for (auto const& map_item : _clients){
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client = map_item.second;
    while (!client->wait_for_service(std::chrono::milliseconds(50))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        assert(0);
      }
    }
  }


  rclcpp::WallRate loop_rate(10);
  while(rclcpp::ok()){

    for (auto const& map_item : _clients){

      int client_id = map_item.first;
      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client = map_item.second;


      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

      std::shared_ptr<std_srvs::srv::Empty::Request> request;
      request = std::make_shared<std_srvs::srv::Empty::Request>();

      rclcpp::Client<std_srvs::srv::Empty>::SharedFuture result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
          rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        continue;
      }
      
      std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
      auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

      this->stats.client_requests_latency[client_id].first += request_duration;
      this->stats.client_requests_latency[client_id].second ++;

    }

    loop_rate.sleep();
  }

  return;
}
