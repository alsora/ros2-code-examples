#include <inttypes.h>
#include <memory>
#include "simple_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = simple_interfaces::action::Fibonacci;

rclcpp::Node::SharedPtr g_node = nullptr;

void feedback_callback(
  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  RCLCPP_INFO(
    g_node->get_logger(),
    "Next number in sequence received: %" PRId64,
    feedback->sequence.back());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("simple_action_client");
  auto action_client = rclcpp_action::create_client<Fibonacci>(g_node, "fibonacci");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(g_node->get_logger(), "Action server not available after waiting");
    assert(0);
  }

  RCLCPP_INFO(g_node->get_logger(), "Client created!");

  // Populate a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 3;

  RCLCPP_INFO(g_node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto goal_handle_future = action_client->async_send_goal(goal_msg, feedback_callback);

  if (rclcpp::spin_until_future_complete(g_node, goal_handle_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(g_node->get_logger(), "Send goal call failed :(");
    assert(0);
  }

  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(g_node->get_logger(), "Goal was rejected by server");
    assert(0);
  }

  // Wait for the server to be done with the goal
  auto result_future = goal_handle->async_result();

  std::chrono::seconds request_timeout(5);

  RCLCPP_INFO(g_node->get_logger(), "Waiting for result");
  auto wait_result = rclcpp::spin_until_future_complete(
    g_node,
    result_future,
    request_timeout);

  if (wait_result == rclcpp::executor::FutureReturnCode::TIMEOUT){
    RCLCPP_INFO(g_node->get_logger(), "canceling goal");
    // Cancel the goal since it is taking too long
    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
    if (rclcpp::spin_until_future_complete(g_node, cancel_result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(g_node->get_logger(), "failed to cancel goal");
      rclcpp::shutdown();
      assert(0);
    }
    RCLCPP_INFO(g_node->get_logger(), "goal is being canceled");
  }
  else if (wait_result == rclcpp::executor::FutureReturnCode::SUCCESS){

    rclcpp_action::ClientGoalHandle<Fibonacci>::Result result = result_future.get();
    switch(result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(g_node->get_logger(), "result received");
        for (auto number : result.response->sequence){
          RCLCPP_INFO(g_node->get_logger(), "%" PRId64, number);
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(g_node->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(g_node->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(g_node->get_logger(), "Unknown result code");
        rclcpp::shutdown();
        assert(0);
    }
  }
  else {
    RCLCPP_ERROR(g_node->get_logger(), "failed to get result");
    rclcpp::shutdown();
    assert(0);
  }

  action_client.reset();
  g_node.reset();
  rclcpp::shutdown();
  return 0;
}