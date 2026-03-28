#include <chrono>
#include <memory>
#include <string>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: gripper_ctrl <gripper_goal_angle> <option: max_effort>\n"
              << "gripper_goal_angle: [-1.57, 0] rad, negative direction indicates open.\n"
              << "max_effort: [3, 20] N·m, default: 10 N·m" << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gripper_ctrl");
  auto action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    node, "z1_gripper");

  if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available.");
    rclcpp::shutdown();
    return -1;
  }

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = std::stod(argv[1]);
  goal.command.max_effort = argc == 3 ? std::stod(argv[2]) : 10.0;

  auto future_handle = action_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, future_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send gripper goal.");
    rclcpp::shutdown();
    return -1;
  }

  auto goal_handle = future_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal was rejected.");
    rclcpp::shutdown();
    return -1;
  }

  auto result_future = action_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to receive gripper result.");
    rclcpp::shutdown();
    return -1;
  }

  const auto wrapped_result = result_future.get();
  const auto & result = wrapped_result.result;
  RCLCPP_INFO(
    node->get_logger(),
    "Gripper move finished. Angle: %.4f, Effort: %.4f",
    result->position,
    result->effort);

  rclcpp::shutdown();
  return 0;
}
