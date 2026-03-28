#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_interface");

  static const std::string planning_group = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, planning_group);
  move_group_interface.setMaxVelocityScalingFactor(0.15);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  geometry_msgs::msg::Pose target_pose_1;
  target_pose_1.orientation.w = 0.0;
  target_pose_1.position.x = 0.3;
  target_pose_1.position.y = 0.0;
  target_pose_1.position.z = 0.4;
  move_group_interface.setPoseTarget(target_pose_1);

  moveit::planning_interface::MoveGroupInterface::Plan plan_1;
  const bool success =
    static_cast<bool>(move_group_interface.plan(plan_1));
  if (success) {
    move_group_interface.execute(plan_1);
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose_1);

  auto target_pose_2 = target_pose_1;
  target_pose_2.position.x += 0.1;
  waypoints.push_back(target_pose_2);
  target_pose_2.position.z -= 0.1;
  waypoints.push_back(target_pose_2);

  moveit_msgs::msg::RobotTrajectory trajectory;
  move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan plan_2;
  plan_2.trajectory_ = trajectory;
  move_group_interface.execute(plan_2);

  move_group_interface.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  if (static_cast<bool>(move_group_interface.plan(plan_home))) {
    move_group_interface.execute(plan_home);
  }

  rclcpp::shutdown();
  return 0;
}
