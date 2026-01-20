/**
 * @file z1_control_node.cpp
 * @brief 双臂 Z1 控制节点
 *        使用 MoveIt 接收末端位姿控制两个机械臂
 *        使用 actionlib 控制夹爪
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @class SingleArmController
 * @brief 单臂控制器类
 */
class SingleArmController
{
public:
    SingleArmController(ros::NodeHandle& nh, 
                        const std::string& arm_name,
                        const std::string& planning_group,
                        const std::string& gripper_action_name)
        : nh_(nh),
          arm_name_(arm_name),
          move_group_(planning_group),
          gripper_client_(gripper_action_name, true)
    {
        // 配置 MoveIt
        move_group_.setPlanningTime(10.0);
        move_group_.setNumPlanningAttempts(5);
        move_group_.setMaxVelocityScalingFactor(0.3);
        move_group_.setMaxAccelerationScalingFactor(0.3);

        // 获取末端执行器 link
        end_effector_link_ = move_group_.getEndEffectorLink();
        ROS_INFO("[%s] End effector link: %s", arm_name_.c_str(), end_effector_link_.c_str());

        // 参考坐标系
        reference_frame_ = move_group_.getPlanningFrame();
        ROS_INFO("[%s] Planning frame: %s", arm_name_.c_str(), reference_frame_.c_str());

        // 等待夹爪 action server
        ROS_INFO("[%s] Waiting for gripper action server: %s", arm_name_.c_str(), gripper_action_name.c_str());
        if (gripper_client_.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("[%s] Gripper action server connected.", arm_name_.c_str());
            gripper_available_ = true;
        }
        else
        {
            ROS_WARN("[%s] Gripper action server not available.", arm_name_.c_str());
            gripper_available_ = false;
        }

        // 订阅话题
        pose_sub_ = nh_.subscribe(arm_name + "/target_pose", 1, &SingleArmController::poseCallback, this);
        gripper_sub_ = nh_.subscribe(arm_name + "/gripper_command", 1, &SingleArmController::gripperCallback, this);

        ROS_INFO("[%s] Arm controller initialized.", arm_name_.c_str());
    }

    /**
     * @brief 移动到目标位姿
     */
    bool moveToPose(const geometry_msgs::Pose& target)
    {
        ROS_INFO("[%s] Moving to target pose: [%.3f, %.3f, %.3f]",
                 arm_name_.c_str(), target.position.x, target.position.y, target.position.z);

        move_group_.setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("[%s] Planning succeeded. Executing...", arm_name_.c_str());
            move_group_.execute(plan);
            ROS_INFO("[%s] Motion completed.", arm_name_.c_str());
            return true;
        }
        else
        {
            ROS_WARN("[%s] Motion planning failed.", arm_name_.c_str());
            return false;
        }
    }

    /**
     * @brief 控制夹爪
     */
    bool controlGripper(double position, double max_effort = 20.0)
    {
        if (!gripper_available_)
        {
            ROS_WARN("[%s] Gripper action server not available.", arm_name_.c_str());
            return false;
        }

        control_msgs::GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;

        ROS_INFO("[%s] Sending gripper command: position=%.3f", arm_name_.c_str(), position);

        gripper_client_.sendGoal(goal);
        bool finished = gripper_client_.waitForResult(ros::Duration(5.0));

        if (finished)
        {
            auto state = gripper_client_.getState();
            ROS_INFO("[%s] Gripper action finished: %s", arm_name_.c_str(), state.toString().c_str());
            return (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        }
        else
        {
            ROS_WARN("[%s] Gripper action timed out.", arm_name_.c_str());
            gripper_client_.cancelGoal();
            return false;
        }
    }

private:
    ros::NodeHandle& nh_;
    std::string arm_name_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

    ros::Subscriber pose_sub_;
    ros::Subscriber gripper_sub_;

    std::string end_effector_link_;
    std::string reference_frame_;
    bool gripper_available_;

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        moveToPose(*msg);
    }

    void gripperCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        controlGripper(msg->data);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "z1_dual_control_node");
    ros::NodeHandle nh("~");

    // 等待 MoveIt 初始化
    ros::Duration(1.0).sleep();

    // 初始化异步 spinner
    ros::AsyncSpinner spinner(4);  // 4 线程支持双臂
    spinner.start();

    // 读取参数
    std::string left_planning_group, right_planning_group;
    nh.param<std::string>("left_planning_group", left_planning_group, "left_arm_manipulator");
    nh.param<std::string>("right_planning_group", right_planning_group, "right_arm_manipulator");

    std::string left_gripper_action, right_gripper_action;
    nh.param<std::string>("left_gripper_action", left_gripper_action, "left_arm/left_z1_gripper");
    nh.param<std::string>("right_gripper_action", right_gripper_action, "right_arm/right_z1_gripper");

    try
    {
        // 创建左臂控制器
        ROS_INFO("Initializing left arm controller...");
        SingleArmController left_controller(nh, "left_arm", left_planning_group, left_gripper_action);

        // 创建右臂控制器
        ROS_INFO("Initializing right arm controller...");
        SingleArmController right_controller(nh, "right_arm", right_planning_group, right_gripper_action);

        ROS_INFO("===========================================");
        ROS_INFO("Dual Z1 Control Node Ready");
        ROS_INFO("===========================================");
        ROS_INFO("Left Arm Topics:");
        ROS_INFO("  - ~left_arm/target_pose (geometry_msgs/Pose)");
        ROS_INFO("  - ~left_arm/gripper_command (std_msgs/Float64)");
        ROS_INFO("Right Arm Topics:");
        ROS_INFO("  - ~right_arm/target_pose (geometry_msgs/Pose)");
        ROS_INFO("  - ~right_arm/gripper_command (std_msgs/Float64)");
        ROS_INFO("===========================================");

        ros::waitForShutdown();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
