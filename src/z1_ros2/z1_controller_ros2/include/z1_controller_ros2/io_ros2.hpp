/**
 * io_ros2.hpp
 *
 * ROS2 移植自 z1_controller/sim/IOROS.h
 * 仿真后端：通过 ROS2 topic 与 Gazebo 交互。
 *
 * 话题映射（与 ROS1 完全一致）：
 *   命令发布: /z1_gazebo/Joint0X_controller/command  [uni_dingo_msgs/MotorCmd]
 *   状态订阅: /z1_gazebo/Joint0X_controller/state    [uni_dingo_msgs/MotorState]
 *   夹爪命令: /z1_gazebo/gripper_controller/command   [uni_dingo_msgs/MotorCmd]
 *   夹爪状态: /z1_gazebo/gripper_controller/state     [uni_dingo_msgs/MotorState]
 */

#ifndef IO_ROS2_HPP
#define IO_ROS2_HPP

#include <array>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <uni_dingo_msgs/msg/motor_cmd.hpp>
#include <uni_dingo_msgs/msg/motor_state.hpp>

#include "IOInterface/IOInterface.h"
#include "IOInterface/LowLevelMsg/LowLevelCmd.h"
#include "IOInterface/LowLevelMsg/LowLevelState.h"

/**
 * @brief 仿真关节状态缓存（对齐 ROS1 unitree_legged_msgs::MotorState 全部字段）
 */
struct SimMotorState {
  uint8_t mode{0};
  double q{0.0};
  double dq{0.0};
  double ddq{0.0};
  double tau_est{0.0};
};

class IOROS2 : public IOInterface
{
public:
  explicit IOROS2(rclcpp::Node::SharedPtr node);
  ~IOROS2() = default;

  bool init() override;
  bool isConnected() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;

private:
  void _sendCmd(const LowLevelCmd *cmd);
  void _recvState(LowLevelState *state);
  void _initPublishers();
  void _initSubscribers();

  // 每关节独立状态回调（对齐 ROS1 IOROS 的独立话题订阅方式）
  void _joint00Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);
  void _joint01Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);
  void _joint02Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);
  void _joint03Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);
  void _joint04Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);
  void _joint05Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);
  void _gripperCallback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg);

  rclcpp::Node::SharedPtr _node;

  // 命令发布（每关节独立，与 ROS1 话题名完全相同）
  std::array<rclcpp::Publisher<uni_dingo_msgs::msg::MotorCmd>::SharedPtr, 6> _servo_pub;
  rclcpp::Publisher<uni_dingo_msgs::msg::MotorCmd>::SharedPtr _gripper_pub;

  // 状态订阅（每关节独立，与 ROS1 完全相同）
  std::array<rclcpp::Subscription<uni_dingo_msgs::msg::MotorState>::SharedPtr, 6> _servo_sub;
  rclcpp::Subscription<uni_dingo_msgs::msg::MotorState>::SharedPtr _gripper_sub;

  // 关节状态缓存（6 关节 + 1 夹爪）
  std::array<SimMotorState, 6> _joint_states{};
  SimMotorState _gripper_state{};

  bool _is_connected{false};
  mutable std::mutex _state_mutex;
};

#endif  // IO_ROS2_HPP
