/**
 * io_ros2.cpp
 *
 * ROS2 移植自 z1_controller/sim/IOROS.cpp
 * 完整对齐 ROS1 的 IOROS 发布/接收逻辑：
 *   - 命令通过 uni_dingo_msgs/MotorCmd 发布（含 q, dq, tau, kp, kd, mode）
 *   - 状态通过 uni_dingo_msgs/MotorState 独立话题接收（含 q, dq, ddq, tau_est, mode）
 *
 * 话题映射（与 ROS1 完全一致）：
 *   发布：/z1_gazebo/Joint0X_controller/command  (X = 1..6)
 *         /z1_gazebo/gripper_controller/command
 *   订阅：/z1_gazebo/Joint0X_controller/state    (X = 1..6)
 *         /z1_gazebo/gripper_controller/state
 */

#include "z1_controller_ros2/io_ros2.hpp"
#include <chrono>
#include <thread>

// 命令发布话题（名称与 ROS1 完全相同）
static const std::array<std::string, 6> CMD_TOPICS = {
  "/z1_gazebo/Joint01_controller/command",
  "/z1_gazebo/Joint02_controller/command",
  "/z1_gazebo/Joint03_controller/command",
  "/z1_gazebo/Joint04_controller/command",
  "/z1_gazebo/Joint05_controller/command",
  "/z1_gazebo/Joint06_controller/command",
};
static const std::string GRIPPER_CMD_TOPIC = "/z1_gazebo/gripper_controller/command";

// 状态订阅话题（名称与 ROS1 完全相同）
static const std::array<std::string, 6> STATE_TOPICS = {
  "/z1_gazebo/Joint01_controller/state",
  "/z1_gazebo/Joint02_controller/state",
  "/z1_gazebo/Joint03_controller/state",
  "/z1_gazebo/Joint04_controller/state",
  "/z1_gazebo/Joint05_controller/state",
  "/z1_gazebo/Joint06_controller/state",
};
static const std::string GRIPPER_STATE_TOPIC = "/z1_gazebo/gripper_controller/state";

IOROS2::IOROS2(rclcpp::Node::SharedPtr node)
: _node(node)
{
  RCLCPP_INFO(_node->get_logger(),
    "IOROS2: ROS2 Gazebo simulation backend initializing.");

  // 先初始化订阅者
  _initSubscribers();
  // 启动 AsyncSpinner 等效：等待订阅者开始接收（对齐 ROS1 的 usleep(300000)）
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  // 再初始化发布者
  _initPublishers();

  initialized_ = true;

  // 从参数读取 gripper 开关（对齐 ROS1 的 _nm.param）
  _node->declare_parameter<bool>("z1_ctrl.UnitreeGripperYN", false);
  _node->get_parameter("z1_ctrl.UnitreeGripperYN", hasGripper);
}

bool IOROS2::init()
{
  // 发布一次带 mode=10 的命令触发 Gazebo 响应，检测连接（对齐 ROS1 IOROS::init）
  uni_dingo_msgs::msg::MotorCmd cmd_msg;
  cmd_msg.mode = 10;
  cmd_msg.q = 0.0f;
  cmd_msg.dq = 0.0f;
  cmd_msg.tau = 0.0f;
  cmd_msg.kp = 0.0f;
  cmd_msg.kd = 0.0f;

  _servo_pub[0]->publish(cmd_msg);

  // 检查夹爪是否存在
  _gripper_pub->publish(cmd_msg);

  // 稍等，让 Gazebo 有机会响应
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(_node);

  std::lock_guard<std::mutex> lock(_state_mutex);
  return _is_connected;
}

bool IOROS2::isConnected()
{
  // 对齐 ROS1: return true;
  return true;
}

bool IOROS2::sendRecv(const LowLevelCmd *cmd, LowLevelState *state)
{
  _sendCmd(cmd);
  rclcpp::spin_some(_node);
  _recvState(state);
  return true;
}

void IOROS2::_sendCmd(const LowLevelCmd *lowCmd)
{
  // 对齐 ROS1 IOROS::_sendCmd — 发送完整的 MotorCmd（含 q, dq, tau, Kp, Kd, mode）
  for (size_t i = 0; i < 6; ++i) {
    uni_dingo_msgs::msg::MotorCmd cmd_msg;
    cmd_msg.mode = 10;
    cmd_msg.q    = static_cast<float>(lowCmd->q[i]);
    cmd_msg.dq   = static_cast<float>(lowCmd->dq[i]);
    cmd_msg.tau  = static_cast<float>(lowCmd->tau[i]);
    cmd_msg.kp   = static_cast<float>(lowCmd->Kp[i]);
    cmd_msg.kd   = static_cast<float>(lowCmd->Kd[i]);

    _servo_pub[i]->publish(cmd_msg);
  }

  if (hasGripper) {
    uni_dingo_msgs::msg::MotorCmd gripper_msg;
    gripper_msg.q   = static_cast<float>(lowCmd->q.back());
    gripper_msg.dq  = static_cast<float>(lowCmd->dq.back());
    gripper_msg.tau = static_cast<float>(lowCmd->tau.back());
    gripper_msg.kp  = static_cast<float>(lowCmd->Kp.back());
    gripper_msg.kd  = static_cast<float>(lowCmd->Kd.back());
    _gripper_pub->publish(gripper_msg);
  }
}

void IOROS2::_recvState(LowLevelState *state)
{
  // 对齐 ROS1 IOROS::_recvState — 读取完整的 MotorState（含 ddq → tauEst）
  std::lock_guard<std::mutex> lock(_state_mutex);
  for (size_t i = 0; i < 6; ++i) {
    state->q[i]   = _joint_states[i].q;
    state->dq[i]  = _joint_states[i].dq;
    state->tau[i] = _joint_states[i].tau_est;
  }
  state->q.back()   = _gripper_state.q;
  state->dq.back()  = _gripper_state.dq;
  state->tau.back() = _gripper_state.tau_est;
}

void IOROS2::_initPublishers()
{
  for (size_t i = 0; i < 6; ++i) {
    _servo_pub[i] = _node->create_publisher<uni_dingo_msgs::msg::MotorCmd>(
      CMD_TOPICS[i], 1);
  }
  _gripper_pub = _node->create_publisher<uni_dingo_msgs::msg::MotorCmd>(
    GRIPPER_CMD_TOPIC, 1);
}

void IOROS2::_initSubscribers()
{
  // 每关节独立订阅（对齐 ROS1 IOROS::_initRecv 的 7 个独立订阅者）
  _servo_sub[0] = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    STATE_TOPICS[0], 1,
    std::bind(&IOROS2::_joint00Callback, this, std::placeholders::_1));
  _servo_sub[1] = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    STATE_TOPICS[1], 1,
    std::bind(&IOROS2::_joint01Callback, this, std::placeholders::_1));
  _servo_sub[2] = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    STATE_TOPICS[2], 1,
    std::bind(&IOROS2::_joint02Callback, this, std::placeholders::_1));
  _servo_sub[3] = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    STATE_TOPICS[3], 1,
    std::bind(&IOROS2::_joint03Callback, this, std::placeholders::_1));
  _servo_sub[4] = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    STATE_TOPICS[4], 1,
    std::bind(&IOROS2::_joint04Callback, this, std::placeholders::_1));
  _servo_sub[5] = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    STATE_TOPICS[5], 1,
    std::bind(&IOROS2::_joint05Callback, this, std::placeholders::_1));
  _gripper_sub = _node->create_subscription<uni_dingo_msgs::msg::MotorState>(
    GRIPPER_STATE_TOPIC, 1,
    std::bind(&IOROS2::_gripperCallback, this, std::placeholders::_1));
}

// ── 关节状态回调（对齐 ROS1 IOROS::_joint0XCallback）──────────────────────

void IOROS2::_joint00Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _is_connected = true;
  _joint_states[0].mode    = msg->mode;
  _joint_states[0].q       = msg->q;
  _joint_states[0].dq      = msg->dq;
  _joint_states[0].ddq     = msg->ddq;
  _joint_states[0].tau_est = msg->tau_est;
}

void IOROS2::_joint01Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _joint_states[1].mode    = msg->mode;
  _joint_states[1].q       = msg->q;
  _joint_states[1].dq      = msg->dq;
  _joint_states[1].ddq     = msg->ddq;
  _joint_states[1].tau_est = msg->tau_est;
}

void IOROS2::_joint02Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _joint_states[2].mode    = msg->mode;
  _joint_states[2].q       = msg->q;
  _joint_states[2].dq      = msg->dq;
  _joint_states[2].ddq     = msg->ddq;
  _joint_states[2].tau_est = msg->tau_est;
}

void IOROS2::_joint03Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _joint_states[3].mode    = msg->mode;
  _joint_states[3].q       = msg->q;
  _joint_states[3].dq      = msg->dq;
  _joint_states[3].ddq     = msg->ddq;
  _joint_states[3].tau_est = msg->tau_est;
}

void IOROS2::_joint04Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _joint_states[4].mode    = msg->mode;
  _joint_states[4].q       = msg->q;
  _joint_states[4].dq      = msg->dq;
  _joint_states[4].ddq     = msg->ddq;
  _joint_states[4].tau_est = msg->tau_est;
}

void IOROS2::_joint05Callback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _joint_states[5].mode    = msg->mode;
  _joint_states[5].q       = msg->q;
  _joint_states[5].dq      = msg->dq;
  _joint_states[5].ddq     = msg->ddq;
  _joint_states[5].tau_est = msg->tau_est;
}

void IOROS2::_gripperCallback(const uni_dingo_msgs::msg::MotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_state_mutex);
  _gripper_state.mode    = msg->mode;
  _gripper_state.q       = msg->q;
  _gripper_state.dq      = msg->dq;
  _gripper_state.ddq     = msg->ddq;
  _gripper_state.tau_est = msg->tau_est;
}
