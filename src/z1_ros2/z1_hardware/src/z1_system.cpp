#include "z1_hardware/z1_system.hpp"
#include "unitree_arm_sdk/unitree_arm.h"

#include <chrono>
#include <cmath>

namespace z1_hardware {

hardware_interface::CallbackReturn Z1System::on_init(
    const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 保留 ROS1 风格的完整参数面，哪怕当前硬件接口主要通过 z1_ctrl_ros2 间接通信。
  mcu_ip_ = info_.hardware_parameters.at("mcu_ip");
  port_to_mcu_ = std::stoi(info_.hardware_parameters.at("port_to_mcu"));
  port_to_sdk_ = std::stoi(info_.hardware_parameters.at("port_to_sdk"));
  controller_ip_ = info_.hardware_parameters.at("controller_ip");
  own_port_ = std::stoi(info_.hardware_parameters.at("own_port"));
  has_gripper_ = (info_.hardware_parameters.count("has_gripper") &&
                  info_.hardware_parameters.at("has_gripper") == "true");

  // Gripper SDK 写入参数（可选，有默认值）
  if (info_.hardware_parameters.count("gripper_max_tau")) {
    gripper_max_tau_ = std::stod(info_.hardware_parameters.at("gripper_max_tau"));
  }
  if (info_.hardware_parameters.count("gripper_epsilon")) {
    gripper_epsilon_ = std::stod(info_.hardware_parameters.at("gripper_epsilon"));
  }

  // Validate joint count
  size_t expected_joints = has_gripper_ ? 7 : 6;
  if (info_.joints.size() != expected_joints)
  {
    RCLCPP_ERROR(logger_,
      "Expected %zu joints, got %zu", expected_joints, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(6, 0.0);
  hw_velocities_.resize(6, 0.0);
  hw_efforts_.resize(6, 0.0);
  hw_commands_.resize(6, 0.0);

  RCLCPP_INFO(
    logger_,
    "Z1System initialized: mcu_ip=%s, port_to_mcu=%d, controller_ip=%s, own_port=%d, port_to_sdk=%d, gripper=%s",
    mcu_ip_.c_str(),
    port_to_mcu_,
    controller_ip_.c_str(),
    own_port_,
    port_to_sdk_,
    has_gripper_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Z1System::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger_,
    "Configuring Z1 arm connection via bridge %s:%d (mcu=%s:%d, sdk_port=%d)...",
    controller_ip_.c_str(),
    own_port_,
    mcu_ip_.c_str(),
    port_to_mcu_,
    port_to_sdk_);

  try {
    arm_ = std::make_unique<UNITREE_ARM_SDK::UnitreeArm>(
        controller_ip_, static_cast<uint>(own_port_), static_cast<uint>(port_to_sdk_));
    arm_->init();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to initialize Z1 arm: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Z1 arm connection established.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Z1System::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating Z1 arm...");

  // Read current arm position and sync commands to prevent jumps
  for (int i = 0; i < 6; i++) {
    hw_positions_[i] = arm_->armState.q[i];
    hw_commands_[i] = hw_positions_[i];
  }
  if (has_gripper_) {
    gripper_position_ = arm_->armState.gripperState.angle;
    gripper_command_ = gripper_position_;
  }

  RCLCPP_INFO(logger_, "Z1 arm activated. Initial joints synced.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Z1System::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating Z1 arm (passive mode)...");
  arm_->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  arm_->sendRecv();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Z1System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // First 6 joints are arm joints
  for (size_t i = 0; i < 6; i++) {
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }

  // Gripper joint (index 6)
  if (has_gripper_) {
    state_interfaces.emplace_back(
        info_.joints[6].name, hardware_interface::HW_IF_POSITION, &gripper_position_);
    state_interfaces.emplace_back(
        info_.joints[6].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_);
    state_interfaces.emplace_back(
        info_.joints[6].name, hardware_interface::HW_IF_EFFORT, &gripper_effort_);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Z1System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < 6; i++) {
    command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  if (has_gripper_) {
    command_interfaces.emplace_back(
        info_.joints[6].name, hardware_interface::HW_IF_POSITION, &gripper_command_);
  }

  return command_interfaces;
}

hardware_interface::return_type Z1System::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (int i = 0; i < 6; i++) {
    hw_positions_[i] = arm_->armState.q[i];
    hw_velocities_[i] = arm_->armState.dq[i];
    hw_efforts_[i] = arm_->armState.tau[i];
  }

  if (has_gripper_) {
    gripper_position_ = arm_->armState.gripperState.angle;
    gripper_velocity_ = arm_->armState.gripperState.speed;
    gripper_effort_ = arm_->armState.gripperState.tau;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Z1System::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  arm_->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;

  for (int i = 0; i < 6; i++) {
    arm_->armCmd.q_d[i] = hw_commands_[i];
  }

  if (has_gripper_) {
    arm_->armCmd.gripperCmd.angle = gripper_command_;
    arm_->armCmd.gripperCmd.maxTau = gripper_max_tau_;
    arm_->armCmd.gripperCmd.epsilon_inner = gripper_epsilon_;
    arm_->armCmd.gripperCmd.epsilon_outer = gripper_epsilon_;
  }

  arm_->sendRecv();
  return hardware_interface::return_type::OK;
}

}  // namespace z1_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(z1_hardware::Z1System, hardware_interface::SystemInterface)
