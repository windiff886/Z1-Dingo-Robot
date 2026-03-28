#ifndef Z1_HARDWARE__Z1_SYSTEM_HPP_
#define Z1_HARDWARE__Z1_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace UNITREE_ARM_SDK { class UnitreeArm; }

namespace z1_hardware {

class Z1System : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Z1System)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<UNITREE_ARM_SDK::UnitreeArm> arm_;

  // Joint state storage (6 arm joints)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;

  // Gripper
  double gripper_position_{0.0};
  double gripper_velocity_{0.0};
  double gripper_effort_{0.0};
  double gripper_command_{0.0};

  // Gripper SDK 写入参数（与 ROS1 z1_hw 对齐）
  double gripper_max_tau_{10.0};   // 抓取最大力矩 (N·m)，SDK 默认值为 10
  double gripper_epsilon_{0.01};   // 抓取角度容差 (rad)，与 ROS1 默认值一致

  bool has_gripper_{true};
  std::string mcu_ip_{"192.168.123.110"};
  int port_to_mcu_{8881};
  int port_to_sdk_{8871};
  std::string controller_ip_{"127.0.0.1"};
  int own_port_{8872};

  rclcpp::Logger logger_{rclcpp::get_logger("Z1System")};
};

}  // namespace z1_hardware

#endif  // Z1_HARDWARE__Z1_SYSTEM_HPP_
