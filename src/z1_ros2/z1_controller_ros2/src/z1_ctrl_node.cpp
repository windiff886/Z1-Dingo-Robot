/**
 * z1_ctrl_node.cpp
 *
 * ROS2 直接移植自 z1_controller/main.cpp
 * 保留所有 FSM 对象实例化逻辑，仅替换 ROS1 API 为 ROS2 API。
 *
 * 通信链（与 ROS1 完全相同）:
 *   Z1System (UnitreeArm SDK) ──UDP──► z1_ctrl (UdpSdk, port_to_sdk)
 *                                           │ FSM
 *                                      IOUDP ──UDP──► MCU (mcu_ip:port_to_mcu)
 */

#include <csignal>
#include <sched.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// FSM & 控制组件（链接自 libUnitreeArm_x86_64.so）
#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Init.h"
#include "FSM/State_JointCtrl.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_Passive.h"
#include "FSM/State_LowCmd.h"
#include "FSM/State_ClearError.h"
#include "IOInterface/IOUDP/IOUDP.h"
#include "cmdInterface/cmdSdk.h"
#include "cmdInterface/CtrlComponents.h"
#include "model/manipulator.h"

// ROS2 仿真后端
#include "z1_controller_ros2/io_ros2.hpp"

// pinocchio URDF 解析
#include "pinocchio/parsers/urdf.hpp"

namespace pin = pinocchio;

static bool running = true;

int main(int argc, char **argv)
{
  std::cout << std::fixed << std::setprecision(4);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("z1_ctrl");

  // ── 参数声明（与 ROS1 config.yaml / real_ctrl.launch 参数名完全一致）──
  node->declare_parameter<std::string>("udp.mcu_ip",     "192.168.123.110");
  node->declare_parameter<int>        ("udp.port_to_mcu", 8881);
  node->declare_parameter<int>        ("udp.port_to_sdk", 8871);
  node->declare_parameter<bool>       ("collision.open",  false);
  node->declare_parameter<double>     ("collision.limitT", 10.0);
  node->declare_parameter<std::string>("communication",   "UDP");
  node->declare_parameter<bool>       ("cmdCache",        false);

  std::string mcu_ip;
  int port_to_mcu, port_to_sdk;
  bool collision_open, cmd_cache;
  double collision_limit_t;
  std::string communication;

  node->get_parameter("udp.mcu_ip",     mcu_ip);
  node->get_parameter("udp.port_to_mcu", port_to_mcu);
  node->get_parameter("udp.port_to_sdk", port_to_sdk);
  node->get_parameter("collision.open",  collision_open);
  node->get_parameter("collision.limitT", collision_limit_t);
  node->get_parameter("communication",   communication);
  node->get_parameter("cmdCache",        cmd_cache);

  collision_limit_t = std::clamp(collision_limit_t, 5.0, 30.0);

  RCLCPP_INFO(node->get_logger(),
    "z1_ctrl starting: mcu=%s:%d  sdk_port=%d  comm=%s  collision=%s(%.1f Nm)",
    mcu_ip.c_str(), port_to_mcu, port_to_sdk,
    communication.c_str(),
    collision_open ? "ON" : "OFF", collision_limit_t);

  // ── CtrlComponents 组装（与 ROS1 main.cpp 完全相同）──────────────────────
  auto ctrlComp = std::make_shared<CtrlComponents>();
  ctrlComp->collisionOpen   = collision_open;
  ctrlComp->collisionLimitT = collision_limit_t;

  // 从包的 share 目录加载 z1.urdf（替代 ros::package::getPath）
  std::string pkg_share =
    ament_index_cpp::get_package_share_directory("z1_controller_ros2");
  ctrlComp->projectPath = pkg_share;

  pin::Model model;
  pin::urdf::buildModel(pkg_share + "/config/z1.urdf", model);
  ctrlComp->arm = std::make_shared<ArmModel>(model);

  // SDK 接收端（监听 port_to_sdk，接受 Z1System 的 ArmCmd）
  ctrlComp->cmdSdk = std::make_shared<UdpSdk>(
    port_to_sdk, ctrlComp->lowState, cmd_cache);

  // IO 后端：真机 UDP 或 ROS2 仿真
  if (communication == "UDP") {
    ctrlComp->ioInter = std::make_shared<IOUDP>(mcu_ip.c_str(), port_to_mcu);
  } else if (communication == "ROS") {
    ctrlComp->ioInter = std::make_shared<IOROS2>(node);
  } else {
    RCLCPP_WARN(node->get_logger(),
      "Unknown communication '%s', defaulting to UDP.", communication.c_str());
    ctrlComp->ioInter = std::make_shared<IOUDP>(mcu_ip.c_str(), port_to_mcu);
  }

  // ── 等待硬件连接（与 ROS1 完全相同）─────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "Waiting for connection with Arm...");
  signal(SIGINT, [](int) {
    running = false;
    RCLCPP_INFO(rclcpp::get_logger("z1_ctrl"), "Shutting down z1_ctrl.");
  });

  while (running) {
    if (ctrlComp->ioInter->init()) {
      RCLCPP_INFO(node->get_logger(),
        "Connected with Arm. Gripper: %s",
        ctrlComp->ioInter->hasGripper ? "present" : "absent");
      break;
    }
    usleep(100000);
  }

  if (!ctrlComp->ioInter->initialized()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to Arm. Exiting.");
    rclcpp::shutdown();
    return -1;
  }

  // ── FSM 构建（8 态，与 ROS1 完全相同）───────────────────────────────────
  std::string csv_path = pkg_share + "/config";
  std::vector<std::shared_ptr<FSMState>> states;
  states.push_back(std::make_shared<State_Init>(ctrlComp));
  states.push_back(std::make_shared<State_Passive>(ctrlComp));
  states.push_back(std::make_shared<State_JointCtrl>(ctrlComp));
  states.push_back(std::make_shared<State_Teach>(ctrlComp, csv_path));
  states.push_back(std::make_shared<State_TeachRepeat>(ctrlComp, csv_path));
  states.push_back(std::make_shared<State_Calibration>(ctrlComp));
  states.push_back(std::make_shared<State_LowCmd>(ctrlComp));
  states.push_back(std::make_shared<State_ClearError>(ctrlComp));

  // FiniteStateMachine 构造函数内部启动 250 Hz 控制线程
  auto fsm = std::make_shared<FiniteStateMachine>(states, ctrlComp);

  RCLCPP_INFO(node->get_logger(), "z1_ctrl FSM running at 250 Hz.");

  // 主线程保活（FSM 运行在独立线程中）
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (running && rclcpp::ok()) {
    exec.spin_some();
    usleep(10000);  // 10 ms
  }

  rclcpp::shutdown();
  return 0;
}
