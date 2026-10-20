# Z1 机械臂组件

每臂一个独立进程，以 ROS 1 topics 接收命令和发布反馈。

## 功能

| 目录 | 功能 |
| --- | --- |
| `z1_control` | 独立 C++ 内核：Pinocchio 运动学/动力学、Ruckig 轨迹、Coal 碰撞检查、位置跟踪、两类流式跟踪、笛卡尔阻抗及停止/恢复；ROS 1 薄接口 |
| `z1_msgs` | 关节/位姿命令、流式会话、执行结果和状态消息 |
| `z1_bringup` | 单臂和双臂 launch、参数配置 |
| `z1_description` | 六轴模型、工具包络及外观网格 |
| `vendor/unitree` | 保留的官方 MCU 通信头文件和动态库；无旧 FSM/SDK 中转运行链路 |

无需力传感器或外力估计。末端有限目标通过 IK 执行关节轨迹；不承诺直线或绕障。仿真为 Pinocchio 动力学后端，未模拟接触。

## Topics

以下均相对单臂命名空间，例如 `/left_arm/joint_goal`。

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `control` | `z1_msgs/ControlCommand` | 使能、失能、停止、恢复、保持及模式选择 |
| `joint_goal` / `pose_goal` | `z1_msgs/JointGoal` / `PoseGoal` | 有限关节目标 / 基座系 TCP 位姿 |
| `joint_stream` / `pose_stream` | `z1_msgs/JointStream` / `PoseStream` | 连续目标；需当前 session_id 和递增 sequence |
| `impedance_gains` | `z1_msgs/ImpedanceGains` | 保持时设置刚度/阻尼 |
| `gripper_goal` | `z1_msgs/GripperGoal` | 夹爪 [-1,0] rad 位置目标（仅存在时可用） |
| `joint_states` | `sensor_msgs/JointState` | 实测 q/dq/tau，不是目标值 |
| `tcp_pose` | `geometry_msgs/PoseStamped` | 从有效测量计算的 TCP 位姿 |
| `status` / `result` | `z1_msgs/Status` / `MotionResult` | 测量时龄、模式、会话、故障 / 命令结果 |

默认失能。先发布 ENABLE；有限目标在 Holding 状态接收，忙时拒绝。流式先选择对应模式，使用 `status.session_id` 发新目标；超时后 STOPPING → AWAIT_RESUME，需 RESUME 和新会话目标。接口细节、示例见 [INTERFACES.md](INTERFACES.md)。

## Launch

在 ROS 1 工作区构建并 source 后：

```bash
# 单臂离线仿真
roslaunch z1_bringup arm.launch

# 两臂独立仿真进程：/left_arm、/right_arm，关节/TF 前缀 left_、right_
roslaunch z1_bringup dual_arm.launch

# 真机：完成固件、力矩控制和模型/参数确认后才启用
roslaunch z1_bringup arm.launch backend:=udp hardware_validated:=true \
  mcu_ip:=192.168.123.110 local_port:=8881
```

可传 `namespace`、`prefix`、`config`、`model_file`、`description_file`。双臂可分别传 `left_config/right_config`、`left_model_file/right_model_file`、`left_description_file/right_description_file`、`left_ip/right_ip` 和 `left_port/right_port`；默认本地端口为 8881/8882，官方 MCU 目标端口为 8880。参数见 [control.yaml](z1_bringup/config/control.yaml)。固定支架模型默认未填写，需要按安装实测配置。

原 MoveIt、ros_control、Gazebo 控制入口已移除。整机描述网格和常量仍保留；其他组件中引用旧机械臂启动方式的入口需在整机重构时改接上述 launch。

## 独立构建

依赖 C++17、CMake、Eigen、Pinocchio 4.x/Coal、Ruckig ≥0.9.2、Boost、OpenSSL；ROS 层另需 ROS 1 roscpp、消息生成及 xacro/robot_state_publisher。

```bash
cmake -S src/z1_ros/z1_control -B /tmp/z1-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/native/dependencies
cmake --build /tmp/z1-build -j1
ctest --test-dir /tmp/z1-build --output-on-failure
/tmp/z1-build/z1_standalone src/z1_ros/z1_description/urdf/z1_control.urdf 6 --demo
```

Standalone 只运行仿真，`--demo` 显式使能并执行小幅关节运动。ROS 1 构建使用 catkin；单独 CMake 可指定 `-DZ1_BUILD_ROS=ON`。测试方法与现场验收项见 [实施记录](IMPLEMENTATION_REPORT.md)，需求见 [REFACTOR_REQUIREMENTS.md](REFACTOR_REQUIREMENTS.md)。
