# Z1 机械臂 SDK

双 Unitree Z1 机械臂的控制 SDK，提供独立 C++ 控制内核、ROS 1 Topic 接口和 MCU UDP 通信。支持关节目标、末端位姿目标、定时轨迹、连续过点、末端直线路径、流式跟踪、笛卡尔阻抗及夹爪控制。

## 启动

构建并加载 ROS 1 工作区后，启动双臂：

```bash
source devel/setup.bash
roslaunch z1_bringup arm.launch
```

左臂命名空间为 `/left_arm`，右臂为 `/right_arm`。两臂各自连接 MCU，启动后处于 Disabled；有效且静止时，首条有限运动目标会从实测位置直接开始执行，无需先发 ENABLE。也可单独 ENABLE 进入 Holding。

## 配置

| 文件 | 内容 |
| --- | --- |
| [communication.yaml](src/z1_ros/z1_bringup/config/communication.yaml) | 各臂 IP、本地 UDP 端口、接收预算 |
| [control.yaml](src/z1_ros/z1_bringup/config/control.yaml) | 控制周期、速度/加速度/jerk、力矩、增益、容差及几何诊断配置 |
| [calibration.yaml](src/z1_ros/z1_control/config/calibration.yaml) | 标定对象、静止检查、回执超时和零位验证容差 |
| [z1_control.urdf](src/z1_ros/z1_description/urdf/z1_control.urdf) | 六轴运动学、质量/惯性、工具坐标系及碰撞几何 |

默认左臂 IP 为 `192.168.123.110`、本地端口 `8881`；右臂 IP 为 `192.168.123.111`、本地端口 `8882`。修改通信配置后仍使用同一启动命令。

## 零点标定

停止双臂控制器并将机械臂摆到机械零位后，运行 `rosrun z1_control z1_calibrate`，按提示分别确认左臂和右臂。程序调用官方标定接口并验证六轴反馈，参数从 YAML 读取。完整步骤及无需 ROS 的独立构建方式见 [双臂零点标定](src/z1_ros/CALIBRATION.md)。

## 接口

每臂通过自身命名空间下的 Topic 接收命令、发布反馈。关节角单位为 rad，位置为 m；末端位姿分别相对 `left_link00` 或 `right_link00`。默认 TCP 为 `link06` 沿 X 正方向平移 0.051 m 的固定工具坐标系。

| Topic | 用途 |
| --- | --- |
| `control` | 使能、停止、恢复、失能及模式选择 |
| `joint_goal` / `pose_goal` | 有限关节目标 / 末端位姿目标 |
| `joint_trajectory` / `joint_waypoints` / `linear_path` | 定时轨迹、连续过点、末端直线路径 |
| `joint_stream` / `pose_stream` | 连续更新关节或末端目标 |
| `joint_reference` / `pose_reference` | 带时间和可选导数的连续参考 |
| `gripper_goal` / `impedance_gains` | 夹爪位置目标 / 笛卡尔阻抗增益 |
| `joint_states` / `tcp_pose` / `status` / `result` | 关节、末端、控制状态及命令结果 |

完整控制流程、消息字段与示例见 [SDK 使用说明](src/z1_ros/README.md)和[接口契约](src/z1_ros/INTERFACES.md)。
