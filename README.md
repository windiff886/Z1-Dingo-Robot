# Z1-Dingo 双臂机器人

ROS 1 Noetic 工作区，包含 Dingo-O 全向底盘、双 Unitree Z1 机械臂、Pan-Tilt 云台、D455 相机，以及整机模型。Z1 控制 SDK 提供关节运动、末端位姿、定时轨迹、连续过点、直线路径、流式跟踪、笛卡尔阻抗和夹爪控制。

## 构建与启动

安装各组件的依赖后，在工作区根目录构建：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

按需要选择入口：

| 要做的事 | 命令 | 说明 |
| --- | --- | --- |
| 控制双臂 | `roslaunch z1_bringup arm.launch` | 启动左右臂控制器，连接真实 MCU |
| 在 RViz 中控制双臂 TCP | `roslaunch unidingo_rviz_test rviz_tcp_test.launch` | 自带双臂控制器和整机显示，与上一入口二选一 |
| 运行底盘 | `roslaunch dingo_control dingo.launch` | 提前建立 CAN/UDP 桥接 |
| 运行云台并查看模型 | `roslaunch pan_tilt_bringup panTilt_view.launch` | 连接串口，启动时以 10 deg/s 回中 |
| 采集彩色与深度图 | `roslaunch d455_camera d455_camera_nodelet.launch` | 连接 D455 |

双臂启动后处于 `Disabled`，交换被动帧，不提供主动姿态保持。反馈有效且机械臂静止时，第一条有限运动目标会从实测位置开始执行；单独发送 `ENABLE` 则进入当前位置保持。两臂分别使用 `/left_arm`、`/right_arm` 命名空间，独立执行命令。

## 配置

| 文件 | 内容 |
| --- | --- |
| [communication.yaml](src/z1_ros/z1_bringup/config/communication.yaml) | 各臂 IP、本地 UDP 端口、接收预算 |
| [control.yaml](src/z1_ros/z1_bringup/config/control.yaml) | 控制周期、速度/加速度/jerk、力矩、增益及容差 |
| [calibration.yaml](src/z1_ros/z1_control/config/calibration.yaml) | 标定对象、静止检查、回执超时和零位验证容差 |
| [z1_control.urdf](src/z1_ros/z1_description/urdf/z1_control.urdf) | 六轴运动学、质量/惯性、工具坐标系；保留供规划使用的碰撞描述 |

默认左臂 IP 为 `192.168.123.110`、本地端口 `8881`；右臂 IP 为 `192.168.123.111`、本地端口 `8882`。ROS 控制周期为 4 ms（目标 250 Hz），接收预算为 3 ms。YAML 在启动时读取，修改后重启对应节点。

## 零点标定

停止双臂控制器，把机械臂摆到机械零位，然后运行：

```bash
rosrun z1_control z1_calibrate
```

程序按提示逐臂确认，调用官方标定接口，再检查六轴反馈。操作步骤、结果含义和独立构建方法见[双臂零点标定](src/z1_ros/CALIBRATION.md)。

## 接口

每臂在自身命名空间下接收命令、发布反馈。关节角单位为 rad，位置单位为 m，末端位姿相对 `left_link00` 或 `right_link00`。默认 TCP 为 `link06` 沿 X 正方向平移 0.051 m 的固定工具坐标系。

| Topic | 用途 |
| --- | --- |
| `control` | 使能、停止、恢复、失能及模式选择 |
| `joint_goal` / `pose_goal` | 有限关节目标 / 末端位姿目标 |
| `joint_trajectory` / `joint_waypoints` / `linear_path` | 定时轨迹、连续过点、末端直线路径 |
| `joint_stream` / `pose_stream` | 连续更新关节或末端目标 |
| `joint_reference` / `pose_reference` | 带时间和可选导数的连续参考 |
| `gripper_goal` / `impedance_gains` | 夹爪位置目标 / 笛卡尔阻抗增益 |
| `joint_states` / `tcp_pose` / `status` / `result` | 关节反馈、末端位姿、控制状态及命令结果 |

消息字段和发布示例见 [Topic 接口说明](src/z1_ros/INTERFACES.md)，参考生成和力矩计算见 [SDK 算法详解](src/z1_ros/SDK_ALGORITHMS.md)。

## 各组件说明

- [Z1 双臂 SDK](src/z1_ros/README.md)：启动、配置、控制流程和原生 C++ 构建。
- [整机 RViz 测试](src/z1_ros/unidingo_rviz_test/README.md)：官方 MoveIt 目标编辑、SDK 轨迹预览、Home 位姿预设和状态显示。
- [Dingo-O 底盘](src/dingo/README.md)与[键盘控制](src/dingo/dingo-test/README.md)。
- [Pan-Tilt 云台](src/pan_tilt_ros/README.md)。
- [D455 相机](src/d455_camera/README.md)。
- [整机模型](src/unidingo_description/README.md)：装配结构、坐标系和模型依赖。
