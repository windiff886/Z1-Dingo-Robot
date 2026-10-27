# Z1 双臂 SDK 启动说明

本目录提供 ROS 1 下双 Unitree Z1 机械臂的 SDK，包括模型解算、运动生成、关节力矩控制、MCU 通信和状态反馈。每臂控制六个主关节，并在设备配有夹爪时提供夹爪控制。

## 唯一启动入口

机械臂的唯一启动入口是 [z1_bringup/launch/arm.launch](z1_bringup/launch/arm.launch)：

```bash
roslaunch z1_bringup arm.launch
```

启动前，安装包清单中的依赖，构建 ROS 1 工作空间并加载 `devel/setup.bash`，同时建立主机到两臂 MCU 的网络连接。

两臂 IP、本地 UDP 端口、接收预算和单臂模型发布开关统一写在 [communication.yaml](z1_bringup/config/communication.yaml)。默认左臂 IP 为 `192.168.123.110`、本地端口 `8881`；右臂 IP 为 `192.168.123.111`、本地端口 `8882`。MCU 目标端口为官方通信库使用的 `8880`。

## 零点标定

停止双臂控制器并将机械臂摆到机械零位后，执行 `rosrun z1_control z1_calibrate`。独立 C++ 工具读取 [calibration.yaml](z1_control/config/calibration.yaml)，复用通信配置，逐臂要求确认、调用官方标定接口并验证新的六轴零位反馈。默认标定左右两臂，也可在 YAML 中选择单臂。操作流程、结果和独立构建方式见 [双臂零点标定](CALIBRATION.md)。

## 这个入口启动什么

### 1. 加载双臂运行配置

读取 [通信配置](z1_bringup/config/communication.yaml)，分别设置 `/left_arm/controller` 和 `/right_arm/controller` 的连接参数。两个控制节点各自加载同一份 [控制配置](z1_bringup/config/control.yaml)，包括控制周期、反馈超时、速度/加速度/jerk、力矩、控制增益、到达容差和碰撞检查参数。

左臂命名空间为 `/left_arm`，关节及 TF 前缀为 `left_`；右臂命名空间为 `/right_arm`，前缀为 `right_`。

### 2. 加载两臂模型

独立启动时，只有将 `communication.yaml` 中的 `publish_description` 设为 `true`，才通过 Xacro 展开 [Z1 描述模型](z1_description/urdf/z1.urdf.xacro)，分别写入 `/left_arm/robot_description` 和 `/right_arm/robot_description`，提供带各自前缀的连杆、关节和工具坐标系。默认关闭，使用整机显示入口时由整机提供描述模型。

无论该开关如何设置，两个控制节点始终读取 [控制 URDF](z1_description/urdf/z1_control.urdf)，使用其中的运动学、关节限位、质量、惯性和碰撞几何，完成末端位姿计算、逆运动学、动力学和距离检查。

末端目标分别相对 `left_link00` 和 `right_link00` 表达。默认 TCP 为 `tool`，位于 `link06` 沿 X 正方向 0.051 m，姿态相同；描述模型中对应 `left_tool` 和 `right_tool`。

### 3. 启动双臂控制器与 MCU 通信

启动 `z1_control` 的两个 `z1_topic_node` 进程：

- **`/left_arm/controller`**：接收左臂命令，连接左臂 MCU，执行控制并发布左臂反馈。
- **`/right_arm/controller`**：接收右臂命令，连接右臂 MCU，执行控制并发布右臂反馈。

每个进程内部包含 ROS Topic 适配、独立控制内核和 MCU 通信适配。通信适配通过官方 IOUDP 完成握手、命令发送及关节反馈解码；控制内核根据运动目标生成参考，并结合关节反馈计算输出力矩。

原生控制周期默认 2 ms（500 Hz）。每轮发送上一轮输出、接收 MCU 反馈，再更新控制状态和下一轮输出。主臂的位置反馈控制及动力学补偿在主机完成，主臂发送的 MCU Kp/Kd 为零；夹爪使用独立的位置增益。

### 4. 发布两臂关节变换

独立启动 `arm.launch` 时，由 [communication.yaml](z1_bringup/config/communication.yaml) 中的 `publish_description` 开关控制（默认 `false`，使用 YAML 布尔值）：

- 设为 `false` 时（默认）：不加载单臂模型也不发布单臂 TF，由整机启动层统一提供模型和 TF。
- 设为 `true` 时：启动两个 `robot_state_publisher` 分别读取两臂描述和 `joint_states`，发布单臂 TF。

启动 `roslaunch unidingo_rviz_test rviz_tcp_test.launch` 时，始终由整机入口统一发布模型和 TF，不启动单臂发布器；此入口不受上述独立发布开关影响。

### 5. 提供运动命令和状态反馈

两个控制节点在各自命名空间下提供相同的 Topic 接口：

| Topic | 内容 |
| --- | --- |
| `control` | 使能、失能、停止、恢复和模式选择 |
| `joint_goal` / `pose_goal` | 六轴关节目标 / TCP 目标位姿 |
| `joint_trajectory` / `joint_waypoints` / `linear_path` | 定时关节轨迹、连续关节过点和末端直线路径 |
| `joint_stream` / `pose_stream` | 持续更新的关节或末端目标 |
| `joint_reference` / `pose_reference` | 带目标时间及可选速度、加速度的连续参考 |
| `gripper_goal` / `impedance_gains` | 夹爪位置目标 / 笛卡尔阻抗增益 |
| `joint_states` / `tcp_pose` | 关节反馈 / 由有效关节反馈计算的 TCP 位姿 |
| `status` / `result` | 控制状态、反馈时龄和周期统计 / 命令接收及执行结果 |

启动后，两臂分别处于 Disabled。向对应的 `control` Topic 发送 ENABLE 后，控制器以当前关节反馈建立保持参考，进入 Holding，再接收运动目标。两臂分别管理使能、运动和停止状态。

有限关节目标使用 Ruckig 生成关节轨迹；末端位姿目标先求逆运动学，再生成关节轨迹；末端直线路径通过笛卡尔插值和连续逆运动学生成。持续目标使用流式跟踪，笛卡尔阻抗以目标位姿为平衡参考。命令字段、时间戳、坐标系和操作码见 [接口契约](INTERFACES.md)。

因此，机械臂运行的主要逻辑是：上层目标经运动生成转为关节参考，控制器结合 MCU 反馈和动力学模型计算力矩，再发送到各臂执行；反馈同时用于更新 TF、控制状态和到达结果。


