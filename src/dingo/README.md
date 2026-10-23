# Dingo-O 底盘启动说明

本目录提供 ROS 1 Noetic 下 Dingo-O 四轮全向底盘的驱动、控制、模型和里程计融合功能。

## 唯一启动入口

底盘的唯一启动入口是 [dingo_control/launch/dingo.launch](dingo_control/launch/dingo.launch)：

```bash
roslaunch dingo_control dingo.launch
```

启动前，安装包清单中的依赖，构建工作空间并加载 `devel/setup.bash`，同时完成主机与 MCU 的网络连接和 CAN/UDP 桥接。主机地址为 `192.168.131.1`，MCU 地址为 `192.168.131.2`；桥接服务建立 `vcan0`，供底盘驱动访问电机。

桥接由系统服务管理，部署文件见 [can-udp-bridge.service](dingo_control/deployment/can-udp-bridge.service) 和 [can-udp-bridge.sh](dingo_control/scripts/can-udp-bridge.sh)。`dingo.launch` 启动 ROS 底盘组件，桥接服务需事先运行。

## 这个入口启动什么

### 1. 加载底盘模型

通过 Xacro 生成 [Dingo URDF](dingo_description/urdf/dingo.urdf.xacro)，写入 `robot_description`，提供底盘结构、四轮关节和安装坐标系的定义。

这里只加载模型参数，`robot_state_publisher` 由整机启动层启动。整机已经加载完整模型时，设置 `load_description:=false`。

### 2. 启动底盘驱动和 MCU 通信

启动 `dingo_base` 的 `dingo_node`，并加载 [电机参数](dingo_control/config/hardware/base.yaml)。驱动通过 `vcan0` 配置和控制四个 Puma 电机，读取车轮反馈；通过 rosserial 接收 MCU 提供的 IMU 与底盘状态，同时接收 MCU 日志。

入口还启动 rosserial 消息信息服务，供 MCU 通信使用。灯光控制、风扇控制、底盘及电机状态诊断也在 `dingo_node` 中运行。

### 3. 加载四轮控制器

入口内部包含 [control.launch](dingo_control/launch/control.launch)，读取 [控制器配置](dingo_control/config/control_omni.yaml)，由控制器加载器将两个控制器加载到 `dingo_node` 内的 `controller_manager`：

- **关节状态控制器**：发布四个车轮的关节状态。
- **麦克纳姆速度控制器**：把底盘的前向、侧向和旋转速度指令换算为四轮速度，并根据车轮反馈计算轮式里程计。

驱动以 25 Hz 读取车轮状态、更新控制器并发送电机命令。`control.launch` 是内部使用的子启动文件，日常启动只使用 `dingo.launch`。

### 4. 发布 IMU 安装变换

默认发布 `base_link → imu_link` 静态变换，描述 IMU 相对底盘的位置和朝向。默认位置和角度均为零，可通过 `imu_x/y/z`、`imu_roll/pitch/yaw` 参数设置。

整机已经发布该变换时，设置 `publish_imu_tf:=false`。

### 5. 融合底盘运动状态

启动 `robot_localization` 的 EKF，加载 [融合配置](dingo_control/config/robot_localization.yaml)，在二维平面内融合：

- 轮式里程计的前向速度、侧向速度和偏航角速度。
- 底盘 IMU 的偏航角速度。

EKF 以 50 Hz 输出融合后的底盘位姿与速度。默认不发布 `odom → base_link` 变换；需要由底盘 EKF 提供这一变换时，在同一个启动命令后加上 `publish_odom_tf:=true`。

因此，底盘运行的主要逻辑是：速度指令经麦克纳姆控制器转为四轮电机命令；车轮反馈形成轮式里程计，再与 MCU 的 IMU 角速度融合，得到底盘运动状态。

## 依赖与源码

`dingo_base` 提供硬件驱动，`dingo_control` 组织启动与配置，`dingo_description` 提供模型，`dingo_msgs` 提供 MCU 状态、风扇和灯光消息。Puma 电机驱动及其消息包、麦克纳姆控制器插件和 ROS 通用组件由运行环境安装提供。

`dingo_msgs` 版本为 `0.3.1`，引自官方 [dingo 仓库](https://github.com/dingo-cpr/dingo/tree/68806fdffabfd612a3414b7e8d6cb19c496f8a0e/dingo_msgs)，固定提交 `68806fdffabfd612a3414b7e8d6cb19c496f8a0e`。消息定义与构建文件保持上游原样，许可证见 [LICENSE](dingo_msgs/LICENSE)。
