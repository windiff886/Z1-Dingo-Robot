# Dingo 底盘组件

面向 Dingo-O 全向底盘，使用 ROS 1 Noetic。组件接收速度命令、读取底盘反馈，并发布融合后的运动状态。

## 1. 当前功能

| 功能 | 文件位置 | 说明 |
| --- | --- | --- |
| 底盘驱动与控制 | `dingo_control/launch/`、`config/control_*.yaml` | 调用官方驱动，配置轮子控制器；支持全向和差速两种配置，默认全向 |
| 通信工具 | `dingo_control/scripts/can-udp-bridge.sh`、`deployment/` | CAN/UDP 桥接脚本及 systemd 服务模板；默认连接 `192.168.131.2:11412`，提供 `vcan0` 接口 |
| EKF 滤波融合 | `dingo_control/config/robot_localization.yaml` | 融合轮式里程计的前向速度、侧向速度和旋转角速度，以及 IMU 的旋转角速度 |
| 反馈监测 | `dingo_control/scripts/feedback_monitor.py` | 检查输入和输出的时效、数值、协方差与坐标变换，发布诊断 |
| 底盘描述 | `dingo_description/urdf/`、`meshes/` | 全向/差速底盘的几何、轮子关节和挂载点 |

实际驱动 `dingo_base`、消息包 `dingo_msgs` 和控制器插件由运行环境提供。本目录不包含完整的底层通信协议实现，也不提供遥操作、导航或整机仿真。

当前使用 `robot_localization` 的 EKF，不启动 Madgwick IMU 姿态滤波，也不融合 IMU 姿态和线加速度。

## 2. Topic 接口

方向以本组件为准；下表为默认名称。

| Topic | 方向 | 消息类型 | 用途 |
| --- | --- | --- | --- |
| `/dingo/cmd_vel` | 输入 | `geometry_msgs/Twist` | 底盘速度命令 |
| `/dingo/odom` | 输出 | `nav_msgs/Odometry` | EKF 融合后的主要运动状态，配置频率 50 Hz |
| `/dingo/odom_raw` | 输出 | `nav_msgs/Odometry` | 未经本地 EKF 的轮式里程计，同时作为 EKF 输入 |
| `/dingo/imu/data_raw` | 输出 | `sensor_msgs/Imu` | 底盘原始 IMU 数据，同时作为 EKF 输入 |
| `/joint_states` | 输出 | `sensor_msgs/JointState` | 轮子关节反馈，配置频率 200 Hz |
| `/diagnostics` | 输出 | `diagnostic_msgs/DiagnosticArray` | EKF 及反馈有效性诊断，反馈监测频率 10 Hz |

命令使用 `linear.x`（前向，m/s）、`linear.y`（侧向，m/s）和 `angular.z`（旋转，rad/s）。全向底盘支持侧移；控制器的命令超时设为 0.25 秒。

主要反馈从 `/dingo/odom` 读取：

```text
twist.twist.linear.x    前向速度，m/s
twist.twist.linear.y    侧向速度，m/s
twist.twist.angular.z   旋转角速度，rad/s
```

其 `header.frame_id` 为 `odom`，`child_frame_id` 为 `base_link`：位置和姿态在 `odom` 坐标系下，速度在 `base_link` 坐标系下。位置与航向是局部估计，会随时间漂移。

输入中断时 EKF 仍可能持续发布预测，因此需同时检查 `/diagnostics` 中的 `dingo/feedback`：

| 状态 | 含义 |
| --- | --- |
| `ready` | 轮式输入、IMU 输入和融合输出均通过监测 |
| `wheel_only` | 融合输出通过监测，仅轮式输入通过监测 |
| `imu_only` | 融合输出通过监测，仅 IMU 输入通过监测，线速度缺少有效轮式测量 |
| `prediction_only` | 输出仍在发布，两路输入均未通过监测 |
| `unavailable` | 融合输出未收到、过期或未通过检查 |

各路超时阈值默认 0.2 秒。监测只报告状态，不修改 EKF 输出；使用方也需检查诊断消息本身是否过期。硬件状态消息类型为 `dingo_msgs/Status`，实际 topic 名称以目标机驱动为准。

## 3. Launch 使用方法

运行环境需安装官方驱动、控制器及 `robot_localization` 等依赖，并加载已构建工作空间的 `setup.bash`。使用 CAN/UDP 桥接的真机需先启动桥接服务；ROS launch 不会自动启动该服务。

```bash
# 启动全向底盘：驱动、控制器、模型加载、IMU 静态 TF、EKF 和反馈监测
roslaunch dingo_control dingo.launch dingo_omni:=1

# 需要里程计 TF 时，由 EKF 发布 odom -> base_link
roslaunch dingo_control dingo.launch dingo_omni:=1 publish_odom_tf:=true

# 自定义命令和融合反馈 topic
roslaunch dingo_control dingo.launch dingo_omni:=1 \
  cmd_vel_topic:=/my_robot/cmd_vel odom_topic:=/my_robot/odom
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `dingo_omni` | `DINGO_OMNI` 环境变量，未设置时为 `1` | `1` 全向，`0` 差速；本机应使用 `1` |
| `motor_params` | `DINGO_MOTOR_PARAMS` 环境变量，未设置时为 `base` | `config/hardware/` 中的配置名；`base` 使用驱动默认参数 |
| `cmd_vel_topic`、`odom_topic`、`odom_raw_topic`、`imu_raw_topic`、`diagnostics_topic` | 上表中的默认名称 | 修改对应 topic，内部融合和监测同步使用新名称 |
| `ekf_config` | 本包 `config/robot_localization.yaml` | EKF 参数文件 |
| `wheel_timeout`、`imu_timeout`、`odom_timeout` | 各 `0.2` 秒 | 各路反馈监测超时阈值 |
| `load_description` | `true` | 加载 `robot_description`；外部已提供兼容模型时设为 `false` |
| `publish_odom_tf` | `false` | 是否由 EKF 发布 `odom -> base_link` |
| `publish_imu_tf` | `true` | 是否发布 `base_link -> imu_link` 静态变换；外部已提供时设为 `false` |
| `imu_frame` | `imu_link` | 与原始 IMU 的 `header.frame_id` 一致 |
| `imu_x`、`imu_y`、`imu_z` | 各 `0` 米 | IMU 相对 `base_link` 的安装位置 |
| `imu_roll`、`imu_pitch`、`imu_yaw` | 各 `0` 弧度 | IMU 相对 `base_link` 的安装角度 |

IMU 安装变换需按实机核实；若传感器轴与底盘轴不对齐，也需调整 EKF 的轴选择。模型加载仅设置描述参数，不启动 `robot_state_publisher`。

仅加载底盘描述时使用：

```bash
roslaunch dingo_description description.launch dingo_omni:=1
```
