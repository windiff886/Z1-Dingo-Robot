# dingo_ros2

Clearpath Robotics Dingo 全向移动底盘的 ROS2 驱动栈（fork 自 Clearpath 官方仓库），为 Uni-Dingo 机器人提供底盘硬件接口、URDF 模型、控制器配置、CAN 总线通信和平台消息定义。包含 **5 个顶层子仓库**、**20+ 个子包**。

> **注意**：本目录是对 Clearpath 官方 ROS2 仓库的定制化 fork，适配了 Dingo-O (DD150) 全向底盘在 Uni-Dingo 项目中的使用需求。在 Uni-Dingo 组合模型中，底盘 URDF 由 `unidingo_description_ros2` 统一组装调用，不直接使用本包的 launch。

## 架构概览

```
dingo_ros2/
├── clearpath_common            # 通用层：URDF 模型、控制器、遥控、定位
│   ├── clearpath_common               # 元包
│   ├── clearpath_platform_description  # 各平台 URDF（DD100/DD150/J100/W200）
│   ├── clearpath_description           # URDF 元包
│   ├── clearpath_mounts_description    # 扩展板/安装件 URDF
│   ├── clearpath_sensors_description   # 传感器 URDF
│   ├── clearpath_manipulators_description  # 机械臂挂载 URDF
│   ├── clearpath_manipulators          # 机械臂 MoveIt 配置
│   ├── clearpath_control               # ros2_control 控制器配置与 launch
│   ├── clearpath_generator_common      # 代码/配置自动生成器
│   └── clearpath_customization         # 自定义项目模板
│       ├── project_bringup             # 自定义项目启动
│       └── project_description         # 自定义项目模型
│
├── clearpath_config            # YAML 配置解析库（Python）
│
├── clearpath_msgs              # 平台消息定义
│   ├── clearpath_msgs                  # 元包
│   ├── clearpath_platform_msgs         # 平台状态/反馈/电源消息
│   └── clearpath_motor_msgs            # Puma 电机驱动消息
│
├── clearpath_robot             # 真机驱动层
│   ├── clearpath_robot                 # 元包
│   ├── clearpath_hardware_interfaces   # ros2_control 硬件接口插件
│   ├── clearpath_motor_drivers         # Puma 电机 CAN 驱动
│   │   └── puma_motor_driver           # CAN 协议实现
│   ├── clearpath_sensors               # 传感器 launch 集合
│   ├── clearpath_generator_robot       # 真机配置生成器
│   └── clearpath_diagnostics           # 诊断工具
│
└── clearpath_ros2_socketcan_interface  # SocketCAN ROS2 桥接
```

### 控制栈层次

```
        ┌─────────────────────────────────────────────┐
        │            uni_dingo_api / BaseAPI           │ ◄─ 应用层 (cmd_vel)
        └──────────────────┬──────────────────────────┘
                           │ /cmd_vel
                           ▼
        ┌─────────────────────────────────────────────┐
        │         platform_velocity_controller        │ ◄─ diff_drive_controller
        │         (clearpath_control)                 │    或 omni_drive_controller
        └──────────────────┬──────────────────────────┘
                           │ wheel velocity commands
                           ▼
        ┌─────────────────────────────────────────────┐
        │         ros2_control Manager                │
        │  ┌─────────────────────────────────────┐    │
        │  │ PumaHardware (SystemInterface)      │    │ ◄─ clearpath_hardware_interfaces
        │  │ ─ read():  updateJointsFromHardware │    │
        │  │ ─ write(): writeCommandsToHardware  │    │
        │  └──────────────┬──────────────────────┘    │
        └─────────────────┼───────────────────────────┘
                          │ ROS2 话题 (platform/puma/cmd, feedback)
                          ▼
        ┌─────────────────────────────────────────────┐
        │         puma_motor_driver (CAN 节点)         │ ◄─ clearpath_motor_drivers
        └──────────────────┬──────────────────────────┘
                           │ SocketCAN (vcan0/can0)
                           ▼
        ┌─────────────────────────────────────────────┐
        │         clearpath_ros2_socketcan_interface   │ ◄─ CAN 帧收发
        └──────────────────┬──────────────────────────┘
                           │ CAN Bus
                           ▼
        ┌─────────────────────────────────────────────┐
        │         Puma Motor Controller (硬件)         │ ◄─ 电机驱动板
        └─────────────────────────────────────────────┘
```

## 核心子包详解

### clearpath_platform_description — 平台 URDF 模型

为 Clearpath 各款底盘提供 URDF/Xacro 模型文件。

**支持的平台：**

| 平台代号 | 型号 | 驱动形式 | 轮数 |
|----------|------|----------|------|
| `dd100` | Dingo-D 1.0 | 差分驱动 | 2 |
| `dd150` | Dingo-O 1.5 | **全向驱动** | 4 |
| `j100` | Jackal | 差分驱动 | 4 |
| `w200` | Warthog | 差分驱动 | 4 |

> **Uni-Dingo 使用的是 `dd150`（Dingo-O 全向底盘）**。

**关键文件：**

| 文件 | 说明 |
|------|------|
| `urdf/dd150/dd150.urdf.xacro` | Dingo-O URDF 主模型 |
| `urdf/dd150/wheels/wheel.urdf.xacro` | 全向轮模型 |
| `urdf/dd150/attachments/top_plate.urdf.xacro` | 顶部安装板 |
| `urdf/common.urdf.xacro` | 通用宏定义 |

### clearpath_control — 控制器配置

为每个平台提供 `ros2_control` 控制器参数和遥控配置。

**Dingo-O (DD150) 控制器配置核心：**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50   # Hz

platform_velocity_controller:
  ros__parameters:
    left_wheel_names: ["front_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint"]
    wheel_separation: 0.37559
    wheel_radius: 0.049
    linear.x.max_velocity: 1.3           # m/s
    angular.z.max_velocity: 4.0          # rad/s
    linear.x.max_acceleration: 1.0       # m/s²
    angular.z.max_acceleration: 2.0      # rad/s²
```

**遥控配置：**

| 配置文件 | 说明 |
|----------|------|
| `config/dd150/teleop_logitech.yaml` | Logitech 手柄遥控 |
| `config/dd150/teleop_ps4.yaml` | PS4 手柄遥控 |
| `config/dd150/localization.yaml` | EKF 里程计融合 |

**Launch 文件：**

| 文件 | 说明 |
|------|------|
| `launch/control.launch.py` | 启动 ros2_control 管理器 + 控制器 spawner |
| `launch/localization.launch.py` | 启动 `robot_localization` EKF |
| `launch/teleop_joy.launch.py` | 启动手柄遥控 |
| `launch/teleop_base.launch.py` | 启动底盘遥控 |

### clearpath_hardware_interfaces — 硬件接口

实现 `hardware_interface::SystemInterface` 的 ros2_control 插件。

**支持的硬件后端：**

| 类 | 适用平台 | 通信方式 |
|----|----------|----------|
| `PumaHardware` | Dingo (DD100/DD150) | CAN (Puma 电机驱动板) |
| `J100Hardware` | Jackal | MCU 串口 |
| `W200Hardware` | Warthog | MCU 串口 |
| `A200Hardware` | Husky | Horizon 串口协议 |
| `DiffDriveHardware` | 通用差分驱动 | 抽象接口 |

> **Uni-Dingo 项目使用 `PumaHardware`**：
> - `read()` 通过 ROS 话题接收 Puma 电机反馈（速度、位移）
> - `write()` 通过 ROS 话题发送速度命令到 Puma 电机
> - 电机反馈/命令经由 `puma_motor_driver` 包与 CAN 总线桥接

### clearpath_motor_drivers / puma_motor_driver — Puma 电机驱动

Puma 电机控制板的 CAN 协议驱动节点，通过 `clearpath_ros2_socketcan_interface` 与 CAN 总线交互。

**节点：**

| 节点 | 说明 |
|------|------|
| `multi_puma_node` | 多 Puma 驱动管理（支持 2/4 电机） |

**PumaFeedback 消息字段：**

```
uint8 device_number         # CAN ID
string device_name          # 关节名（如 front_left_wheel_joint）
float32 duty_cycle          # H 桥占空比 (-1..1)
float32 current             # 电流 (A)
float64 travel              # 编码器位移 (rad)
float64 speed               # 编码器速度 (rad/s)
float64 setpoint            # 控制器设定值
```

**PumaStatus 消息字段：**

```
uint8 device_number
float32 bus_voltage          # 输入电压 (V)
float32 temperature          # 驱动板温度 (°C)
float32 output_voltage       # 电机输出电压 (V)
uint8 mode                   # 控制模式 (VOLTAGE/CURRENT/SPEED/POSITION)
uint8 fault                  # 故障码 (CURRENT/TEMPERATURE/BUS_VOLTAGE/BRIDGE_DRIVER)
```

### clearpath_ros2_socketcan_interface — SocketCAN 桥接

ROS2 SocketCAN 接口层，将 `can_msgs/Frame` 与 Linux SocketCAN 进行桥接。

**节点：**

| Launch | 说明 |
|--------|------|
| `sender.launch.py` | CAN 帧发送节点 |
| `receiver.launch.py` | CAN 帧接收节点 |

### clearpath_msgs — 平台消息定义

#### clearpath_platform_msgs

| 消息 | 说明 |
|------|------|
| `Feedback.msg` | MCU 反馈（50Hz，含电机反馈和控制模式） |
| `Status.msg` | 平台状态（1Hz，含硬件 ID、固件版本、温度、运行时间） |
| `Power.msg` | 电源信息（电压/电流/电池/充电状态，多平台兼容） |
| `Drive.msg` | 驱动命令 |
| `DriveFeedback.msg` | 单路驱动反馈 |
| `StopStatus.msg` | 急停状态 |
| `Lights.msg` / `RGB.msg` | LED 灯控 |
| `Fans.msg` | 散热风扇 |

**服务：**

| 服务 | 说明 |
|------|------|
| `ConfigureMcu.srv` | MCU 配置 |

#### clearpath_motor_msgs

| 消息 | 说明 |
|------|------|
| `PumaFeedback.msg` | 单 Puma 电机反馈 |
| `PumaMultiFeedback.msg` | 多 Puma 电机反馈（聚合） |
| `PumaStatus.msg` | 单 Puma 电机状态 |
| `PumaMultiStatus.msg` | 多 Puma 电机状态（聚合） |

### clearpath_config — YAML 配置解析

Python 库，解析 Clearpath 机器人的 YAML 配置文件（`/etc/clearpath/robot.yaml`），提供类型安全的配置访问接口。

### clearpath_robot — 真机元包

包含以下子组件：

| 子包 | 说明 |
|------|------|
| `clearpath_sensors` | 20+ 种传感器的 launch 文件集 |
| `clearpath_generator_robot` | 根据 YAML 配置自动生成 launch/param 文件 |
| `clearpath_diagnostics` | 系统诊断（`diagnostic_updater`） |

**支持的传感器 launch（部分列表）：**

| 传感器 | Launch 文件 |
|--------|-------------|
| Hokuyo UST 激光雷达 | `hokuyo_ust.launch.py` |
| Velodyne 激光雷达 | `velodyne_lidar.launch.py` |
| Ouster OS1 激光雷达 | `ouster_os1.launch.py` |
| SICK LMS1xx | `sick_lms1xx.launch.py` |
| FLIR Blackfly 相机 | `flir_blackfly.launch.py` |
| Axis 网络相机 | `axis_camera.launch.py` |
| ZED 立体相机 | `stereolabs_zed.launch.py` |
| MicroStrain GQ7 IMU | `microstrain_gq7.launch.py` |
| Phidgets Spatial IMU | `phidgets_spatial.launch.py` |
| Novatel GNSS | `novatel_smart6/7.launch.py` |
| Garmin GPS | `garmin_18x.launch.py` |

## 与 Uni-Dingo 系统的集成关系

在 Uni-Dingo 项目中，`dingo_ros2` 的角色是**底层底盘驱动供应商**：

```
uni_dingo_api_ros2 (BaseAPI)
        │ /cmd_vel
        ▼
unidingo_description_ros2
  └── dingo_base.xacro          ← 引用 clearpath_platform_description
  └── dingo_base.ros2_control.xacro  ← 声明 PumaHardware 插件
        │
        ▼
clearpath_hardware_interfaces   ← 提供 ros2_control SystemInterface
        │
        ▼
puma_motor_driver               ← CAN 通信
        │
        ▼
clearpath_ros2_socketcan        ← Linux SocketCAN
```

**关键集成点：**

| Uni-Dingo 组件 | 引用的 dingo_ros2 组件 |
|----------------|----------------------|
| `unidingo_description_ros2/urdf/dingo_base.xacro` | `clearpath_platform_description` 底盘模型 |
| `unidingo_description_ros2/urdf/dingo_base.ros2_control.xacro` | `clearpath_hardware_interfaces/PumaHardware` 插件 |
| `unidingo_description_ros2/config/base_simulation_controllers.yaml` | `clearpath_control` 控制器参数格式 |
| `uni_dingo_api_ros2` BaseAPI | `/cmd_vel` → `platform_velocity_controller` |

## 使用方法

### 在 Uni-Dingo 项目中使用（推荐）

通常不需要直接使用 `dingo_ros2` 的 launch 文件。通过统一入口启动：

```bash
# 仿真模式（Gazebo 内置 DiffDrive 插件）
ros2 launch uni_dingo_api bringup.launch.py mode:=simulation base:=true

# 真机模式（通过 PumaHardware → CAN 驱动）
ros2 launch uni_dingo_api bringup.launch.py mode:=real base:=true
```

### 独立底盘控制器（调试用）

```bash
# 启动 ros2_control 管理器 + 控制器
ros2 launch clearpath_control control.launch.py setup_path:=/etc/clearpath/

# 启动 EKF 里程计融合
ros2 launch clearpath_control localization.launch.py setup_path:=/etc/clearpath/

# 启动手柄遥控
ros2 launch clearpath_control teleop_joy.launch.py setup_path:=/etc/clearpath/
```

### CAN 总线调试

```bash
# 创建虚拟 CAN 接口（用于无硬件测试）
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 启动 SocketCAN 桥接节点
ros2 launch clearpath_ros2_socketcan_interface sender.launch.py
ros2 launch clearpath_ros2_socketcan_interface receiver.launch.py

# 查看 CAN 帧
candump vcan0
```

### 查看底盘模型

```bash
ros2 launch clearpath_platform_description description.launch.py
```

## 主要参数（Dingo-O / DD150）

| 参数 | 值 |
|------|------|
| 平台型号 | DD150 (Dingo-O) |
| 驱动形式 | 全向驱动 (Omni-directional) |
| 轮径 | 0.049 m |
| 轮距 | 0.37559 m |
| 最大线速度 | 1.3 m/s |
| 最大角速度 | 4.0 rad/s |
| 电机驱动 | Puma (CAN 总线) |
| 控制频率 | 50 Hz |
| 编码器 CPR | 10 |
| 减速比 | 24.0 |
| CAN 设备 | `vcan0`（仿真）/ `can0`（真机） |
| 电机 CAN ID | 2 (左前), 3 (右前) |

## 依赖

- **核心**: `rclcpp`, `hardware_interface`, `controller_manager`, `pluginlib`
- **控制器**: `diff_drive_controller`, `joint_state_broadcaster`
- **通信**: `can_msgs`, Linux SocketCAN
- **定位**: `robot_localization`（EKF 里程计融合）
- **遥控**: `joy`, `teleop_twist_joy`, `teleop_twist_keyboard`
- **消息**: `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs`
- **可视化**: `xacro`, `robot_state_publisher`, `rviz2`

## 许可证

本目录包含的代码源自 [Clearpath Robotics](https://clearpathrobotics.com/) 官方仓库，采用 **BSD 许可证**。详见各文件头部的许可声明。
