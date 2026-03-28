# pan_tilt_ros2

IQR Pan-Tilt 二轴云台的 ROS2 驱动元包，包含自定义消息、硬件驱动、URDF 模型和启动配置四个子包。基于 Modbus RTU 串口协议与云台硬件通信。

## 架构概览

```
 pan_tilt_ros2/
 ├── pan_tilt_msgs         # 自定义消息定义（度数/弧度命令、状态反馈）
 ├── pan_tilt_driver        # C++ 驱动节点 + Python 手柄桥接脚本
 ├── pan_tilt_description   # URDF/xacro 模型 + RViz 可视化 launch
 └── pan_tilt_bringup       # 一键 launch 入口
```

### 数据流

```
                     ┌──────────────────┐
    Joy(手柄)  ──────▶│ pan_tilt_control │ (Python)
                     │  (手柄桥接节点)   │
                     └───────┬──────────┘
                             │ pan_tilt_cmd_deg
                             ▼
┌──────────────────────────────────────────────────────────┐
│              pan_tilt_driver_node (C++)                  │
│                                                          │
│  ┌─────────┐   ┌────────────┐   ┌───────────────────┐   │
│  │ 订阅    │──▶│ PanTilt    │──▶│ Modbus RTU Master │   │
│  │ Cmd     │   │ Driver     │   │ (串口通信)         │   │
│  └─────────┘   └────────────┘   └───────────────────┘   │
│                      │                                   │
│              50Hz 状态轮询                                │
│                      ▼                                   │
│  ┌─────────────────────────────┐                         │
│  │ Publishers:                 │                         │
│  │ · /joint_states             │ → robot_state_publisher │
│  │ · /pan_tilt_status          │ → 状态监控              │
│  └─────────────────────────────┘                         │
└──────────────────────────────────────────────────────────┘
```

## 子包详情

### pan_tilt_msgs

自定义 ROS2 消息类型，CMake 构建（`rosidl_generate_interfaces`）。

| 消息名 | 说明 |
|--------|------|
| `PanTiltCmdDeg.msg` | 角度命令（float32 yaw/pitch 度, int16 speed 度/s） |
| `PanTiltCmdRad.msg` | 弧度命令（float64 yaw/pitch/speed） |
| `PanTiltStatus.msg` | 完整状态反馈（ID、版本、设定值、实际角度、温度、错误码等） |

### pan_tilt_driver

C++ 驱动节点，负责通过 Modbus RTU 串口与云台硬件通信。

#### 核心节点: `pan_tilt_driver_node`

| 属性 | 值 |
|------|------|
| 包名 | `pan_tilt_driver` |
| 可执行文件 | `pan_tilt_driver_node` |
| 节点名 | `pan_tilt_driver_node` |

**参数：**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `ID` | int | `0x01` | 云台 Modbus 从站地址 |
| `port_name` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `yaw_joint_name` | string | `pan_tilt_yaw_joint` | 偏航关节名（用于 JointState） |
| `pitch_joint_name` | string | `pan_tilt_pitch_joint` | 俯仰关节名（用于 JointState） |

**发布话题：**

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/joint_states` | `sensor_msgs/msg/JointState` | 50Hz | 偏航/俯仰弧度值 |
| `/pan_tilt_status` | `pan_tilt_msgs/msg/PanTiltStatus` | 50Hz | 完整设备状态 |

**订阅话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `pan_tilt_cmd_deg` | `pan_tilt_msgs/msg/PanTiltCmdDeg` | 角度命令 |
| `pan_tilt_cmd_rad` | `pan_tilt_msgs/msg/PanTiltCmdRad` | 弧度命令 |

**运动范围限制：**

| 参数 | 度数范围 | 弧度范围 |
|------|----------|----------|
| yaw  | [-60°, 60°] | [-1.047, 1.047] |
| pitch | [-60°, 60°] | [-1.047, 1.047] |
| speed | [1°/s, 30°/s] | [0.017, 0.524] |

#### Python 手柄桥接: `pan_tilt_control.py`

将 `sensor_msgs/Joy` 手柄消息转换为云台角度命令。

| 按键 | 功能 |
|------|------|
| Button 0 | 俯仰 - |
| Button 1 | 偏航 - |
| Button 2 | 俯仰 + |
| Button 3 | 偏航 + |
| Button 5 | 归零 |

### pan_tilt_description

云台 URDF/xacro 模型文件，包含 3D Mesh 和 RViz 可视化 launch。

**关键文件：**

| 文件 | 说明 |
|------|------|
| `pan_tilt.urdf.xacro` | 完整云台模型（可嵌入其他模型） |
| `pan_tilt_only.urdf.xacro` | 独立云台模型（用于单独查看） |
| `pan_tilt_st.urdf.xacro` | 带传动链的云台模型 |
| `kinect_dk.xacro` | 配套 Kinect DK 相机模型 |

**Launch 文件：**

| 文件 | 说明 |
|------|------|
| `view_model.launch.py` | 在 RViz 中查看带 GUI 滑块的云台模型 |
| `view_pan_tilt_only.launch.py` | 仅查看云台独立模型 |

### pan_tilt_bringup

一键启动 launch 入口。

## 使用方法

### 启动真机云台驱动

```bash
ros2 launch pan_tilt_bringup pan_tilt_bringup.launch.py \
  pan_tilt_port:=/dev/ttyUSB0
```

### 可视化云台模型

```bash
# 查看云台模型（带 GUI 关节滑块）
ros2 launch pan_tilt_description view_model.launch.py

# 仅查看独立云台模型
ros2 launch pan_tilt_description view_pan_tilt_only.launch.py
```

### 手柄控制云台

```bash
# 需要先启动手柄驱动并启动云台 bringup
ros2 run pan_tilt_driver pan_tilt_control.py
```

### 命令行发送角度命令

```bash
# 度数命令：将云台移到 yaw=30°, pitch=15°, speed=10°/s
ros2 topic pub /pan_tilt_cmd_deg pan_tilt_msgs/msg/PanTiltCmdDeg \
  '{yaw: 30.0, pitch: 15.0, speed: 10}'

# 弧度命令：
ros2 topic pub /pan_tilt_cmd_rad pan_tilt_msgs/msg/PanTiltCmdRad \
  '{yaw: 0.5, pitch: 0.3, speed: 0.2}'
```

### 通过统一入口启动（推荐）

```bash
ros2 launch uni_dingo_api bringup.launch.py mode:=real head:=true
```

## 依赖

- `rclcpp`, `sensor_msgs`, `std_msgs`
- `rosidl_default_generators`（消息生成）
- 硬件依赖：IQR Pan-Tilt 云台 + USB 串口
