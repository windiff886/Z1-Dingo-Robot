# Dingo ROS Package

Dingo 是 Clearpath Robotics 的室内移动机器人平台的 ROS 驱动包。

## 项目结构

```
dingo/
├── dingo_base        # 基础驱动节点，硬件接口
├── dingo_bringup     # 启动配置和传感器驱动
├── dingo_control     # 控制器、定位和遥控
├── dingo_description # URDF 模型描述
├── dingo_msgs        # 自定义消息类型
├── dingo_robot       # 元包
└── dingo_tests       # 测试包
```

## 编译说明

**本包无需编译**。这是预编译的二进制包，包含已编译好的节点和配置文件。

目录中的 `CATKIN_IGNORE` 文件会让 `catkin_make` 跳过此目录。

如需使用，只需确保：
1. ROS 环境已正确配置 (`source /opt/ros/noetic/setup.bash`)
2. 工作空间已 source (`source devel/setup.bash`)
3. 依赖包已安装（见下方依赖列表）

### 主要依赖

```bash
sudo apt install ros-noetic-controller-manager \
                 ros-noetic-diff-drive-controller \
                 ros-noetic-imu-filter-madgwick \
                 ros-noetic-robot-localization \
                 ros-noetic-twist-mux \
                 ros-noetic-teleop-twist-joy \
                 ros-noetic-joy \
                 ros-noetic-interactive-marker-twist-server
```

## 快速启动

### 1. 启动基础驱动

```bash
# 使用手柄控制 (默认)
roslaunch dingo_base base.launch

# 使用键盘控制
roslaunch dingo_base base.launch teleop:=keyboard
```

这将启动：
- Dingo 硬件驱动节点
- IMU 滤波器
- 控制器 (velocity controller)
- EKF 定位
- 遥控功能 (手柄/键盘，取决于 teleop 参数)
- 诊断聚合器

#### base.launch 详细解析

该 launch 文件初始化机器人运行所需的全部底层功能：

**1. rosserial 消息服务**
- 启动 rosserial 的消息信息服务，用于与底层 MCU 进行串口通信

**2. Dingo 主节点 (dingo_node)**
- 核心驱动节点，与底盘硬件通信
- 根据 `DINGO_OMNI` 环境变量决定是全向驱动还是差速驱动
- 加载电机参数配置文件
- 配置无线网络接口

**3. IMU 滤波器 (imu_filter_madgwick)**
- 使用 Madgwick 算法融合 IMU 的加速度计、陀螺仪和磁力计数据
- 输出稳定的姿态估计（orientation）
- 关键参数：gain=0.1, zeta=0.001

**4. 控制系统 (control.launch)**
- `controller_spawner`: 加载关节状态发布器和速度控制器
- `ekf_localization_node`: 扩展卡尔曼滤波器，融合轮式里程计和 IMU 数据
- `twist_mux`: 速度指令多路复用器，管理多个速度输入源的优先级

**5. 遥控系统 (teleop.launch)**
- `joy_node`: 读取手柄输入（默认 `/dev/input/ps4`）
- `teleop_twist_joy`: 将手柄输入转换为 Twist 速度命令
- `twist_marker_server`: RViz 交互式标记服务器

**6. 诊断聚合器 (diagnostic_aggregator)**
- 聚合机器人各组件的诊断信息
- 根据底盘类型加载对应的诊断配置

**系统架构图：**

```
┌─────────────────────────────────────────────────────────────┐
│                      base.launch                             │
├─────────────────────────────────────────────────────────────┤
│  ┌────────────┐   ┌────────────┐   ┌──────────────────┐    │
│  │ dingo_node │   │ imu_filter │   │ diagnostic_      │    │
│  │  (硬件驱动) │   │ (姿态估计)  │   │ aggregator       │    │
│  └─────┬──────┘   └──────┬─────┘   └──────────────────┘    │
│        │                 │                                   │
│        ▼                 ▼                                   │
│  ┌───────────────────────────────────────────────────┐      │
│  │ control.launch                                     │      │
│  │ controller_spawner | ekf_localization | twist_mux │      │
│  └───────────────────────────────────────────────────┘      │
│                          ▲                                   │
│  ┌───────────────────────┴───────────────────────────┐      │
│  │ teleop.launch                                      │      │
│  │ joy_node ──▶ teleop_twist_joy ──▶ twist_marker    │      │
│  └───────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### 2. 启动传感器配件

```bash
roslaunch dingo_bringup accessories.launch
```

#### accessories.launch 详细解析

该 launch 文件用于启动 Dingo 的外部传感器和配件驱动，通过环境变量控制启用哪些配件。

**支持的传感器和配件：**

| 类别 | 设备 | 启用环境变量 |
|------|------|-------------|
| 2D 激光雷达 (主) | SICK LMS1xx, TIM551, Hokuyo UST10 | `DINGO_LASER=1` |
| 2D 激光雷达 (副) | 同上 | `DINGO_LASER_SECONDARY=1` |
| 3D 激光雷达 | Velodyne VLP16, HDL32E | `DINGO_LASER_3D=1` |
| Intel RealSense (主) | D435, D435i, D415, D455, L515 | `DINGO_REALSENSE=1` |
| Intel RealSense (副) | 同上 | `DINGO_REALSENSE_SECONDARY=1` |
| FLIR BlackflyS 相机 | USB3/GigE 工业相机 | `DINGO_BLACKFLY=1` |
| Microstrain IMU | GX3, GX5, GX25 等 | `DINGO_IMU_MICROSTRAIN=1` |
| Wibotic 无线充电器 | CAN 接口充电器 | `DINGO_WIBOTIC_CHARGER=1` |

**注意**：默认不启动任何传感器，需通过环境变量显式启用。

**使用示例：**

```bash
# 启用前置激光雷达 + RealSense 相机
export DINGO_LASER=1
export DINGO_LASER_HOST=192.168.131.20
export DINGO_REALSENSE=1
roslaunch dingo_bringup accessories.launch
```

**启动顺序**：应先启动 `base.launch`（底盘驱动），再启动 `accessories.launch`（传感器驱动）。

## 环境变量配置

### 底盘类型

| 变量 | 说明 | 默认值 |
|------|------|--------|
| `DINGO_OMNI` | 设置为 `1` 启用全向底盘模式，`0` 为差分底盘 | `0` |

### 激光雷达配置

| 变量 | 说明 | 默认值 |
|------|------|--------|
| `DINGO_LASER` | 设置为 `1` 启用主激光雷达 | `0` |
| `DINGO_LASER_MODEL` | 雷达型号: `lms1xx`, `tim551`, `ust10` | `lms1xx` |
| `DINGO_LASER_HOST` | 雷达 IP 地址 | `192.168.131.20` |
| `DINGO_LASER_TOPIC` | 发布话题名 | `front/scan` |
| `DINGO_LASER_SECONDARY` | 设置为 `1` 启用副激光雷达 | `0` |

### 3D 激光雷达配置

| 变量 | 说明 | 默认值 |
|------|------|--------|
| `DINGO_LASER_3D` | 设置为 `1` 启用 3D 激光雷达 | `0` |
| `DINGO_LASER_3D_MODEL` | 型号: `vlp16`, `hdl32e` | `vlp16` |
| `DINGO_LASER_3D_HOST` | IP 地址 | `192.168.131.20` |

### Intel RealSense 相机配置

| 变量 | 说明 | 默认值 |
|------|------|--------|
| `DINGO_REALSENSE` | 设置为 `1` 启用 RealSense | `0` |
| `DINGO_REALSENSE_MODEL` | 型号: `d435`, `d435i`, `d415`, `d455`, `l515` | `d435` |
| `DINGO_REALSENSE_TOPIC` | 命名空间 | `realsense` |

### 遥控配置

| 变量 | 说明 | 默认值 |
|------|------|--------|
| `DINGO_JOY_DEV` | 手柄设备路径 | `/dev/input/ps4` |

## 遥控方式

### 1. PS4 手柄控制

手柄控制随 `base.launch` 自动启动。

**按键映射（差速模式）：**

| 功能 | 按键/摇杆 | 参数 |
|------|----------|------|
| 前后移动 | 左摇杆 上下 | 0.4 m/s |
| 左右转向 | 左摇杆 左右 | 1.4 rad/s |
| 使能按钮 | **L1** (必须按住) | - |
| 加速模式 | **R1** | 2.0 m/s |

**蓝牙配对：**
```bash
bluetoothctl
power on
scan on
# 手柄进入配对模式：按住 PS + Share 3秒
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
```

### 2. 键盘控制

启动时指定 teleop 参数为 keyboard：

```bash
export DINGO_OMNI=1

source devel/setup.bash 
roslaunch dingo_base base.launch teleop:=keyboard 

source devel/setup.bash 
rosrun dingo_control simple_keyboard_teleop.py
```
```bash
  # 删除机械臂网络的默认网关（它们不需要访问互联网）
  sudo ip route del default via 192.168.123.1 dev enp111s0
  sudo ip route del default via 192.168.131.1 dev enx68da73ae507e
# 杀掉残留的 ROS 进程
killall -9 rosmaster roscore dingo_node 2>/dev/null
# 或者等待几秒后再启动
```


**按键映射（6键模式，匹配 PS4）：**

```
        W (前进)
        
   A (左转)     D (右转)
        
        S (后退)

   空格键: 使能 (必须按住)
   Shift:  加速模式
```

| 按键 | 功能 | 对应 PS4 |
|------|------|----------|
| `W` | 前进 | 左摇杆 ↑ |
| `S` | 后退 | 左摇杆 ↓ |
| `A` | 左转 | 左摇杆 ← |
| `D` | 右转 | 左摇杆 → |
| `空格` | 使能 (必须按住) | L1 |
| `Shift` | 加速模式 | R1 |

**调整速度参数：**
```bash
roslaunch dingo_control keyboard_teleop.launch speed:=0.8 turn:=1.5
```

### 3. 速度指令优先级

通过 `twist_mux` 管理多个速度输入源：

| 来源 | 话题 | 优先级 |
|------|------|--------|
| 急停 (E-Stop) | `/e_stop` | 255 |
| USB 手柄 | `joy_teleop/cmd_vel` | 10 |
| 蓝牙手柄 | `bluetooth_teleop/cmd_vel` | 9 |
| RViz 交互 | `twist_marker_server/cmd_vel` | 8 |
| 键盘 | `keyboard_teleop/cmd_vel` | 7 |
| 外部命令 | `/cmd_vel` | 1 |

> 优先级高的输入会覆盖低优先级输入。

## 使用示例

### 启动全向底盘 + 激光雷达

```bash
export DINGO_OMNI=1
export DINGO_LASER=1
export DINGO_LASER_HOST=192.168.1.10
roslaunch dingo_base base.launch
```

### 单独启动 URDF 描述

```bash
roslaunch dingo_description description.launch
```

## 主要话题

- `/cmd_vel` - 速度指令输入
- `/odom` - 里程计输出
- `/imu/data` - IMU 数据
- `/front/scan` - 前置激光雷达数据（如已配置）
