# Z1-Dingo-Robot 项目结构文档

## 项目概述

本项目是一个综合性的ROS机器人工作区，集成了Dingo移动底盘、Unitree Z1双机械臂、云台和相机系统。项目支持Gazebo仿真和MoveIt运动规划，可实现复杂的全身操控任务。

**工作区根目录:** `/root/Z1-Dingo-Robot`

**主要功能:**
- Dingo四轮差速移动底盘控制
- Unitree Z1双机械臂协同控制
- MoveIt运动规划与轨迹执行
- Gazebo物理仿真
- Pan-Tilt云台控制
- Intel D455深度相机集成
- 全身操控与遥控

---

## 目录结构

```
Z1-Dingo-Robot/
├── src/                          # ROS源代码包
    ├── z1_ros/                   # Unitree Z1机械臂相关包
    ├── unidingo_description/     # 组合机器人描述（Dingo+双Z1臂）
    ├── dingo/                    # Dingo底盘相关包
    ├── pan_tilt_ros/             # 云台控制包
    ├── d455_camera/              # Intel D455相机包
    ├── whole_body_manipulation/  # 全身操控包
    └── z1/                       # Z1额外配置
```

---

## 核心功能模块

### 1. Unitree Z1 机械臂系统 (`src/z1_ros/`)

Unitree Z1是6自由度机械臂，本项目支持单臂和双臂配置。

#### 主要子包:

**1.1 z1_controller**
- **路径:** `src/z1_ros/z1_controller/`
- **功能:** Z1机械臂的控制器核心，提供与SDK的接口
- **关键文件:**
  - `main.cpp` - 控制器主程序入口
  - `config/config.yaml` - UDP通信配置、碰撞参数
  - `config/z1.urdf` - 机械臂URDF模型
  - `sim/sim_ctrl.cpp` - 仿真控制支持
- **配置说明:**
  - UDP端口配置用于MCU通信
  - 碰撞检测参数
  - 命令缓存设置

**1.2 z1_hw**
- **路径:** `src/z1_ros/z1_hw/`
- **功能:** 硬件接口层，连接ROS控制器与Z1硬件
- **关键文件:**
  - `src/z1_hw.cpp` - 硬件接口实现
  - `include/z1_hw.hpp` - 接口头文件
  - `launch/z1_hw.launch` - 硬件接口启动文件
  - `config/controllers.yaml` - 控制器配置

**1.3 z1_moveit_config**
- **路径:** `src/z1_ros/z1_moveit_config/`
- **功能:** 单臂MoveIt配置包
- **关键文件:**
  - `launch/move_group.launch` - MoveIt主节点启动
  - `config/kinematics.yaml` - 运动学求解器配置
  - `config/ompl_planning.yaml` - OMPL规划器参数
  - `config/joint_limits.yaml` - 关节限制
  - `config/z1_description.srdf` - 语义机器人描述

**1.4 z1_dual_moveit_config**
- **路径:** `src/z1_ros/z1_dual_moveit_config/`
- **功能:** 双臂MoveIt配置包
- **说明:** 支持左右双臂的协同运动规划

**1.5 z1_bringup**
- **路径:** `src/z1_ros/z1_bringup/`
- **功能:** Z1机械臂启动脚本集合

**1.6 z1_examples**
- **路径:** `src/z1_ros/z1_examples/`
- **功能:** 示例代码和测试程序
- **关键文件:**
  - `examples/move_group_interface.cpp` - MoveIt接口示例

**1.7 z1_sdk**
- **路径:** `src/z1_ros/z1_sdk/`
- **功能:** Unitree Z1 SDK封装
- **关键文件:**
  - `scripts/example_lowcmd.py` - 底层命令示例

**1.8 z1_rviz**
- **路径:** `src/z1_ros/z1_rviz/`
- **功能:** RViz可视化配置

---

### 2. Uni-Dingo 组合机器人 (`src/unidingo_description/`)

组合机器人描述包，整合Dingo底盘与双Z1机械臂。

#### 关键文件:

**2.1 URDF/Xacro模型**
- `urdf/uni_dingo_dual_arm.urdf.xacro` - 双臂组合主模型
  - 组合: Dingo底盘 + 左Z1臂 + 右Z1臂
  - 参数: UnitreeGripper开关、臂支撑半径、左右臂挂载坐标
  - 通过固定关节将双臂挂载到底盘上
- `urdf/z1_arm.urdf.xacro` - 单臂宏定义
  - 可复用的臂模型，通过前缀(left_/right_)区分
- `urdf/uni_dingo.urdf` - 单体URDF模型

**2.2 Launch文件**
- `launch/dual_arm_gazebo_moveit.launch` - **核心启动文件**
  - 启动Gazebo空世界
  - 加载robot_description（通过xacro生成）
  - 加载左右臂控制器参数
  - 加载PID参数
  - spawn模型到Gazebo
  - 启动joint state publisher
  - 可选启动MoveIt move_group
  - 可选启动RViz可视化
- `launch/dual_arm_world.launch` - 带场景的世界启动
- `launch/display.launch` - RViz显示启动
- `launch/gazebo.launch` - Gazebo仿真启动

**2.3 控制器配置**
- `config/left_controllers.yaml` - 左臂控制器参数
- `config/right_controllers.yaml` - 右臂控制器参数
- `config/z1_gazebo_pid.yaml` - Gazebo仿真PID参数
  ```yaml
  left_joint1: {p: 100.0, i: 0.0, d: 1.0}
  left_joint2: {p: 100.0, i: 0.0, d: 1.0}
  ...
  right_joint1: {p: 100.0, i: 0.0, d: 1.0}
  ...
  ```

**2.4 包配置**
- `package.xml` - ROS包描述文件
- `CMakeLists.txt` - 编译配置

---

### 3. Dingo 移动底盘 (`src/dingo/`)

Clearpath Dingo四轮差速移动底盘的ROS驱动和配置。

#### 主要子包:

**3.1 dingo_control**
- **路径:** `src/dingo/dingo_control/`
- **功能:** 底盘控制接口
- **关键文件:**
  - `launch/control.launch` - 控制节点启动
  - `scripts/keyboard_to_joy.py` - 键盘遥控转joy消息

**3.2 dingo_gazebo**
- **路径:** `src/dingo/dingo_gazebo/`
- **功能:** Dingo的Gazebo仿真支持
- **关键文件:**
  - `launch/dingo_world.launch` - Gazebo世界启动
  - `launch/spawn_dingo.launch` - 在Gazebo中生成Dingo模型

**3.3 dingo_navigation**
- **路径:** `src/dingo/dingo_navigation/`
- **功能:** 导航功能包（AMCL、gmapping等）
- **关键文件:**
  - `launch/amcl_demo.launch` - AMCL定位演示
  - `launch/gmapping_demo.launch` - gmapping建图演示

**3.4 dingo_description**
- **路径:** `src/dingo/dingo_description/`
- **功能:** Dingo机器人URDF描述
- **关键文件:**
  - `urdf/dingo.urdf.xacro` - Dingo底盘模型

**3.5 dingo_bringup**
- **路径:** `src/dingo/dingo_bringup/`
- **功能:** Dingo启动脚本

---

### 4. Pan-Tilt 云台系统 (`src/pan_tilt_ros/`)

云台控制系统，用于相机的俯仰和偏航控制。

#### 主要子包:

**4.1 pan_tilt_driver**
- **路径:** `src/pan_tilt_ros/pan_tilt_driver/`
- **功能:** 云台驱动程序（C++/Qt实现）
- **关键文件:**
  - `src/PanTiltDriver.cpp` - 驱动核心
  - `src/PanTiltDriverNode.cpp` - ROS节点封装
  - `nodes/pan_tilt_control.py` - Python控制节点

**4.2 pan_tilt_description**
- **路径:** `src/pan_tilt_ros/pan_tilt_description/`
- **功能:** 云台URDF描述
- **关键文件:**
  - `urdf/pan_tilt.urdf.xacro` - 云台模型

**4.3 pan_tilt_bringup**
- **路径:** `src/pan_tilt_ros/pan_tilt_bringup/`
- **功能:** 云台启动脚本
- **关键文件:**
  - `launch/panTilt_bringup.launch` - 云台启动文件

**4.4 pan_tilt_msgs**
- **路径:** `src/pan_tilt_ros/pan_tilt_msgs/`
- **功能:** 云台自定义消息类型

---

### 5. Intel D455 相机 (`src/d455_camera/`)

Intel RealSense D455深度相机的ROS接口。

#### 关键文件:
- `src/d455_camera_node.cpp` - 相机ROS节点
- `src/d455_camera_nodelet.cpp` - Nodelet实现
- `launch/d455_camera_nodelet.launch` - Nodelet启动文件

---

### 6. 全身操控 (`src/whole_body_manipulation/`)

整合底盘移动和机械臂操作的全身控制包。

#### 关键文件:
- `launch/keyboard_cmd_vel.launch` - 键盘遥控速度命令
- `config/*.yaml` - 遥控配置文件

---

### 7. Unitree 其他机器人 (`src/z1_ros/unitree_ros/`)

Unitree其他机器人（四足等）的ROS支持包。

#### 包含机器人:
- **robots/z1_description/** - Z1描述文件
- **robots/laikago_description/** - Laikago四足机器人
- **robots/aliengo_description/** - Aliengo四足机器人
- **robots/a1_description/** - A1四足机器人
- **unitree_gazebo/** - Unitree机器人Gazebo插件
- **unitree_ros_to_real/** - 真实机器人通信桥接
  - 包含 `unitree_legged_sdk/` SDK实现

---

## 关键配置文件说明

### MoveIt配置

**运动学求解器** (`z1_moveit_config/config/kinematics.yaml`):
- 配置IK求解器类型（KDL/TracIK等）
- 求解器超时和精度参数

**运动规划** (`z1_moveit_config/config/ompl_planning.yaml`):
- OMPL规划器配置
- 支持多种规划算法（RRT、RRTConnect、EST等）

**关节限制** (`z1_moveit_config/config/joint_limits.yaml`):
- 关节位置、速度、加速度限制
- 安全裕度设置

### Gazebo仿真

**PID参数** (`unidingo_description/config/z1_gazebo_pid.yaml`):
- 每个关节的PID增益
- 左右臂独立配置
- 夹爪控制参数

### Z1控制器配置 (`z1_controller/config/config.yaml`):
```yaml
udp:
  mcu_ip: "xxx.xxx.xxx.xxx"
  port_to_mcu: xxxx
  port_to_sdk: xxxx
collision:
  enabled: true
  threshold: xxx
cmdCache: xxx
```

---

## 工作流程

### 1. Gazebo仿真+MoveIt规划（推荐入门流程）

**启动命令:**
```bash
roslaunch uni_dingo dual_arm_gazebo_moveit.launch
```

**工作流程:**
1. Gazebo启动空世界
2. 加载uni_dingo_dual_arm URDF模型
3. spawn模型到Gazebo
4. 启动左右臂position控制器
5. 启动MoveIt move_group节点
6. （可选）启动RViz可视化

**控制链路:**
```
MoveIt Planning → Trajectory Controller →
Gazebo Joint Controllers → Joint State Publisher →
Robot State Publisher → TF Tree
```

### 2. 真实机器人控制流程

**启动顺序:**
1. 硬件接口层
   ```bash
   roslaunch z1_hw z1_hw.launch
   ```
2. MoveIt规划
   ```bash
   roslaunch z1_moveit_config move_group.launch
   ```
3. 用户接口（RViz/代码）

**控制链路:**
```
MoveIt Planning → Trajectory Controller →
z1_hw (Hardware Interface) →
z1_controller (SDK Interface) →
Unitree SDK → Z1 Hardware
```

### 3. 导航与建图流程

**SLAM建图:**
```bash
roslaunch dingo_navigation gmapping_demo.launch
```

**AMCL定位:**
```bash
roslaunch dingo_navigation amcl_demo.launch
```

---

## 数据流架构

```
┌─────────────────────────────────────────────────────────────┐
│                        用户层                                │
│  RViz可视化 / MoveIt接口 / 键盘遥控 / Python脚本            │
└─────────────────────────────────────────────────────────────┘
                            ↓↑
┌─────────────────────────────────────────────────────────────┐
│                      ROS通信层                               │
│  Topics / Services / Actions / TF                           │
└─────────────────────────────────────────────────────────────┘
                            ↓↑
┌──────────────────┬──────────────────┬─────────────────────┐
│  MoveIt规划层    │   导航层         │   传感器层          │
│  move_group      │   move_base      │   camera/lidar      │
│  planning scene  │   costmap        │   joint states      │
└──────────────────┴──────────────────┴─────────────────────┘
                            ↓↑
┌──────────────────┬──────────────────┬─────────────────────┐
│  控制器层        │   硬件接口层     │   仿真层            │
│  trajectory_ctrl │   z1_hw          │   Gazebo            │
│  joint_ctrl      │   dingo_base     │   physics engine    │
└──────────────────┴──────────────────┴─────────────────────┘
                            ↓↑
┌─────────────────────────────────────────────────────────────┐
│                      硬件层                                  │
│  Z1机械臂 / Dingo底盘 / 云台 / 相机                         │
└─────────────────────────────────────────────────────────────┘
```
