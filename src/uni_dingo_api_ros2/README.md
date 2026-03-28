# uni_dingo_api_ros2 (uni_dingo_api)

Uni-Dingo 机器人的**统一 Python API 与系统启动包**，提供面向应用开发者的高级接口和整机 launch 管理。作为整个系统的**顶层入口**，封装了底盘、双臂、云台、感知四大子系统。

## 架构概览

```
┌─────────────────────────────────────────────────────────────┐
│                        RobotAPI                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────────┐  │
│  │ BaseAPI  │  │ ArmAPI   │  │ HeadAPI  │  │ Perception │  │
│  │ (底盘)   │  │ (双臂)   │  │ (云台)   │  │ API (感知) │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └─────┬──────┘  │
│       │              │              │               │        │
│  cmd_vel        MoveIt2 Py      PanTilt         Image/      │
│  Odometry       GripperCmd      CmdDeg          PointCloud   │
│       │              │              │               │        │
│       ▼              ▼              ▼               ▼        │
│  ┌──────┐      ┌──────────┐  ┌──────────┐   ┌──────────┐   │
│  │Dingo │      │z1_ros2   │  │pan_tilt  │   │d455      │   │
│  │底盘  │      │控制器    │  │driver    │   │camera    │   │
│  └──────┘      └──────────┘  └──────────┘   └──────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 类结构

```
RobotAPI                          # 顶层统一入口
├── base: BaseAPI                 # 全向底盘控制（cmd_vel / odom）
├── left_arm: ArmAPI              # 左臂控制（MoveIt2 + Gripper）
├── right_arm: ArmAPI             # 右臂控制（MoveIt2 + Gripper）
├── arms: ArmAPI (both)           # 双臂协同控制
├── head: HeadAPI                 # 云台控制（仿真/真机自动切换）
├── perception: PerceptionAPI     # 深度相机数据获取
└── ActionFuture                  # 异步操作封装（Promise 模式）
```

## 目录结构

```
uni_dingo_api_ros2/
├── package.xml
├── setup.py / setup.cfg
├── config/
│   └── robot_config.yaml           # 机器人全局配置
├── launch/
│   ├── bringup.launch.py           # 总入口（simulation/real 自动分发）
│   ├── simulation.launch.py        # Gazebo 仿真 launch
│   ├── real.launch.py              # 真机 launch
│   ├── visualization.launch.py     # 纯模型可视化 launch
│   └── includes/
│       ├── base.launch.py          # 底盘子 launch
│       └── moveit.launch.py        # MoveIt2 子 launch
├── uni_dingo_api/
│   ├── __init__.py
│   ├── robot_state_publisher_node.py  # 统一状态发布节点
│   └── api/
│       ├── __init__.py
│       ├── robot_api.py            # RobotAPI 主类
│       ├── arm_api.py              # ArmAPI（MoveIt2 Python 接口）
│       ├── base_api.py             # BaseAPI（底盘速度/里程计）
│       ├── head_api.py             # HeadAPI（云台仿真/真机双模）
│       ├── perception_api.py       # PerceptionAPI（RGB/Depth/PointCloud）
│       └── action_future.py        # ActionFuture 异步封装
├── scripts/
└── test/
    ├── test_ros2_api_contracts.py
    ├── test_bringup_launch_contract.py
    └── ...（多个对齐/契约测试）
```

## API 模块详解

### RobotAPI

统一顶层入口，自动根据 `robot_config.yaml` 初始化所有子系统。

```python
import rclpy
from uni_dingo_api import RobotAPI

rclpy.init()
robot = RobotAPI(mode='simulation')   # 或 mode='real'

# 检查就绪状态
if robot.is_ready():
    print("机器人已就绪")

# 获取全机器人状态
state = robot.get_state()

# 急停与恢复
robot.emergency_stop()
robot.recover()

# 全身协调运动
robot.whole_body_move(
    base_goal=(1.0, 0.0, 0.0),
    left_arm_goal=[0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
).wait()

robot.destroy()
rclpy.shutdown()
```

### BaseAPI — 底盘控制

```python
# 速度控制（立即执行，非阻塞）
robot.base.move(linear_x=0.5, linear_y=0.0, angular_z=0.3)
robot.base.stop()

# 目标点导航（异步，带 ActionFuture）
future = robot.base.move_to(x=2.0, y=1.0, theta=0.0)
future.wait()

# 状态查询
pose = robot.base.get_pose_2d()     # Pose2D(x, y, theta)
vel = robot.base.get_velocity()      # Twist
odom = robot.base.get_odom()         # nav_msgs/Odometry
is_moving = robot.base.is_moving()
```

**话题映射（可通过 config 自定义）：**

| 功能 | 话题 | 类型 |
|------|------|------|
| 速度命令 | `/cmd_vel` | `geometry_msgs/Twist` |
| 里程计 | `/odometry/filtered` | `nav_msgs/Odometry` |

### ArmAPI — 机械臂控制

基于 MoveIt2 Python API (`moveit_py`)，支持位姿规划、关节规划、预设位姿和夹爪控制。

```python
# 末端位姿控制
from geometry_msgs.msg import Pose
target = Pose()
target.position.x = 0.4
target.position.z = 0.3
robot.left_arm.move_to_pose(target).wait()

# 关节空间控制
robot.left_arm.move_to_joint([0.0, 0.5, -0.5, 0.0, 0.0, 0.0]).wait()

# 预设位姿
robot.left_arm.move_to_named('home').wait()

# 双臂协同
robot.arms.move_to_named('home').wait()

# 夹爪控制（ROS2 Action: GripperCommand）
robot.left_arm.gripper_open().wait()
robot.left_arm.gripper_close().wait()

# 状态查询
joints = robot.left_arm.get_joint_states()  # {names, positions, velocities, efforts}
ee_pose = robot.left_arm.get_end_effector_pose()  # geometry_msgs/Pose via TF
```

**关键配置：**

| 参数 | 左臂 | 右臂 |
|------|------|------|
| Planning Group | `left_arm_manipulator` | `right_arm_manipulator` |
| End Effector Link | `left_link06` | `right_link06` |
| Joint Names | `left_joint1..6` | `right_joint1..6` |
| Gripper Action | `/left_arm/left_gripper_controller/gripper_cmd` | `/right_arm/right_gripper_controller/gripper_cmd` |

### HeadAPI — 云台控制

自动根据 `mode` 切换仿真/真机通信方式：
- **仿真模式**：通过 `Float64` 话题发送位置命令到 Gazebo position controller
- **真机模式**：通过 `PanTiltCmdDeg` 消息控制 IQR 云台驱动

```python
# 角度控制（弧度）
robot.head.set_angles(pan=0.5, tilt=-0.2).wait()

# 视觉跟踪（look at 3D 点）
from geometry_msgs.msg import PointStamped
point = PointStamped()
point.header.frame_id = 'base_link'
point.point.x = 1.0
point.point.z = 0.5
robot.head.look_at(point).wait()

# 归零
robot.head.home().wait()

# 查询状态
pan, tilt = robot.head.get_angles()           # 弧度
pan_deg, tilt_deg = robot.head.get_angles_deg()  # 角度
```

### PerceptionAPI — 感知接口

懒初始化设计，首次调用时才创建订阅者，避免不使用时占用资源。

```python
# 获取 RGB 图像
rgb_msg = robot.perception.get_rgb(timeout=2.0)      # sensor_msgs/Image
rgb_np = robot.perception.get_rgb_numpy(timeout=2.0)  # numpy BGR array

# 获取深度图
depth_msg = robot.perception.get_depth()
depth_np = robot.perception.get_depth_numpy()  # float32 矩阵 (单位: 米)

# 获取点云
cloud = robot.perception.get_pointcloud()  # sensor_msgs/PointCloud2

# 相机内参
intrinsics = robot.perception.get_intrinsics()  # {fx, fy, cx, cy, width, height}

# 像素到 3D 点转换
xyz = robot.perception.pixel_to_point(u=320, v=240, depth=1.5)

# 获取相机在 base_link 下的位姿
camera_pose = robot.perception.get_camera_pose(frame='base_link')
```

### ActionFuture — 异步操作封装

所有运动类接口返回 `ActionFuture`，提供 Promise 风格的异步操作：

```python
future = robot.left_arm.move_to_named('home')

# 阻塞等待
future.wait(timeout=30.0)

# 查询状态
future.is_done()       # bool
future.is_success()    # bool
future.get_result()    # Any
future.get_error()     # str

# 回调方式
future.on_done(lambda result, success: print(f"完成: {success}"))

# 取消
future.cancel()

# 快捷工厂
ActionFuture.success(result=...)
ActionFuture.failure('error msg')
```

## Launch 系统

### 总入口

```bash
ros2 launch uni_dingo_api bringup.launch.py
```

根据 `mode` 参数自动分发到 `simulation.launch.py` 或 `real.launch.py`。

#### 顶层参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `simulation` | `simulation` 或 `real` |
| `gui` | `true` | 是否显示 Gazebo GUI |
| `rviz` | `false` | 是否启动 RViz |
| `world` | `dingo_race.world` | Gazebo 世界文件 |
| `paused` | `true` | Gazebo 是否暂停启动 |
| `teleop` | `none` | `keyboard` / `joystick` / `none` |
| `moveit` | `true` | 是否启动 MoveIt2 |
| `auto_safe_pose` | `true` | 启动后自动归位 |
| `base` | `true` | 启用底盘 |
| `left_arm` | `true` | 启用左臂 |
| `right_arm` | `true` | 启用右臂 |
| `head` | `true` | 启用云台 |
| `perception` | `true` | 启用相机 |

### 仿真模式

```bash
ros2 launch uni_dingo_api bringup.launch.py \
  mode:=simulation \
  teleop:=keyboard \
  rviz:=true
```

### 真机模式

```bash
ros2 launch uni_dingo_api bringup.launch.py \
  mode:=real \
  left_arm_ip:=192.168.123.110 \
  right_arm_ip:=192.168.123.111
```

### 纯模型可视化

```bash
ros2 launch uni_dingo_api visualization.launch.py gui:=true
```

## 配置文件

`config/robot_config.yaml` 是全局配置中心，定义：

- 机器人模式（simulation/real）
- 各子系统启用状态与话题映射
- 机械臂 Planning Group、关节名、控制器
- 云台关节限位与话题
- 感知话题（仿真/真机分别配置）
- 预设命名位姿（如 `home`）
- 运动规划参数（速度缩放、加速度缩放、规划时间）

## 依赖

- `rclpy`, `ament_index_python`
- `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `control_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `moveit` (MoveIt2 Python interface `moveit_py`)
- `pan_tilt_msgs`（可选，真机云台控制）
- `cv_bridge`（可选，OpenCV 图像转换）
- `numpy`
