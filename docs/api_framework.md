# Uni-Dingo 统一机器人 API 框架

## 1. 设计目标

`uni_dingo_api` 将底盘、双臂、云台和相机封装成一套统一 Python API，当前 ROS2 版本已经具备以下特征：

- `mode` 支持 `simulation` / `real`
- 既支持内部创建 `Node`，也支持外部注入 `Node`
- 长时操作统一返回 `ActionFuture`
- 双臂规划基于 MoveIt2 (`moveit_py` + `z1_moveit_config`)
- 顶层 `bringup.launch.py` 已恢复为 ROS1 风格总入口
- `simulation` 语义为 Gazebo 仿真，`real` 语义为真机 bringup

## 2. 快速使用

```python
import rclpy
from geometry_msgs.msg import Pose
from uni_dingo_api import RobotAPI

rclpy.init()

robot = RobotAPI(mode='simulation')

if robot.is_ready():
    robot.base.move(linear_x=0.3, angular_z=0.2)
    robot.left_arm.move_to_named('home').wait(timeout=30.0)

    pose = Pose()
    pose.position.x = 0.4
    pose.position.z = 0.3
    robot.left_arm.move_to_pose(pose).wait(timeout=10.0)

    ee_pose = robot.left_arm.get_end_effector_pose()
    state = robot.get_state()

    robot.whole_body_move(
        base_goal=(0.5, 0.0, 0.0),
        left_arm_goal=[0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
    ).wait()

robot.destroy()
rclpy.shutdown()
```

## 3. 主要类与能力

### 3.1 RobotAPI

```python
class RobotAPI:
    base: BaseAPI | None
    left_arm: ArmAPI | None
    right_arm: ArmAPI | None
    arms: ArmAPI | None
    head: HeadAPI | None
    perception: PerceptionAPI | None

    def __init__(self, mode=None, config_path=None, node=None)
    def get_state(self) -> dict
    def is_ready(self) -> bool
    def emergency_stop(self) -> None
    def recover(self) -> bool
    def home(self) -> ActionFuture
    def stop_all(self) -> None
    def whole_body_move(self, base_goal=None, left_arm_goal=None, right_arm_goal=None) -> ActionFuture

    @property
    def mode(self) -> str
    @property
    def is_simulation(self) -> bool
    @property
    def node(self) -> Node

    def spin(self) -> None
    def destroy(self) -> None
```

### 3.2 BaseAPI

```python
class BaseAPI:
    def move(self, linear_x=0.0, linear_y=0.0, angular_z=0.0) -> None
    def move_to(self, x: float, y: float, theta: float, frame='odom') -> ActionFuture
    def stop(self) -> None
    def emergency_stop(self) -> None
    def recover(self) -> None

    def get_velocity(self) -> Twist
    def get_pose_2d(self) -> Pose2D
    def get_topics(self) -> dict
    def is_moving(self) -> bool
    def is_enabled(self) -> bool
    def set_enabled(self, enabled: bool) -> None
```

### 3.3 ArmAPI

```python
class ArmAPI:
    def __init__(self, node, side='left' | 'right' | 'both', mode='simulation', config=None)

    def move_to_pose(self, pose: Pose, frame='base_link', side=None) -> ActionFuture
    def move_to_joint(self, positions: list[float], side=None) -> ActionFuture
    def move_to_named(self, name: str, side=None) -> ActionFuture
    def move_both(self, left_pose: Pose, right_pose: Pose, frame='base_link') -> ActionFuture
    def stop(self, side=None) -> None

    def gripper_open(self, side=None, position=-1.57) -> ActionFuture
    def gripper_close(self, side=None, position=0.0) -> ActionFuture
    def gripper_set(self, position: float, side=None) -> ActionFuture

    def get_joint_states(self, side=None) -> dict | None
    def get_end_effector_pose(self, side=None, frame='base_link') -> Pose | dict | None
    def is_moving(self, side=None) -> bool
    def is_enabled(self) -> bool
    def set_enabled(self, enabled: bool) -> None
    def set_velocity_scaling(self, scaling: float) -> None
    def set_acceleration_scaling(self, scaling: float) -> None
```

当前执行控制器映射：

- 左臂：`left_arm_controller`
- 右臂：`right_arm_controller`

MoveIt2 配置包：

- `/src/z1_ros2/z1_moveit_config`

### 3.4 HeadAPI

```python
class HeadAPI:
    def set_angles(self, pan: float, tilt: float, speed: float = 0.5) -> ActionFuture
    def look_at(self, point: PointStamped, frame='base_link') -> ActionFuture
    def home(self) -> ActionFuture
    def stop(self) -> None

    def get_angles(self) -> tuple[float, float]
    def get_angles_deg(self) -> tuple[float, float]
    def get_limits(self) -> dict
    def is_moving(self) -> bool
    def set_enabled(self, enabled: bool) -> None
    def is_enabled(self) -> bool
```

### 3.5 PerceptionAPI

```python
class PerceptionAPI:
    def get_rgb(self, timeout=1.0) -> Optional[Image]
    def get_rgb_numpy(self, timeout=1.0) -> Optional[np.ndarray]
    def get_depth(self, timeout=1.0) -> Optional[Image]
    def get_depth_numpy(self, timeout=1.0) -> Optional[np.ndarray]
    def get_pointcloud(self, timeout=1.0) -> Optional[PointCloud2]
    def get_camera_info(self, timeout=1.0) -> Optional[CameraInfo]
    def get_intrinsics(self) -> Optional[dict]
    def get_camera_pose(self, frame='base_link', timeout=1.0) -> Optional[Pose]
    def lookup_transform(self, target_frame, source_frame, timeout=1.0) -> Optional[TransformStamped]
    def pixel_to_point(self, u, v, depth) -> Optional[tuple[float, float, float]]
    def has_data(self) -> bool
    def get_topics(self) -> dict
    def set_enabled(self, enabled: bool) -> None
    def is_enabled(self) -> bool
```

### 3.6 ActionFuture

```python
class ActionFuture:
    def wait(self, timeout=None)
    def is_done(self) -> bool
    def is_success(self) -> bool
    def get_result(self)
    def get_error(self) -> str
    def on_done(self, callback) -> None

    @staticmethod
    def success(result=None) -> ActionFuture
    @staticmethod
    def failure(error_msg='') -> ActionFuture
```

## 4. 当前系统配置要点

`uni_dingo_api/config/robot_config.yaml` 当前已包含：

- `base`：
  - `/cmd_vel`
  - `/odometry/filtered`
- `left_arm` / `right_arm`：
  - planning group
  - end effector link
  - joint names
  - trajectory controller
- `head`：
  - `pan_tilt_yaw_joint`
  - `pan_tilt_pitch_joint`
  - 仿真 / 真机命令话题
- `perception`：
  - 仿真与真机 RGB / depth / points / camera_info 话题
- `named_poses.home`
- `motion`：
  - velocity / acceleration scaling
  - planning time

## 5. 统一启动参数

`uni_dingo_api bringup.launch.py` 当前是总入口，主要参数为：

- `mode:=simulation|real`
- `gui:=true|false`
- `rviz:=true|false`
- `world:=<gazebo_world>`
- `paused:=true|false`
- `teleop:=keyboard|joystick|none`
- `moveit:=true|false`
- `auto_safe_pose:=true|false`
- `base:=true|false`
- `left_arm:=true|false`
- `right_arm:=true|false`
- `head:=true|false`
- `perception:=true|false`
- `state_publisher:=true|false`
- `has_gripper:=true|false`
- `moveit_package:=<package>`

`uni_dingo_api real.launch.py` 仍可作为独立真机快捷入口，承接的核心真机参数为：

- `left_arm_ip`
- `left_port_to_mcu`
- `left_port_to_sdk`
- `left_own_port`
- `right_arm_ip`
- `right_port_to_mcu`
- `right_port_to_sdk`
- `right_own_port`
- `pan_tilt_port`
- `camera_serial`
- `color_fps`
- `depth_fps`

独立模型可视化入口：

- `uni_dingo_api visualization.launch.py`
- 主要参数：
  - `gui:=true|false`
  - `use_rviz:=true|false`
  - `head:=true|false`
  - `perception:=true|false`
  - `has_gripper:=true|false`

## 6. 与 ROS1 版本的主要差异

| 方面 | ROS1 | ROS2 |
|---|---|---|
| 顶层入口 | `bringup.launch` + `real_robot.launch` | `bringup.launch.py` + `real.launch.py` |
| 运动规划 | `moveit_commander` | `moveit_py` + `z1_moveit_config` |
| 硬件抽象 | `ros_control` | `ros2_control` |
| 云台摇杆工具 | `pan_tilt_control.py` | `pan_tilt_control.py` (ROS2 版) |
| 位姿可视化 | `pose_visualizer.py` | `pose_visualizer.py` (ROS2 版) |
| 底盘集成 | 本仓库内 ROS1 包 | 外部 Clearpath ROS2 包，通过适配层接入 |

## 7. 已知边界

- `bringup.launch.py --show-args` 的参数展示面仍不完全等价于 ROS1
- Gazebo 与真机的运行级联调证据仍需继续补充
- 底盘整机闭环仍依赖外部 Clearpath ROS2 包
- D455 的运行时数据仍依赖本机正确安装 `realsense2_camera`

当前更完整的对齐状态请参见：

- `docs/ros2_strict_alignment_status.md`
