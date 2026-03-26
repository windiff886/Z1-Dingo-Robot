# Uni-Dingo 统一机器人 API 框架 (ROS2)

## 1. 设计理念

将 Dingo 底盘、双 Z1 机械臂、Pan-Tilt 云台、D455 相机整合为**一个统一的移动操作机器人系统**：

- **仿真/真机无缝切换** — 同一套 Python 代码，`mode` 参数切换运行环境
- **模块化组合** — 各子系统可独立启用/禁用，通过 `robot_config.yaml` 配置
- **异步优先** — 长时操作返回 `ActionFuture`，支持 `wait()` / `is_success()` / 回调
- **Node 注入** — API 不自行创建节点，接受外部 `rclpy.Node` 参数，便于集成

---

## 2. 快速使用

```python
import rclpy
rclpy.init()

from uni_dingo_api import RobotAPI

# 方式一：API 内部创建节点
robot = RobotAPI(mode='real')

# 方式二：传入已有节点
node = rclpy.create_node('my_app')
robot = RobotAPI(mode='simulation', node=node)

# 底盘
robot.base.move(linear_x=0.3, angular_z=0.5)
robot.base.stop()

# 机械臂
robot.left_arm.move_to_named('home').wait(timeout=30.0)
robot.right_arm.move_to_joint([0.0, 0.5, -0.3, 0.0, 0.0, 0.0]).wait()

from geometry_msgs.msg import Pose
target = Pose()
target.position.x = 0.4
target.position.z = 0.3
future = robot.left_arm.move_to_pose(target)
future.wait()
if future.is_success():
    print('到达目标位姿')
else:
    print(f'失败: {future.get_error()}')

# 夹爪
robot.left_arm.gripper_open().wait()
robot.left_arm.gripper_close().wait()

# 双臂协同
robot.arms.move_to_named('home').wait()

# 云台
robot.head.set_angles(pan=0.5, tilt=-0.2).wait()
robot.head.look_at(point_stamped).wait()
robot.head.home().wait()

# 感知
rgb_msg = robot.perception.get_rgb(timeout=2.0)
depth_np = robot.perception.get_depth_numpy()
intrinsics = robot.perception.get_intrinsics()
camera_pose = robot.perception.get_camera_pose(frame='base_link')

# 状态查询
state = robot.get_state()
joints = robot.left_arm.get_joint_states()
pan, tilt = robot.head.get_angles()

# 安全
robot.emergency_stop()
robot.recover()
robot.stop_all()

# 清理
robot.destroy()
rclpy.shutdown()
```

---

## 3. API 接口

### 3.1 RobotAPI — 整机入口

```python
class RobotAPI:
    base: BaseAPI              # 底盘 (可为 None)
    left_arm: ArmAPI           # 左臂 (可为 None)
    right_arm: ArmAPI          # 右臂 (可为 None)
    arms: ArmAPI               # 双臂协同 (可为 None)
    head: HeadAPI              # 云台 (可为 None)
    perception: PerceptionAPI  # 感知 (可为 None)

    def __init__(self, mode=None, config_path=None, node=None)
    def get_state(self) -> dict
    def emergency_stop(self) -> None
    def recover(self) -> bool
    def home(self) -> ActionFuture        # 双臂/单臂归零
    def stop_all(self) -> None
    def spin(self)                        # 阻塞旋转内部节点
    def destroy(self)                     # 释放资源
    @property
    def mode(self) -> str                 # 'simulation' | 'real'
    @property
    def is_simulation(self) -> bool
    @property
    def node(self) -> Node                # 底层 rclpy.Node
```

### 3.2 BaseAPI — 底盘控制

```python
class BaseAPI:
    def move(self, linear_x=0, linear_y=0, angular_z=0) -> None
    def stop(self) -> None
    def emergency_stop(self) -> None
    def recover(self) -> None

    def get_velocity(self) -> Twist       # 当前速度
    def get_pose_2d(self) -> Pose2D       # 当前 2D 位姿 (from odom)
    def is_moving(self) -> bool
    def is_enabled(self) -> bool
    def set_enabled(self, enabled: bool) -> None
```

### 3.3 ArmAPI — 机械臂控制

```python
class ArmAPI:
    def __init__(self, node, side='left'|'right'|'both', mode, config)

    # 运动规划 (MoveIt2)
    def move_to_pose(self, pose: Pose, frame='base_link') -> ActionFuture
    def move_to_joint(self, positions: List[float]) -> ActionFuture  # 6 个关节角
    def move_to_named(self, name: str) -> ActionFuture               # 'home', 等
    def stop(self) -> None

    # 夹爪 (ros2 action: GripperCommand)
    def gripper_open(self, position=-1.57) -> ActionFuture
    def gripper_close(self, position=0.0) -> ActionFuture
    def gripper_set(self, position: float) -> ActionFuture

    # 状态查询
    def get_joint_states(self) -> dict    # {names, positions, velocities, efforts}
    def is_moving(self) -> bool
    def is_enabled(self) -> bool
    def set_enabled(self, enabled: bool) -> None
    def set_velocity_scaling(self, scaling: float) -> None
    def set_acceleration_scaling(self, scaling: float) -> None
```

### 3.4 HeadAPI — 云台控制

```python
class HeadAPI:
    def set_angles(self, pan: float, tilt: float, speed=0.5) -> ActionFuture
    def look_at(self, point: PointStamped) -> ActionFuture
    def home(self) -> ActionFuture
    def stop(self) -> None

    def get_angles(self) -> Tuple[float, float]       # (pan, tilt) 弧度
    def get_angles_deg(self) -> Tuple[float, float]    # (pan, tilt) 角度
    def get_limits(self) -> dict                       # pan_min/max, tilt_min/max
    def is_enabled(self) -> bool
    def set_enabled(self, enabled: bool) -> None
```

### 3.5 PerceptionAPI — 感知接口

```python
class PerceptionAPI:
    # 图像获取 (懒初始化订阅)
    def get_rgb(self, timeout=1.0) -> Optional[Image]
    def get_rgb_numpy(self, timeout=1.0) -> Optional[np.ndarray]       # BGR
    def get_depth(self, timeout=1.0) -> Optional[Image]
    def get_depth_numpy(self, timeout=1.0) -> Optional[np.ndarray]     # 米
    def get_pointcloud(self, timeout=1.0) -> Optional[PointCloud2]
    def get_camera_info(self, timeout=1.0) -> Optional[CameraInfo]

    # 内参
    def get_intrinsics(self) -> Optional[dict]   # {fx, fy, cx, cy, width, height}

    # TF
    def get_camera_pose(self, frame='base_link') -> Optional[Pose]
    def lookup_transform(self, target, source) -> Optional[TransformStamped]

    # 工具
    def pixel_to_point(self, u, v, depth) -> Optional[Tuple[float,float,float]]
    def has_data(self) -> bool
    def get_topics(self) -> dict
```

### 3.6 ActionFuture — 异步操作

```python
class ActionFuture:
    def wait(self, timeout=None) -> bool     # 阻塞等待完成
    def is_done(self) -> bool                # 非阻塞查询
    def is_success(self) -> bool
    def get_result(self) -> Any
    def get_error(self) -> str
    def on_done(self, callback) -> None      # 注册回调

    @staticmethod
    def success(result=None) -> ActionFuture
    @staticmethod
    def failure(error_msg='') -> ActionFuture
```

---

## 4. 配置文件

`config/robot_config.yaml`:

```yaml
robot:
  mode: "simulation"            # "simulation" | "real"

subsystems:
  base:
    enabled: true
    type: "dingo_omni"
    cmd_vel_topic: "/cmd_vel"
    odom_topic: "/odometry/filtered"
    max_linear_vel: 1.0
    max_angular_vel: 2.0

  left_arm:
    enabled: true
    planning_group: "left_arm_manipulator"
    end_effector_link: "left_link06"

  right_arm:
    enabled: true
    planning_group: "right_arm_manipulator"
    end_effector_link: "right_link06"

  head:
    enabled: true
    pan_min: -1.047              # ±60°
    pan_max: 1.047
    tilt_min: -1.047
    tilt_max: 1.047

  perception:
    enabled: true
    camera_frame: "camera_link"

named_poses:
  home:
    left: [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]
    right: [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]

motion:
  default_velocity_scaling: 0.1
  default_acceleration_scaling: 0.1
  planning_time: 5.0
```

---

## 5. 设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| API 语言 | Python (rclpy) | 开发效率高，适合上层应用 |
| 运动规划 | MoveIt2 (`moveit_py`) | ROS2 标准运动规划框架 |
| 硬件抽象 | `ros2_control` | ROS2 标准硬件接口，统一控制器管理 |
| 异步模型 | 自定义 `ActionFuture` | 封装 ROS2 action client，简化使用 |
| 节点管理 | 外部注入 or 内部创建 | 灵活集成，避免多节点冲突 |
| 模式切换 | YAML 配置 + `mode` 参数 | 运行时灵活，代码层无 if/else |
| 坐标系 | `base_link` 为默认参考系 | 移动机器人惯例 |

---

## 6. 与 ROS1 版本的主要差异

| 方面 | ROS1 | ROS2 |
|------|------|------|
| 节点创建 | `rospy.init_node()` 全局 | `rclpy.Node` 对象注入 |
| 运动规划 | `moveit_commander` | `moveit_py` (`MoveItPy`) |
| 夹爪控制 | `actionlib.SimpleActionClient` | `rclpy.action.ActionClient` |
| 硬件接口 | `ros_control` | `ros2_control` (`SystemInterface`) |
| 参数获取 | `rospy.get_param()` | `node.declare_parameter()` + `get_parameter()` |
| 日志 | `rospy.loginfo()` | `node.get_logger().info()` |
| 包格式 | `catkin` (CMakeLists.txt) | `ament_python` (setup.py) |
| TF | `tf2_ros.Buffer()` | `tf2_ros.Buffer()` + 需传入 Node |
| 定时器 | `rospy.Rate` + `while` 循环 | `node.create_timer()` 回调 |
