# Uni-Dingo 统一机器人 API 框架

## 1. 愿景

将 Dingo 底盘、双 Z1 机械臂、Pan-Tilt 云台、D455 相机整合为**一个统一的移动操作机器人系统**，通过标准化接口实现：

- **仿真/真机无缝切换**：同一套上层代码，一个参数切换运行环境
- **模块化组合**：各子系统可独立运行，也可任意组合
- **统一控制接口**：上层应用无需关心底层硬件差异

---

## 2. 核心原则

| 原则 | 说明 |
|------|------|
| **环境无关** | 接口不暴露 sim/real 细节，由配置决定后端 |
| **异步优先** | 长时操作返回 Future/ActionClient，支持取消和反馈 |
| **状态可观测** | 所有子系统提供统一的状态查询接口 |
| **故障隔离** | 单个子系统故障不影响其他子系统 |

---

## 3. 接口定义

### 3.1 BaseAPI - 底盘控制

```python
class BaseAPI:
    # 运动控制
    def move(self, linear: Vector3, angular: Vector3) -> None
    def move_to(self, pose: Pose2D, frame: str = "map") -> ActionFuture
    def stop(self) -> None

    # 状态查询
    def get_odom(self) -> Odometry
    def get_velocity(self) -> Twist

    # 导航（可选）
    def navigate_to(self, goal: PoseStamped) -> ActionFuture
    def cancel_navigation(self) -> None
```

### 3.2 ArmAPI - 机械臂控制

```python
class ArmAPI:
    def __init__(self, side: Literal["left", "right", "both"])

    # 运动控制
    def move_to_pose(self, pose: Pose, frame: str = "base_link") -> ActionFuture
    def move_to_joint(self, positions: List[float]) -> ActionFuture
    def move_to_named(self, name: str) -> ActionFuture  # "home", "ready", "tucked"
    def stop(self) -> None

    # 夹爪控制
    def gripper_open(self) -> ActionFuture
    def gripper_close(self, force: float = 0.5) -> ActionFuture
    def gripper_set(self, position: float) -> ActionFuture

    # 状态查询
    def get_joint_states(self) -> JointState
    def get_end_effector_pose(self, frame: str = "base_link") -> Pose
    def is_moving(self) -> bool

    # 双臂协同
    def move_both(self, left_pose: Pose, right_pose: Pose) -> ActionFuture
    def move_relative(self, offset: Pose) -> ActionFuture
```

### 3.3 HeadAPI - 云台控制

```python
class HeadAPI:
    # 运动控制
    def look_at(self, point: PointStamped) -> ActionFuture
    def set_angles(self, pan: float, tilt: float) -> ActionFuture
    def home(self) -> ActionFuture

    # 状态查询
    def get_angles(self) -> Tuple[float, float]  # (pan, tilt)
```

### 3.4 PerceptionAPI - 感知接口

```python
class PerceptionAPI:
    # 相机数据
    def get_rgb(self) -> Image
    def get_depth(self) -> Image
    def get_pointcloud(self) -> PointCloud2

    # 相机信息
    def get_camera_info(self) -> CameraInfo
    def get_camera_pose(self, frame: str = "base_link") -> Pose

    # TF 查询
    def lookup_transform(self, target: str, source: str) -> Transform
```

### 3.5 RobotAPI - 整机接口

```python
class RobotAPI:
    """统一入口，聚合所有子系统"""

    base: BaseAPI
    left_arm: ArmAPI
    right_arm: ArmAPI
    arms: ArmAPI  # 双臂协同接口
    head: HeadAPI
    perception: PerceptionAPI

    # 整机状态
    def get_robot_state(self) -> RobotState
    def emergency_stop(self) -> None
    def recover(self) -> bool

    # 全身运动（底盘+手臂协同）
    def whole_body_move(self, base_goal: Pose2D, arm_goals: Dict) -> ActionFuture
```

---

## 4. 仿真/真机切换机制

### 4.1 配置驱动

```yaml
# config/robot_config.yaml
robot:
  mode: "simulation"  # "simulation" | "real"

  base:
    enabled: true
    type: "dingo_omni"  # "dingo_omni" | "dingo_diff"

  arms:
    enabled: true
    left: true
    right: true
    gripper: true

  head:
    enabled: true

  perception:
    camera: true
    lidar: false
```

### 4.2 启动方式

```bash
# 仿真模式
roslaunch uni_dingo_api bringup.launch mode:=simulation gui:=true rviz:=true

# 真机模式
roslaunch uni_dingo_api real_robot.launch

# 部分组件（仅底盘+左臂）
roslaunch uni_dingo_api bringup.launch mode:=simulation arms.right:=false head:=false
```

### 4.3 后端实现结构

```
src/uni_dingo_api/
├── api/
│   ├── base_api.py
│   ├── arm_api.py
│   ├── head_api.py
│   ├── perception_api.py
│   └── robot_api.py
├── backends/
│   ├── simulation/
│   │   ├── gazebo_base.py
│   │   ├── moveit_arm.py
│   │   └── gazebo_head.py
│   └── real/
│       ├── dingo_base.py
│       ├── z1_arm.py
│       └── pan_tilt_head.py
├── config/
│   └── robot_config.yaml
└── launch/
    ├── bringup.launch
    └── real_robot.launch
```

---

## 5. ROS 话题与服务规范

### 5.1 命名规范

```
/uni_dingo/
├── base/
│   ├── cmd_vel              # geometry_msgs/Twist
│   ├── odom                 # nav_msgs/Odometry
│   └── state                # uni_dingo_msgs/BaseState
├── left_arm/
│   ├── joint_states         # sensor_msgs/JointState
│   ├── end_effector_pose    # geometry_msgs/PoseStamped
│   ├── move_to_pose         # Action: MoveArmAction
│   ├── gripper/
│   │   ├── command          # control_msgs/GripperCommand
│   │   └── state            # control_msgs/GripperState
│   └── state                # uni_dingo_msgs/ArmState
├── right_arm/
│   └── ...                  # 同 left_arm
├── head/
│   ├── command              # geometry_msgs/Point (look_at)
│   ├── angles               # uni_dingo_msgs/HeadAngles
│   └── state                # uni_dingo_msgs/HeadState
├── perception/
│   ├── rgb/image_raw        # sensor_msgs/Image
│   ├── depth/image_raw      # sensor_msgs/Image
│   ├── points               # sensor_msgs/PointCloud2
│   └── camera_info          # sensor_msgs/CameraInfo
└── state                    # uni_dingo_msgs/RobotState
```

### 5.2 自定义消息

```
uni_dingo_msgs/
├── msg/
│   ├── RobotState.msg       # 整机状态
│   ├── BaseState.msg        # 底盘状态
│   ├── ArmState.msg         # 机械臂状态
│   └── HeadState.msg        # 云台状态
├── srv/
│   ├── GetRobotState.srv
│   └── EmergencyStop.srv
└── action/
    ├── MoveArm.action
    ├── MoveBase.action
    └── WholeBodyMove.action
```

---

## 6. 使用示例

### 6.1 Python 脚本

```python
#!/usr/bin/env python3
from uni_dingo_api import RobotAPI

# 初始化（自动检测 sim/real）
robot = RobotAPI()

# 底盘移动
robot.base.move_to(x=1.0, y=0.0, theta=0.0).wait()

# 左臂移动到预设位姿
robot.left_arm.move_to_named("ready").wait()

# 右臂移动到指定位姿
target_pose = Pose(position=Point(0.4, -0.1, 0.3), orientation=...)
robot.right_arm.move_to_pose(target_pose).wait()

# 云台看向目标点
robot.head.look_at(point=PointStamped(...)).wait()

# 获取相机图像
rgb = robot.perception.get_rgb()

# 双臂协同
robot.arms.move_both(left_pose, right_pose).wait()

# 夹取动作
robot.left_arm.gripper_close(force=0.5).wait()
```

### 6.2 键盘遥控

```bash
roslaunch whole_body_example keyboard_teleop.launch mode:=simulation
```

---

## 7. 设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| API 语言 | Python 优先 | 开发效率高，ROS 生态支持好 |
| 后端切换 | 配置文件 + 工厂模式 | 运行时灵活，代码解耦 |
| 异步模型 | ActionLib | ROS 标准，支持取消和反馈 |
| 状态管理 | 集中式 RobotState | 便于监控和诊断 |
| 坐标系 | base_link 为默认 | 移动机器人惯例 |
