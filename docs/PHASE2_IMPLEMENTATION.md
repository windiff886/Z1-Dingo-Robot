# Phase 2 实施记录：完善仿真环境

**完成日期**: 2026-01-28
**状态**: ✅ 已完成

---

## 1. 概述

Phase 2 完成了 HeadAPI（云台控制）、PerceptionAPI（感知接口）的实现，并增强了状态发布和 Gazebo 传感器仿真配置。

### 1.1 完成的目标

| 任务 | 状态 | 说明 |
|------|------|------|
| 实现 HeadAPI 云台控制接口 | ✅ | 支持 set_angles、look_at、home |
| 实现 PerceptionAPI 感知接口 | ✅ | 支持 RGB、深度、点云获取 |
| 完善状态发布节点 | ✅ | 发布详细的子系统状态 |
| 添加 Gazebo 传感器仿真配置 | ✅ | 相机和云台控制器配置 |
| 更新配置和启动文件 | ✅ | 支持启用/禁用子系统 |

---

## 2. 新增文件清单

```
src/uni_dingo_api/
├── src/uni_dingo_api/api/
│   ├── head_api.py           # [新增] 云台控制 API
│   └── perception_api.py     # [新增] 感知接口 API
├── config/
│   ├── robot_config.yaml     # [更新] 添加 head 和 perception 配置
│   └── pan_tilt_controllers.yaml  # [新增] 云台控制器配置
├── urdf/
│   └── sensors.gazebo.xacro  # [新增] Gazebo 传感器插件配置
├── launch/
│   └── bringup.launch        # [更新] 支持子系统启用/禁用
└── scripts/
    └── robot_state_publisher.py  # [更新] 发布详细状态
```

---

## 3. API 接口说明

### 3.1 HeadAPI - 云台控制

```python
from uni_dingo_api import RobotAPI

robot = RobotAPI()

# 设置云台角度（弧度）
robot.head.set_angles(pan=0.5, tilt=-0.3).wait()

# 获取当前角度
pan, tilt = robot.head.get_angles()           # 弧度
pan_deg, tilt_deg = robot.head.get_angles_deg()  # 角度

# 看向目标点
from geometry_msgs.msg import PointStamped
point = PointStamped()
point.header.frame_id = 'base_link'
point.point.x = 1.0
point.point.y = 0.0
point.point.z = 0.5
robot.head.look_at(point).wait()

# 回到原点
robot.head.home().wait()

# 停止
robot.head.stop()

# 获取关节限制
limits = robot.head.get_limits()
# {'pan_min': -1.047, 'pan_max': 1.047, 'tilt_min': -1.047, 'tilt_max': 1.047}
```

### 3.2 PerceptionAPI - 感知接口

```python
from uni_dingo_api import RobotAPI

robot = RobotAPI()

# 获取 RGB 图像
rgb_msg = robot.perception.get_rgb()           # sensor_msgs/Image
rgb_numpy = robot.perception.get_rgb_numpy()   # numpy array (H, W, 3)

# 获取深度图像
depth_msg = robot.perception.get_depth()
depth_numpy = robot.perception.get_depth_numpy()  # numpy array (H, W) in meters

# 获取点云
points = robot.perception.get_pointcloud()  # sensor_msgs/PointCloud2

# 获取相机内参
info = robot.perception.get_camera_info()
intrinsics = robot.perception.get_intrinsics()
# {'fx': ..., 'fy': ..., 'cx': ..., 'cy': ..., 'width': 640, 'height': 480}

# 像素坐标转 3D 点
x, y, z = robot.perception.pixel_to_point(u=320, v=240, depth=1.5)

# 获取相机位姿
camera_pose = robot.perception.get_camera_pose(frame='base_link')

# TF 查询
transform = robot.perception.lookup_transform('base_link', 'camera_link')

# 检查是否有数据
if robot.perception.has_data():
    print("Camera data available")

# 获取话题配置
topics = robot.perception.get_topics()
# {'rgb': '/camera/color/image_raw', 'depth': '...', ...}
```

---

## 4. 配置说明

### 4.1 robot_config.yaml 新增配置

```yaml
subsystems:
  head:
    enabled: true
    pan_joint: "pan_tilt_yaw_joint"
    tilt_joint: "pan_tilt_pitch_joint"
    pan_min: -1.047    # ±60 degrees in radians
    pan_max: 1.047
    tilt_min: -1.047
    tilt_max: 1.047
    # 仿真话题
    sim_pan_topic: "/pan_tilt/pan_controller/command"
    sim_tilt_topic: "/pan_tilt/tilt_controller/command"
    # 真机话题
    real_cmd_topic: "/pan_tilt_cmd_deg"
    real_status_topic: "/pan_tilt_status"

  perception:
    enabled: true
    camera_frame: "camera_link"
    optical_frame: "camera_color_optical_frame"
    # 仿真话题
    sim_rgb_topic: "/camera/color/image_raw"
    sim_depth_topic: "/camera/depth/image_raw"
    sim_points_topic: "/camera/depth/points"
    # 真机话题 (RealSense D455)
    real_rgb_topic: "/d455/color/image_raw"
    real_depth_topic: "/d455/aligned_depth_to_color/image_raw"
```

### 4.2 pan_tilt_controllers.yaml

```yaml
pan_tilt_joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

pan_controller:
  type: position_controllers/JointPositionController
  joint: pan_tilt_yaw_joint
  pid: {p: 50.0, i: 0.1, d: 1.0}

tilt_controller:
  type: position_controllers/JointPositionController
  joint: pan_tilt_pitch_joint
  pid: {p: 50.0, i: 0.1, d: 1.0}
```

---

## 5. 启动方式

```bash
# 启动仿真（默认启用所有子系统）
roslaunch uni_dingo_api bringup.launch

# 禁用云台
roslaunch uni_dingo_api bringup.launch head:=false

# 禁用感知
roslaunch uni_dingo_api bringup.launch perception:=false

# 仅启用底盘和机械臂
roslaunch uni_dingo_api bringup.launch head:=false perception:=false

# 真机模式（需要 Phase 3）
roslaunch uni_dingo_api bringup.launch mode:=real
```

---

## 6. ROS 话题

### 6.1 新增发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/uni_dingo/state` | `uni_dingo_msgs/RobotState` | 包含 head 和 perception 状态 |

### 6.2 云台相关话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pan_tilt/pan_controller/command` | `std_msgs/Float64` | Pan 角度命令（仿真）|
| `/pan_tilt/tilt_controller/command` | `std_msgs/Float64` | Tilt 角度命令（仿真）|
| `/pan_tilt_cmd_deg` | `pan_tilt_msgs/PanTiltCmdDeg` | 命令（真机）|

### 6.3 相机相关话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 点云 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |

---

## 7. 状态发布

`/uni_dingo/state` 话题现在包含：

```yaml
RobotState:
  header: ...
  mode: "simulation"
  emergency_stopped: false
  all_ready: true
  last_update: ...

  base:
    enabled: true
    moving: false
    pose: ...
    velocity: ...

  left_arm:
    side: "left"
    enabled: true
    moving: false
    joint_positions: [...]
    joint_velocities: [...]
    gripper: {position: ...}

  right_arm:
    side: "right"
    ...

  head:
    enabled: true
    moving: false
    pan: 0.0
    tilt: 0.0
    pan_min: -1.047
    pan_max: 1.047
    tilt_min: -1.047
    tilt_max: 1.047

  subsystem_statuses:
    - {name: "base", status: OK, ready: true}
    - {name: "left_arm", status: OK, ready: true}
    - {name: "right_arm", status: OK, ready: true}
    - {name: "head", status: OK, ready: true}
    - {name: "perception", status: OK, ready: true}
```

---

## 8. Gazebo 传感器配置

`sensors.gazebo.xacro` 提供：

1. **RGB-D 相机** (模拟 Intel RealSense D455)
   - 分辨率: 640x480
   - 帧率: 30 FPS
   - 视场角: ~69.4°
   - 深度范围: 0.1m - 10m

2. **云台控制器**
   - Pan/Tilt 位置控制器
   - PID 控制

---

## 9. 架构更新

```
┌─────────────────────────────────────────────────────────────────┐
│                        RobotAPI                                  │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌───────────┐ │
│  │ BaseAPI │ │ ArmAPI  │ │ ArmAPI  │ │ HeadAPI │ │Perception │ │
│  │  base   │ │left_arm │ │right_arm│ │  head   │ │   API     │ │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └───────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## 10. 后续计划

### Phase 3: 真机适配
- [ ] 实现 Dingo 真机后端
- [ ] 实现 Z1 SDK 后端
- [ ] 实现 Pan-Tilt 真机后端
- [ ] 实现 RealSense D455 后端
- [ ] 联调与标定

### Phase 4: 高级功能
- [ ] 全身运动控制优化
- [ ] 视觉伺服
- [ ] 行为树 / 状态机框架
- [ ] 远程监控界面

---

## 11. 编译验证

```bash
cd /root/Z1-Dingo-Robot
catkin_make --pkg uni_dingo_api

# 编译结果: 成功 ✅
```

---

## 12. 版本更新

- **API 版本**: 0.1.0 → 0.2.0
- **新增 API**: HeadAPI, PerceptionAPI
- **新增配置**: head, perception 子系统配置
- **新增文件**: head_api.py, perception_api.py, sensors.gazebo.xacro, pan_tilt_controllers.yaml

---

*文档版本: 1.0*
*最后更新: 2026-01-28*
