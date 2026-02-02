# Uni-Dingo API

统一机器人 API 包，整合底盘、机械臂、云台和相机为一个统一接口。

## 架构

```
┌─────────────────────────────────────────────────────┐
│                     RobotAPI                        │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌───────────┐ │
│  │ BaseAPI │ │ ArmAPI  │ │ HeadAPI │ │Perception │ │
│  │         │ │(左/右)  │ │         │ │   API     │ │
│  └─────────┘ └─────────┘ └─────────┘ └───────────┘ │
└─────────────────────────────────────────────────────┘
                          │
          ┌───────────────┴───────────────┐
          ▼                               ▼
┌─────────────────────┐       ┌─────────────────────┐
│  Simulation Backend │       │    Real Backend     │
│  - GazeboBase       │       │  - DingoBase        │
│  - MoveItArm        │       │  - Z1Arm            │
│  - (Gazebo head)    │       │  - PanTiltHead      │
│                     │       │  - D455Camera       │
└─────────────────────┘       └─────────────────────┘
```

## 启动

### 仿真模式

```bash
roslaunch uni_dingo_api bringup.launch mode:=simulation gui:=true rviz:=true
```

### 真机模式

```bash
roslaunch uni_dingo_api real_robot.launch
```

## 配置

配置文件位于 `config/robot_config.yaml`：

```yaml
robot:
  mode: "simulation"  # "simulation" | "real"
  base:
    enabled: true
  arms:
    enabled: true
    left: true
    right: true
  head:
    enabled: true
  perception:
    camera: true
```

## Python API 使用

```python
#!/usr/bin/env python3
from uni_dingo_api import RobotAPI

robot = RobotAPI()

# 底盘控制
robot.base.move(vx=0.5, vy=0.0, wz=0.0)
odom = robot.base.get_odom()

# 机械臂控制
robot.left_arm.move_to_named("home").wait()
robot.right_arm.move_to_joint([0, 0, 0, 0, 0, 0]).wait()

# 夹爪控制
robot.left_arm.gripper_close().wait()

# 云台控制
robot.head.set_angles(pan=0.0, tilt=0.0)

# 感知
rgb = robot.perception.get_rgb()
depth = robot.perception.get_depth()
```

## 目录结构

```
uni_dingo_api/
├── src/uni_dingo_api/
│   ├── api/                 # API 接口定义
│   │   ├── robot_api.py     # 整机 API
│   │   ├── base_api.py      # 底盘 API
│   │   ├── arm_api.py       # 机械臂 API
│   │   ├── head_api.py      # 云台 API
│   │   └── perception_api.py # 感知 API
│   └── backends/            # 后端实现
│       ├── simulation/      # 仿真后端
│       └── real/            # 真机后端
├── config/                  # 配置文件
├── launch/                  # 启动文件
└── scripts/                 # 脚本
```

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/uni_dingo/state` | `uni_dingo_msgs/RobotState` | 整机状态 (10Hz) |

## 依赖

- uni_dingo_msgs
- unidingo_description
- dingo, z1_ros, pan_tilt_ros, d455_camera
