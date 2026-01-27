# Phase 1 实施记录：统一机器人框架基础搭建

**完成日期**: 2026-01-28
**状态**: ✅ 已完成

---

## 1. 概述

Phase 1 完成了 Uni-Dingo 统一机器人框架的基础搭建，包括消息定义、核心 API 接口和仿真后端实现。

### 1.1 完成的目标

| 任务 | 状态 | 说明 |
|------|------|------|
| 创建 `uni_dingo_msgs` 消息包 | ✅ | 定义了机器人状态、动作等消息类型 |
| 创建 `uni_dingo_api` 包结构 | ✅ | Python 包，包含 API 和后端 |
| 实现 BaseAPI 及仿真后端 | ✅ | 底盘控制接口 + Gazebo 后端 |
| 实现 ArmAPI 及仿真后端 | ✅ | 机械臂控制接口 + MoveIt 后端 |
| 实现 RobotAPI 整机接口 | ✅ | 聚合所有子系统的统一入口 |
| 创建统一启动文件 | ✅ | `bringup.launch` 支持 sim/real 切换 |

---

## 2. 新增文件清单

### 2.1 uni_dingo_msgs 包

```
src/uni_dingo_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── RobotState.msg          # 整机状态
│   ├── BaseState.msg           # 底盘状态
│   ├── ArmState.msg            # 机械臂状态
│   ├── HeadState.msg           # 云台状态
│   ├── GripperState.msg        # 夹爪状态
│   └── SubsystemStatus.msg     # 子系统状态
├── srv/
│   ├── GetRobotState.srv       # 获取机器人状态
│   ├── SetEnabled.srv          # 启用/禁用子系统
│   └── EmergencyStop.srv       # 急停服务
└── action/
    ├── MoveBase.action         # 底盘移动动作
    ├── MoveArm.action          # 机械臂移动动作
    ├── MoveGripper.action      # 夹爪控制动作
    ├── MoveHead.action         # 云台移动动作
    └── WholeBodyMove.action    # 全身协调移动动作
```

### 2.2 uni_dingo_api 包

```
src/uni_dingo_api/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── src/uni_dingo_api/
│   ├── __init__.py
│   ├── api/
│   │   ├── __init__.py
│   │   ├── robot_api.py        # RobotAPI - 整机统一接口
│   │   ├── base_api.py         # BaseAPI - 底盘控制
│   │   ├── arm_api.py          # ArmAPI - 机械臂控制
│   │   └── action_future.py    # ActionFuture - 异步操作封装
│   └── backends/
│       ├── __init__.py
│       ├── simulation/
│       │   ├── __init__.py
│       │   ├── gazebo_base.py  # Gazebo 底盘后端
│       │   └── moveit_arm.py   # MoveIt 机械臂后端
│       └── real/
│           └── __init__.py     # 真机后端（Phase 3）
├── config/
│   └── robot_config.yaml       # 机器人配置文件
├── launch/
│   └── bringup.launch          # 统一启动文件
└── scripts/
    ├── robot_state_publisher.py  # 状态发布节点
    └── test_api.py               # API 测试脚本
```

---

## 3. API 接口说明

### 3.1 RobotAPI - 整机接口

```python
from uni_dingo_api import RobotAPI

robot = RobotAPI()                    # 自动检测模式
robot = RobotAPI(mode='simulation')   # 指定仿真模式
robot = RobotAPI(mode='real')         # 指定真机模式

# 子系统访问
robot.base          # BaseAPI 实例
robot.left_arm      # 左臂 ArmAPI 实例
robot.right_arm     # 右臂 ArmAPI 实例
robot.arms          # 双臂协同 ArmAPI 实例

# 整机方法
robot.get_state()           # 获取完整机器人状态
robot.is_ready()            # 检查是否就绪
robot.emergency_stop()      # 急停
robot.recover()             # 从急停恢复
robot.home()                # 所有臂回 home 位
robot.stop_all()            # 停止所有运动
robot.whole_body_move(...)  # 全身协调运动
```

### 3.2 BaseAPI - 底盘控制

```python
# 速度控制
robot.base.move(linear_x=0.5, linear_y=0.0, angular_z=0.3)
robot.base.stop()

# 位置控制（返回 ActionFuture）
future = robot.base.move_to(x=1.0, y=0.5, theta=1.57)
future.wait()               # 阻塞等待完成
future.is_done()            # 检查是否完成
future.is_success()         # 检查是否成功
future.cancel()             # 取消动作

# 状态查询
robot.base.get_odom()       # 获取里程计
robot.base.get_pose_2d()    # 获取 2D 位姿
robot.base.get_velocity()   # 获取当前速度
robot.base.is_moving()      # 是否在移动

# 启用/禁用
robot.base.set_enabled(True)
robot.base.is_enabled()
```

### 3.3 ArmAPI - 机械臂控制

```python
# 运动控制（均返回 ActionFuture）
robot.left_arm.move_to_named('home').wait()     # 移动到命名位姿
robot.left_arm.move_to_joint([0, 0, 0, 0, 0, 0]).wait()  # 关节空间
robot.left_arm.move_to_pose(target_pose).wait()  # 笛卡尔空间
robot.left_arm.stop()

# 夹爪控制
robot.left_arm.gripper_open().wait()
robot.left_arm.gripper_close().wait()
robot.left_arm.gripper_set(position=-0.5).wait()

# 状态查询
robot.left_arm.get_joint_states()       # 关节状态
robot.left_arm.get_end_effector_pose()  # 末端位姿
robot.left_arm.is_moving()              # 是否在移动

# 双臂协同
robot.arms.move_to_named('home').wait()
robot.arms.move_both(left_pose, right_pose).wait()

# 配置
robot.left_arm.set_velocity_scaling(0.5)
robot.left_arm.set_acceleration_scaling(0.3)
```

### 3.4 ActionFuture - 异步操作

```python
# 基本用法
future = robot.left_arm.move_to_named('home')
future.wait()                # 阻塞等待
result = future.get_result() # 获取结果

# 非阻塞检查
if future.is_done():
    if future.is_success():
        print("成功")
    else:
        print(f"失败: {future.get_error()}")

# 回调方式
future.on_done(lambda result, success: print(f"完成: {success}"))
future.on_feedback(lambda fb: print(f"进度: {fb.progress}"))

# 取消
future.cancel()
```

---

## 4. 配置文件

### 4.1 robot_config.yaml

```yaml
robot:
  name: "uni_dingo"
  mode: "simulation"      # "simulation" | "real"

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
    enabled: false        # Phase 2

  perception:
    enabled: false        # Phase 2

named_poses:
  home:
    left: [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]
    right: [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]

motion:
  default_velocity_scaling: 0.3
  default_acceleration_scaling: 0.3
```

---

## 5. 使用方法

### 5.1 启动机器人

```bash
# 仿真模式（完整，带 GUI）
roslaunch uni_dingo_api bringup.launch

# 仿真模式（无 GUI，用于服务器）
roslaunch uni_dingo_api bringup.launch gui:=false

# 仿真模式（无 RViz）
roslaunch uni_dingo_api bringup.launch rviz:=false

# 真机模式（Phase 3 完成后可用）
roslaunch uni_dingo_api bringup.launch mode:=real
```

### 5.2 运行测试

```bash
# 运行 API 测试脚本
rosrun uni_dingo_api test_api.py
```

### 5.3 Python 代码示例

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from uni_dingo_api import RobotAPI

rospy.init_node('my_robot_control')

# 初始化
robot = RobotAPI()

# 检查状态
print(f"模式: {robot.mode}")
print(f"就绪: {robot.is_ready()}")

# 底盘移动
robot.base.move(linear_x=0.3)
rospy.sleep(2.0)
robot.base.stop()

# 机械臂回 home
robot.left_arm.move_to_named('home').wait()
robot.right_arm.move_to_named('home').wait()

# 或双臂同时
robot.arms.move_to_named('home').wait()

# 移动到目标位姿
target = Pose()
target.position = Point(0.4, 0.0, 0.3)
target.orientation = Quaternion(0, 0, 0, 1)

future = robot.left_arm.move_to_pose(target)
future.wait()

if future.is_success():
    print("到达目标位置")
else:
    print(f"失败: {future.get_error()}")

# 夹爪操作
robot.left_arm.gripper_open().wait()
rospy.sleep(1.0)
robot.left_arm.gripper_close().wait()

# 查询状态
state = robot.get_state()
print(f"左臂关节: {state['left_arm']['joint_positions']}")
```

---

## 6. ROS 话题和服务

### 6.1 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/uni_dingo/state` | `uni_dingo_msgs/RobotState` | 整机状态 (10Hz) |

### 6.2 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/odometry/filtered` | `nav_msgs/Odometry` | 底盘里程计 |

### 6.3 发布到的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |

---

## 7. 架构说明

```
┌─────────────────────────────────────────────────────────┐
│                    用户代码 / 应用层                      │
│              from uni_dingo_api import RobotAPI          │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                      RobotAPI                            │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────────┐   │
│  │ BaseAPI │ │ ArmAPI  │ │ ArmAPI  │ │ (HeadAPI)   │   │
│  │  base   │ │left_arm │ │right_arm│ │   Phase 2   │   │
│  └─────────┘ └─────────┘ └─────────┘ └─────────────┘   │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                    后端层 (Backends)                     │
│  ┌─────────────────────┐  ┌─────────────────────────┐  │
│  │  simulation/        │  │  real/ (Phase 3)        │  │
│  │  ├─ gazebo_base.py │  │  ├─ dingo_base.py       │  │
│  │  └─ moveit_arm.py  │  │  └─ z1_arm.py           │  │
│  └─────────────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                     ROS 通信层                           │
│        Topics / Services / Actions / TF                  │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│              Gazebo / MoveIt / 真实硬件                   │
└─────────────────────────────────────────────────────────┘
```

---

## 8. 后续计划

### Phase 2: 完善仿真环境
- [ ] 实现 HeadAPI（云台控制）
- [ ] 实现 PerceptionAPI（相机接口）
- [ ] 完善状态发布（详细的关节/传感器状态）
- [ ] 添加 Gazebo 传感器仿真
- [ ] 编写更多测试用例

### Phase 3: 真机适配
- [ ] 实现 Dingo 真机后端
- [ ] 实现 Z1 SDK 后端
- [ ] 实现 Pan-Tilt 真机后端
- [ ] 实现 RealSense 后端
- [ ] 联调与标定

### Phase 4: 高级功能
- [ ] 全身运动控制优化
- [ ] 碰撞检测与规避
- [ ] 行为树 / 状态机框架
- [ ] 远程监控界面

---

## 9. 已知限制

1. **夹爪控制**: 当前夹爪控制是占位实现，需要根据实际硬件完善
2. **双臂同步**: `move_both()` 当前是串行执行，未来可优化为真正并行
3. **move_to()**: 底盘位置控制使用简单比例控制，复杂场景建议使用导航栈
4. **真机后端**: Phase 3 才会实现

---

## 10. 编译验证

```bash
# 编译命令
cd /root/Z1-Dingo-Robot
catkin_make --pkg uni_dingo_msgs uni_dingo_api

# 编译结果: 成功 ✅
```

---

*文档版本: 1.0*
*最后更新: 2026-01-28*
