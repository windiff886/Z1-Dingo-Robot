# whole_body_example_ros2 (whole_body_example)

Uni-Dingo 机器人的**示例与工具包**，提供键盘遥控和位姿可视化两个实用工具，用于开发调试和功能演示。

## 架构概览

```
whole_body_example_ros2/
├── whole_body_example/
│   ├── __init__.py
│   ├── keyboard_teleop.py     # 键盘遥控全机器人控制
│   └── pose_visualizer.py     # 机械臂预设位姿可视化
├── launch/
│   ├── keyboard_teleop.launch.py
│   └── pose_visualizer.launch.py
├── rviz/                       # RViz 预设配置
├── setup.py / setup.cfg
├── package.xml
└── test/
    └── test_example_entrypoints.py
```

## 工具说明

### 1. keyboard_teleop — 键盘遥控

基于 `RobotAPI` 的终端键盘遥控工具，支持同时控制底盘、双臂末端、夹爪和云台。

#### 核心特性

- **底盘控制**：全向移动（前后左右平移 + 旋转），Shift 加速
- **机械臂末端控制**：相对位移方式控制当前选中臂的末端位姿（基于 MoveIt2 规划）
- **双臂切换**：Tab 键切换当前控制的手臂
- **夹爪控制**：独立控制左/右夹爪开关
- **云台控制**：增量式偏航/俯仰控制，Shift 加速
- **20Hz 发布循环**：独立线程持续发布底盘速度命令
- **非阻塞手臂运动**：手臂运动在后台线程执行，不阻塞键盘监听

#### 按键映射

| 功能 | 按键 | 说明 |
|------|------|------|
| **底盘** | | |
| 前进/后退 | `w` / `s` | Shift (W/S) 加速 ×2 |
| 左移/右移 | `a` / `d` | Shift (A/D) 加速 ×2 |
| 左转/右转 | `q` / `e` | Shift (Q/E) 加速 ×2 |
| 急停 | `Space` | 停止所有运动 |
| **机械臂** | | |
| 双臂 Home | `1` | |
| 左臂 Home | `2` | |
| 右臂 Home | `3` | |
| **末端位姿** | | (当前选中臂) |
| 末端上/下 | `t` / `g` | Shift 加速(T/G): ±5cm |
| 末端左/右 | `f` / `h` | ±2cm |
| 末端前/后 | `r` / `y` | Shift 加速(R/Y): ±5cm |
| 切换臂 | `Tab` | 左臂 ↔ 右臂 |
| **夹爪** | | |
| 左夹爪 开/关 | `o` / `p` | |
| 右夹爪 开/关 | `[` / `]` | |
| **云台** | | |
| 上仰/下俯 | `i` / `k` | Shift (I/K) 加速 |
| 左转/右转 | `j` / `l` | Shift (J/L) 加速 |
| 云台归零 | `H` | |
| **系统** | | |
| 退出 | `ESC` | |

#### 使用方法

```bash
# 直接运行（默认使用 robot_config.yaml 中的 mode）
ros2 run whole_body_example keyboard_teleop

# 指定模式
ros2 run whole_body_example keyboard_teleop --ros-args -p mode:=simulation

# 通过 launch 文件运行
ros2 launch whole_body_example keyboard_teleop.launch.py
```

#### 工作原理

```
┌──────────────────────────────────────────────────────────┐
│                  KeyboardTeleop                          │
│  ┌──────────┐     ┌──────────────┐     ┌─────────────┐  │
│  │ 键盘监听 │────▶│ 按键解析     │────▶│ RobotAPI    │  │
│  │ (主线程) │     │ & 状态更新   │     │ 调用        │  │
│  └──────────┘     └──────────────┘     └─────────────┘  │
│                                                          │
│  ┌──────────────────────┐                                │
│  │ 速度发布线程 (20Hz)  │──── robot.base.move() ────▶    │
│  └──────────────────────┘                                │
│                                                          │
│  ┌──────────────────────┐                                │
│  │ 手臂运动线程 (按需)  │──── arm.move_to_*() ────▶     │
│  └──────────────────────┘                                │
└──────────────────────────────────────────────────────────┘
```

### 2. pose_visualizer — 位姿可视化

在 RViz 中以 50Hz 发布 JointState 来可视化机械臂预设位姿，用于调试和验证关节配置。

#### 核心特性

- 以 50Hz 发布完整机器人（20 个关节）的 JointState
- 支持预设位姿切换（如 `home`）
- 交互式命令行：支持实时切换位姿、调整云台、控制夹爪
- 显示关节弧度/角度详情

#### 支持的关节

```
底盘: front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel
左臂: left_joint1~6, left_jointGripper
右臂: right_joint1~6, right_jointGripper
云台: pan_tilt_yaw_joint, pan_tilt_pitch_joint
```

#### 交互命令

| 命令 | 说明 |
|------|------|
| `home` | 切换到 home 位姿 |
| `head <pan> <tilt>` | 设置云台角度（弧度） |
| `grip open` / `grip close` | 控制夹爪 |
| `help` | 显示帮助 |
| `q` | 退出 |

#### 使用方法

```bash
# 直接运行
ros2 run whole_body_example pose_visualizer

# 指定位姿名和发布频率
ros2 run whole_body_example pose_visualizer --ros-args \
  -p pose_name:=home \
  -p publish_rate:=50.0

# 通过 launch 文件运行（自带 RViz）
ros2 launch whole_body_example pose_visualizer.launch.py
```

#### 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pose_name` | string | `home` | 初始位姿名称 |
| `publish_rate` | float | `50.0` | JointState 发布频率 (Hz) |

## 预设位姿

| 位姿名 | 左臂关节 | 右臂关节 |
|--------|----------|----------|
| `home` | `[0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]` | `[0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]` |

## 依赖

- `rclpy`, `sensor_msgs`, `geometry_msgs`
- `uni_dingo_api`（keyboard_teleop 通过 RobotAPI 控制机器人）
