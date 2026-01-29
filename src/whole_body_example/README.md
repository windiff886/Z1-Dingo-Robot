# Whole Body Example

Uni-Dingo 机器人使用示例，基于 `uni_dingo_api` 统一接口。

## 编译

```bash
cd ~/lxy/Z1-Dingo-Robot
catkin_make --pkg whole_body_example
source devel/setup.bash
```

## 依赖

- `uni_dingo_api` - 统一机器人 API
- `uni_dingo` - 机器人 URDF 描述
- `z1_dual_moveit_config` - MoveIt 配置
- ROS Noetic

---

## 示例 1: keyboard_teleop.py - 键盘遥控

使用键盘实时控制机器人的底盘、机械臂、云台和夹爪。

### 启动

```bash
# 仿真模式
roslaunch whole_body_example keyboard_teleop.launch mode:=simulation

# 真机模式
roslaunch whole_body_example keyboard_teleop.launch mode:=real

# 直接运行 (需要先启动仿真或真机环境)
rosrun whole_body_example keyboard_teleop.py _mode:=simulation
```

### 按键映射

#### 底盘控制

| 按键 | 功能 | 备注 |
|------|------|------|
| `W` / `S` | 前进 / 后退 | Shift 加速 (2x) |
| `A` / `D` | 左移 / 右移 | 全向底盘专用 |
| `Q` / `E` | 左转 / 右转 | Shift 加速 (2x) |
| `空格` | 急停 | 停止所有运动 |

#### 机械臂控制

| 按键 | 功能 |
|------|------|
| `1` | 双臂 Home (完全伸直) |
| `2` | 双臂 Ready (准备抓取) |
| `3` | 双臂 Stow (收纳位姿) |
| `4` | 左臂 Home |
| `5` | 右臂 Home |

#### 末端位姿控制

通过按键控制当前选中机械臂的末端执行器位置（基于 MoveIt 规划）。

| 按键 | 功能 | 步进量 |
|------|------|--------|
| `T` / `G` | 末端上升 / 下降 | 2cm (Shift: 5cm) |
| `F` / `h` | 末端左移 / 右移 | 2cm (Shift: 5cm) |
| `R` / `Y` | 末端前进 / 后退 | 2cm (Shift: 5cm) |
| `Tab` | 切换控制左臂/右臂 | - |

i
> **注意**: 小写 `h` 用于末端右移，大写 `H` 用于云台归零。

#### 夹爪控制

通过轨迹控制器直接控制夹爪开合。

| 按键 | 功能 |
|------|------|
| `O` / `P` | 左臂夹爪 开 / 关 |
| `[` / `]` | 右臂夹爪 开 / 关 |

#### 云台控制

| 按键 | 功能 | 备注 |
|------|------|------|
| `I` / `K` | 上仰 / 下俯 | Shift 加速 (2x) |
| `J` / `L` | 左转 / 右转 | Shift 加速 (2x) |
| `H` | 云台归零 | 大写 H |

#### 系统

| 按键 | 功能 |
|------|------|
| `ESC` | 退出程序 |

### 工作原理

- 非阻塞键盘读取 (基于 `termios`)
- 独立线程以 20Hz 持续发布底盘速度命令
- 无键盘输入时速度自动衰减归零，防止失控
- 机械臂运动异步执行，不阻塞键盘响应
- 末端位姿控制基于 MoveIt 运动规划，保持末端姿态不变
- 夹爪控制通过 JointTrajectory 直接发送到轨迹控制器
- 状态栏实时显示当前速度和选中的机械臂

---

## 示例 2: pose_visualizer.py - 位姿可视化

在 RViz 中可视化机械臂的预设位姿，无需启动仿真环境或真机。

### 启动

```bash
roslaunch whole_body_example pose_visualizer.launch
```

启动后会打开两个窗口:
1. **RViz** - 显示机器人 3D 模型
2. **终端** - 输入命令切换位姿

### 命令

#### 预设位姿切换

```
输入命令 > home       # 完全伸直，水平向前
输入命令 > ready      # 肩部抬起，肘部微弯，准备抓取
输入命令 > tucked     # 肘部大幅弯曲，紧凑收缩
输入命令 > stow       # 收纳位姿，腕部旋转 90°
```

#### 循环对比

```
输入命令 > compare    # 依次显示所有位姿，每个停留 2 秒
```

#### 云台控制

```
输入命令 > head 0.5 -0.3    # 设置云台角度 (弧度): pan=0.5, tilt=-0.3
输入命令 > head 0 0          # 云台归零
```

#### 夹爪控制

```
输入命令 > grip open     # 双手夹爪张开
输入命令 > grip close    # 双手夹爪闭合
```

#### 退出

```
输入命令 > q
```

### 预设位姿说明

| 位姿 | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 | 说明 |
|------|--------|--------|--------|--------|--------|--------|------|
| home | 0° | 0° | 0° | -4.3° | 0° | 0° | 完全伸直 |
| ready | 0° | 28.6° | -28.6° | -4.3° | 0° | 0° | 肘部微弯 |
| tucked | 0° | 85.9° | -85.9° | -4.3° | 0° | 0° | 大幅弯曲 |
| stow | 0° | 63° | -63° | 90° | 0° | 0° | 腕部旋转收纳 |

### 发布的关节

脚本会发布机器人完整的关节状态，确保 TF 树完整:

```
底盘轮子:  front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel
左臂:     left_joint1 ~ left_joint6, left_jointGripper
右臂:     right_joint1 ~ right_joint6, right_jointGripper
云台:     pan_tilt_yaw_joint, pan_tilt_pitch_joint
```

---

## 文件结构

```
whole_body_example/
├── CMakeLists.txt
├── package.xml
├── README.md
├── scripts/
│   ├── keyboard_teleop.py     # 键盘遥控
│   └── pose_visualizer.py     # 位姿可视化
├── launch/
│   ├── keyboard_teleop.launch
│   └── pose_visualizer.launch
└── rviz/
    └── pose_visualizer.rviz   # RViz 配置文件
```
