# Dingo 键盘测试

ROS 1 包名为 `dingo_test`，目录名为 `dingo-test`。
节点向 `/dingo/cmd_vel` 发布 `geometry_msgs/Twist`，支持全向平移和旋转。

在 ROS 1 环境中，从工作区根目录构建：

```bash
catkin_make
source devel/setup.bash
```

先在一个终端启动底盘：

```bash
roslaunch dingo_control dingo.launch
```

再在另一个已加载工作区环境的交互终端运行：

```bash
rosrun dingo_test keyboard_teleop.py
```

| 按键 | 动作 |
|------|------|
| W / S | 前进 / 后退 |
| A / D | 向左 / 向右平移 |
| Q / E | 逆时针 / 顺时针旋转 |
| U / I | 左前 / 右前 |
| J / K | 左后 / 右后 |
| 空格或其他非运动键 | 停车 |
| X / Ctrl-C | 退出 |

方向均相对底盘坐标系。斜向平移经过归一化，不增加总线速度。
终端按键采用单键重复机制，不支持组合按键；按住运动键持续控制。
默认 0.2 秒未收到按键便发布零速度；操作系统首次按键的重复延迟可能导致短暂停顿。
正常退出及 ROS 关闭时发送零速度；进程被强制终止后由底盘 0.25 秒指令超时保护停止。
请将终端焦点保持在节点上。

可通过私有参数调整：

```bash
rosrun dingo_test keyboard_teleop.py \
  _cmd_vel_topic:=/dingo/cmd_vel \
  _linear_speed:=0.2 \
  _angular_speed:=0.5 \
  _publish_rate:=20.0 \
  _key_timeout:=0.2
```

线速度单位 m/s，角速度单位 rad/s。发布频率及按键超时单位分别为 Hz 和秒。
节点启动时只发布零速度，收到运动按键后才发布运动指令。
