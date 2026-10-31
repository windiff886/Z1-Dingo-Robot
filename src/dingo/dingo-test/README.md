# Dingo 键盘测试

ROS 1 包名 `dingo_test`，目录名 `dingo-test`。节点向 `/dingo/cmd_vel` 发 `geometry_msgs/Twist`，支持全向平移和旋转。

## 构建与运行

在 ROS 1 环境里，从工作区根目录构建：

```bash
catkin_make
source devel/setup.bash
```

一个终端启动底盘：

```bash
roslaunch dingo_control dingo.launch
```

另一个终端，同样加载工作区环境，运行：

```bash
rosrun dingo_test keyboard_teleop.py
```

将键盘焦点放在运行节点的终端，节点从标准输入读取按键。

## 按键

| 按键 | 动作 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 向左 / 向右平移 |
| Q / E | 逆时针 / 顺时针旋转 |
| U / I | 左前 / 右前 |
| J / K | 左后 / 右后 |
| 空格或其他非运动键 | 停车 |
| X / Ctrl-C | 退出 |

方向按底盘坐标系：X 向前、Y 向左，逆时针为正旋转。斜向平移经过归一化，合成速度不超过设定的线速度。大小写按键效果相同。

按住运动键，通过终端的按键重复持续发送目标；多键同时按下时以最近读到的键为准。默认 0.2 秒没有新按键就发布零速度。操作系统首次按键的重复延迟可能造成短暂停顿。

正常退出和 ROS 关闭时，节点会发布零速度。进程被强制结束后，由底盘控制器的 0.25 秒 `cmd_vel` 超时将目标速度归零；实际减速由底盘控制器和机械响应决定。

## 参数

```bash
rosrun dingo_test keyboard_teleop.py \
  _cmd_vel_topic:=/dingo/cmd_vel \
  _linear_speed:=0.2 \
  _angular_speed:=0.5 \
  _publish_rate:=20.0 \
  _key_timeout:=0.2
```

线速度单位 m/s，角速度 rad/s，发布频率 Hz，按键超时秒。节点启动时先发零速度，收到运动键后才发运动指令。
