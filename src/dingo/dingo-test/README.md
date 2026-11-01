# Dingo 键盘测试

这个 ROS 1 包的包名是 `dingo_test`，目录名是 `dingo-test`。节点向 `/dingo/cmd_vel` 发布 `geometry_msgs/Twist` 消息，支持全向平移和旋转。

## 构建与运行

在 ROS 1 环境中，从工作区根目录构建：

```bash
catkin_make
source devel/setup.bash
```

先在一个终端启动底盘：

```bash
roslaunch dingo_control dingo.launch
```

再打开另一个终端，同样加载工作区环境，然后运行：

```bash
rosrun dingo_test keyboard_teleop.py
```

把键盘焦点放在运行节点的终端上，节点会从标准输入读取按键。

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

方向按底盘坐标系定义：X 轴向前、Y 轴向左，逆时针旋转为正。斜向平移会做归一化，合成速度不超过设定的线速度。按键不区分大小写。

按住运动键时，节点依靠终端的按键重复持续发送目标；同时按下多个键时，以最近读到的键为准。默认情况下，0.2 秒没有新按键就发布零速度。操作系统首次按键的重复延迟可能造成短暂停顿。

正常退出和 ROS 关闭时，节点都会发布零速度。如果进程被强制结束，底盘控制器会在 `cmd_vel` 超过 0.25 秒未更新时把目标速度归零；实际减速过程由底盘控制器和机械响应决定。

## 参数

```bash
rosrun dingo_test keyboard_teleop.py \
  _cmd_vel_topic:=/dingo/cmd_vel \
  _linear_speed:=0.2 \
  _angular_speed:=0.5 \
  _publish_rate:=20.0 \
  _key_timeout:=0.2
```

线速度单位为 m/s，角速度单位为 rad/s，发布频率单位为 Hz，按键超时单位为秒。节点启动时会先发布零速度，收到运动键后才发送运动指令。
