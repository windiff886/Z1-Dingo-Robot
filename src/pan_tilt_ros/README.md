# pan_tilt_ros

这是 IQR 第四代云台的 ROS 驱动。设备手册见[在线文档](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## 环境

- Ubuntu 20.04 + ROS 1 Noetic（当前工作区）
- `serial` 包：`sudo apt install ros-noetic-serial`

驱动通过 Modbus RTU 串口与设备通信。`serial` 已经在 CMake 中使用，但还没有写入 `package.xml`，因此需要单独安装。

## 构建

在工作区根目录执行：

```bash
catkin_make
source devel/setup.bash
```

## 串口权限

先把 udev 规则复制到系统目录：

```bash
roscd pan_tilt_bringup/config/
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```

规则会按 USB VID/PID `2341:8037` 匹配设备，并创建 `/dev/pan_tilt` 符号链接。复制完成后重新插拔 USB 线即可生效。

## 启动

驱动启动时会发送一条回中命令：`yaw=0°`、`pitch=0°`，速度为 `10 deg/s`。接好电源和 USB 后运行：

```bash
roslaunch pan_tilt_bringup panTilt_view.launch
```

这个入口会启动驱动、加载云台 URDF，并运行 `robot_state_publisher` 和 RViz。如果只想运行驱动、不打开模型，请使用 `panTilt_bringup.launch`。

如果串口打开失败，驱动会启动失败，错误会通过启动日志报告。正常退出时，驱动会先停止采集线程并等待它结束，然后再释放串口。

串口线程会更新状态缓存，每轮读取后等待 10 ms。ROS 主循环的目标频率是 50 Hz，它负责发布缓存；首份有效的设备状态到达后才开始发布，因此消息时间戳表示的是发布时间。上面的 launch 使用串口 `/dev/pan_tilt`，波特率为 115200，设备 ID 为 1。如果要更换串口，请运行：

```bash
roslaunch pan_tilt_bringup panTilt_bringup.launch port_name:=/dev/ttyUSB0
```

## 控制

### 角度制

```bash
rostopic pub -1 /pan_tilt_cmd_deg pan_tilt_msgs/PanTiltCmdDeg "yaw: 0.0
pitch: 0.0
speed: 10"
```

`yaw` 和 `pitch` 的单位是度，范围是 `[-60, 60]`；`speed` 的类型是 `int16`，单位是 deg/s，取值范围为 1～30。超出范围的消息会被拒绝，日志中会给出允许区间。

### 弧度制

```bash
rostopic pub -1 /pan_tilt_cmd_rad pan_tilt_msgs/PanTiltCmdRad "yaw: 0.0
pitch: 0.0
speed: 0.2"
```

`yaw` 和 `pitch` 的单位是 rad，接收范围是 `[-1.0471974, 1.0471974]`；`speed` 的单位是 rad/s，接收范围是 `[0.01745329, 0.5235987]`。驱动会把速度换算后截断为整数 deg/s，因为设备执行速度的粒度是 1 deg/s。

## 反馈

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/pan_tilt_status` | `pan_tilt_msgs/PanTiltStatus` | 设备 ID、版本、速度、目标角度和当前角度（单位：度）、温度、原始编码器值、错误码 |
| `/joint_states` | `sensor_msgs/JointState` | 云台两个轴的角度（单位：rad），关节名由 launch 设置为 `pan_tilt_yaw_joint` 和 `pan_tilt_pitch_joint` |

关节名在 [panTilt_bringup.launch](pan_tilt_bringup/launch/panTilt_bringup.launch) 中配置。如果直接 `rosrun` 而不传私有参数，串口默认使用 `/dev/ttyUSB0`，关节名则是 `iqr_pan_tilt_yaw_joint` 和 `iqr_pan_tilt_pitch_joint`。
