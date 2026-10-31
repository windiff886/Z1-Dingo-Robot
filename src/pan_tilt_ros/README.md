# pan_tilt_ros

IQR 第四代云台的 ROS 驱动。设备手册见[在线文档](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## 环境

- Ubuntu 20.04 + ROS 1 Noetic（当前工作区）
- `serial` 包：`sudo apt install ros-noetic-serial`

驱动通过 Modbus RTU 串口通信。`serial` 已在 CMake 中使用，尚未写入 `package.xml`，因此需要单独安装。

## 构建

在工作区根目录：

```bash
catkin_make
source devel/setup.bash
```

## 串口权限

把 udev 规则复制到系统目录：

```bash
roscd pan_tilt_bringup/config/
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```

规则按 USB VID/PID `2341:8037` 匹配，创建 `/dev/pan_tilt` 符号链接。复制完重新插拔 USB 线生效。

## 启动

驱动启动时会发送 `yaw=0°`、`pitch=0°`、速度 `10 deg/s` 的回中命令。接好电源和 USB 后运行：

```bash
roslaunch pan_tilt_bringup panTilt_view.launch
```

这个入口启动驱动、加载云台 URDF，并运行 `robot_state_publisher` 和 RViz。运行驱动、不打开模型时，用 `panTilt_bringup.launch`。

串口打开失败时，驱动启动失败，错误由启动日志报告。正常退出时先停止采集线程并等待其结束，再释放串口。

串口线程更新状态缓存，每轮读取后等待 10 ms。ROS 主循环以 50 Hz 为目标频率发布缓存，首份有效设备状态到达后开始发布，因此消息时间戳表示发布时间。上述 launch 使用串口 `/dev/pan_tilt`，波特率 115200，设备 ID 1。更换串口时运行：

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

`yaw`、`pitch` 单位为度，范围 `[-60, 60]`；`speed` 为 `int16`，单位 deg/s，取值为 1～30。超出范围的消息会被拒绝，日志给出允许区间。

### 弧度制

```bash
rostopic pub -1 /pan_tilt_cmd_rad pan_tilt_msgs/PanTiltCmdRad "yaw: 0.0
pitch: 0.0
speed: 0.2"
```

`yaw`、`pitch` 单位为 rad，接收范围为 `[-1.0471974, 1.0471974]`；`speed` 单位 rad/s，接收范围为 `[0.01745329, 0.5235987]`。驱动将速度换算后截断为整数 deg/s，设备执行的速度粒度为 1 deg/s。

## 反馈

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/pan_tilt_status` | `pan_tilt_msgs/PanTiltStatus` | 设备 ID、版本、速度、目标角度和当前角度（度）、温度、原始编码器、错误码 |
| `/joint_states` | `sensor_msgs/JointState` | 云台两轴角度（rad），关节名由 launch 设为 `pan_tilt_yaw_joint` 和 `pan_tilt_pitch_joint` |

关节名在 [panTilt_bringup.launch](pan_tilt_bringup/launch/panTilt_bringup.launch) 中配置。直接 `rosrun`、不传私有参数时，串口使用 `/dev/ttyUSB0`，关节名为 `iqr_pan_tilt_yaw_joint` 和 `iqr_pan_tilt_pitch_joint`。
