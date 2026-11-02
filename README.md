# Z1-Dingo 双臂机器人

本仓库包含 Dingo-O 底盘、两台 Unitree Z1 机械臂、Pan-Tilt 云台、D455 相机与整机模型。机械臂使用完整的**宇树官方 SDK 和控制器**，左右臂独立运行；原自研 Z1 控制栈已移除。

## Z1 双臂：原生 CMake 构建，不需要 ROS

```bash
cmake -S src/z1_sdk -B /tmp/z1-build \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/z1-build -j2

# 只检查配置，不连接设备
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py check

# 启动两套官方控制器，会连接真实机械臂
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py controllers
```

另开终端，选择一臂调用官方接口：

```bash
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py client --arm left
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py client --arm right
```

双臂网络、夹爪和负载配置见 [arms.yaml](src/z1_sdk/dual_arm/config/arms.yaml)。默认左臂 IP 为 `192.168.123.110`、带夹爪；右臂为 `192.168.123.111`、不带夹爪。两臂的 MCU 本地端口和 SDK 端口均独立。

启动客户端不自动回 Home。使用官方位姿和状态语义，不保留原自研 ROS Topic、摩擦辨识、标定工具或 RViz 控制预览接口。完整启动、C++/Python 调用和退出说明见 [Z1 SDK](src/z1_sdk/README.md)，详细源码解读见 [项目详解](src/z1_sdk/PROJECT_GUIDE.zh-CN.md)。

## 其他 ROS 1 组件

其他组件和描述包仍使用 ROS 1 Noetic 工作区。安装依赖后，在仓库根目录构建：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`src/z1_sdk` 是独立 CMake 工程，catkin 会跳过它；机械臂按上一节单独构建启动。

| 组件 | 入口与文档 |
| --- | --- |
| 底盘 | `roslaunch dingo_control dingo.launch`；需先建立 CAN/UDP 桥接，见 [Dingo](src/dingo/README.md) |
| 云台 | `roslaunch pan_tilt_bringup panTilt_view.launch`；连接串口并以 10 deg/s 回中，见 [Pan-Tilt](src/pan_tilt_ros/README.md) |
| 相机 | `roslaunch d455_camera d455_camera_nodelet.launch`，见 [D455](src/d455_camera/README.md) |
| 整机模型 | [Unidingo Description](src/unidingo_description/README.md) |
| 机械臂显示模型 | [z1_description](src/z1_description)；只提供模型与网格，不参与官方 SDK 控制计算 |
