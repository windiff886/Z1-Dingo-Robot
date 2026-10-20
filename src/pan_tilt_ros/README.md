# pan_tilt_ros
IQR第四代云台ROS驱动，[在线手册](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## 环境要求
- Ubuntu 18.04
- ROS melodic

## 安装与编译
更新软件
```shell
sudo apt-get update
```
安装依赖项（串口驱动依赖 ROS serial 包）
```shell
sudo apt-get install ros-melodic-serial
```
创建ROS工作空间
```shell
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
克隆代码并编译
```shell
git clone https://github.com/I-Quotient-Robotics/pan_tilt_ros.git
cd ..
catkin_make
```
环境设置
```shell
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
添加串口rules文件
```shell
roscd pan_tilt_bringup/config/
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```
**注意：添加串口rules文件后，需要重新插拔USB线使之生效。**

## 使用
首先，连接好云台的电源和USB通信端口。

### 驱动启动与位置控制
启动驱动节点
```shell
roslaunch pan_tilt_bringup panTilt_view.launch
```
可通过 ROS 话题命令直接控制云台。

发送度制位置消息（speed 需为整数，单位 deg/s，范围 1~30）：
```shell
rostopic pub /pan_tilt_cmd_deg pan_tilt_msgs/PanTiltCmdDeg "yaw: 0.0
pitch: 0.0
speed: 10" 
```

发送弧度制位置消息（单位 rad，范围约 ±1.047 rad；速度单位 rad/s，范围 0.0175~0.5236）：
```shell
rostopic pub /pan_tilt_cmd_rad pan_tilt_msgs/PanTiltCmdRad "yaw: 0.0
pitch: 0.0
speed: 0.2"
```

### 状态反馈话题
- `/pan_tilt_status` (`pan_tilt_msgs/PanTiltStatus`): 驱动状态、目标与当前角度（度）
- `/joint_states` (`sensor_msgs/JointState`): 云台关节状态（弧度，包含 `pan_tilt_yaw_joint` 与 `pan_tilt_pitch_joint`）

