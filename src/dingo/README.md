# Dingo-O 底盘

ROS 1 Noetic 下 Dingo-O 四轮全向底盘的驱动、控制、模型和里程计融合。

## 启动

```bash
roslaunch dingo_control dingo.launch
```

启动前安装各包 `package.xml` 中的依赖，构建工作区并加载 `devel/setup.bash`。通信使用主机地址 `192.168.131.1` 和 MCU 地址 `192.168.131.2`；CAN/UDP 桥接服务通过 socat 将 UDP 数据接到伪终端，再由 slcand 创建 `vcan0`，底盘驱动通过该接口访问电机。

桥接由 systemd 管理，部署文件是 [can-udp-bridge.service](dingo_control/deployment/can-udp-bridge.service) 和 [can-udp-bridge.sh](dingo_control/scripts/can-udp-bridge.sh)。服务文件从 `/usr/local/bin/can-udp-bridge.sh` 启动脚本。先部署并启动桥接服务，再运行 `dingo.launch`。

## 启动内容

### 1. 底盘模型

用 Xacro 展开 [dingo.urdf.xacro](dingo_description/urdf/dingo.urdf.xacro)，写入 `robot_description`，提供底盘结构、四轮关节和安装坐标系。`robot_state_publisher` 由整机启动层运行；整机已加载完整模型时，传 `load_description:=false`，避免覆盖它。

### 2. 驱动与 MCU 通信

启动 `dingo_base` 的 `dingo_node`，加载[电机参数](dingo_control/config/hardware/base.yaml)。驱动通过 `vcan0` 配置四个 Puma 电机、读取车轮反馈、下发速度命令；齿轮比 24，编码器 CPR 10，速度环 PID 为 `0.025 / 0.005 / 0`。

MCU 消息通过 rosserial 传输：`dingo_node` 在 `11411` 端口建立 UDP session，接收 `imu/data_raw`、`mcu/status` 等消息；`11413` 端口接收 MCU 日志。节点主循环为 25 Hz，依次读取 CAN、更新关节、运行 controller_manager、发送命令和请求数据。

该节点还根据电池、电机和 MCU 状态切换灯光，按速度阈值调节风扇，并发布 `diagnostics`。

### 3. 四轮控制器

`dingo.launch` 内部包含 [control.launch](dingo_control/launch/control.launch)，读取[控制器配置](dingo_control/config/control_omni.yaml)，用 controller_spawner 把两个控制器加载到 `dingo_node` 的 `controller_manager`：

- **关节状态控制器**：25 Hz 发布四个车轮的关节状态。
- **麦克纳姆速度控制器**：把前向、侧向、旋转速度换成四轮速度，并根据轮速算轮式里程计，25 Hz 发布。

配置的速度上限为前向、侧向各 `1.3 m/s`，偏航 `4.0 rad/s`；对应加速度上限为 `1.0 m/s²` 和 `2.0 rad/s²`。`cmd_vel` 超过 `0.25 s` 未更新时，控制器将目标速度置零。

### 4. IMU 安装变换

默认发布 `base_link → imu_link` 静态变换，位置和角度初值都是零，可用 `imu_x/y/z`、`imu_roll/pitch/yaw` 改。整机已经发布这条变换时，传 `publish_imu_tf:=false`。

### 5. 融合底盘运动状态

启动 `robot_localization` 的 EKF，读取[融合配置](dingo_control/config/robot_localization.yaml)，在二维平面里融合：

- 轮式里程计的前向速度、侧向速度和偏航角速度。
- 底盘 IMU 的偏航角速度。

EKF 以 50 Hz 输出融合后的位姿和速度。默认不发布 `odom → base_link`，需要底盘提供这条变换时，启动命令加 `publish_odom_tf:=true`。

## 启动参数

`dingo.launch` 的主要参数及默认值：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `cmd_vel_topic` | `/dingo/cmd_vel` | 外部速度指令，`geometry_msgs/Twist` |
| `odom_topic` | `/dingo/odom` | EKF 融合后的里程计 |
| `odom_raw_topic` | `/dingo/odom_raw` | 轮式里程计原始输出 |
| `imu_raw_topic` | `/dingo/imu/data_raw` | 底盘 IMU |
| `diagnostics_topic` | `/diagnostics` | 诊断输出 |
| `ekf_config` | `dingo_control/config/robot_localization.yaml` | EKF 配置文件 |
| `load_description` | `true` | 是否加载 `robot_description` |
| `publish_odom_tf` | `false` | EKF 是否发布 `odom → base_link` |
| `publish_imu_tf` | `true` | 是否发布 `base_link → imu_link` |
| `imu_x/y/z`、`imu_roll/pitch/yaw` | `0` | IMU 安装位置和角度 |
| `imu_frame` | `imu_link` | IMU 坐标系名 |
| `wireless_interface` | 环境变量 `DINGO_WIRELESS_INTERFACE`，缺省 `wlp2s0` | 诊断用的无线网卡 |

## 依赖与源码

`dingo_base` 提供硬件驱动，`dingo_control` 组织启动和配置，`dingo_description` 提供模型，`dingo_msgs` 提供 MCU 状态、风扇和灯光消息。Puma 电机驱动及消息包、麦克纳姆控制器插件和 ROS 通用组件由运行环境安装。

`dingo_msgs` 版本 `0.3.1`，来自官方 [dingo 仓库](https://github.com/dingo-cpr/dingo/tree/68806fdffabfd612a3414b7e8d6cb19c496f8a0e/dingo_msgs)，固定提交 `68806fdffabfd612a3414b7e8d6cb19c496f8a0e`。消息定义和构建文件保持上游原样，许可证见 [LICENSE](dingo_msgs/LICENSE)。
