# D455 Camera

Intel RealSense D455 深度相机的 ROS 1 驱动封装。

## 原理

基于 Intel RealSense SDK 2.0 (librealsense2)，通过 USB 3.0 连接 D455 相机，采集彩色流与深度流。通过 SDK 官方 `rs2::align(RS2_STREAM_COLOR)` 将深度图实时对齐到目标彩色相机视角，并发布对应的共享内参信息。

- **内存复制与传输说明**：本驱动在采集线程中将 librealsense 缓冲区数据复制为 ROS 消息结构；同进程内的零拷贝仅在 nodelet 架构下向同一进程内的订阅节点传递消息指针时生效，跨进程订阅仍需要序列化与网络/IPC 传输。
- **时钟策略**：严格采用 SDK `GLOBAL_TIME`（将设备时间关联到主机系统时间），启动与重连时检查实际帧时间域并预热；不支持 `/use_sim_time=true`。这不意味着 RGB、深度或其他设备已经硬件同步。
- **TF 责任说明**：本驱动不自带 TF 发布器，消息统一附带配置的 `color_frame_id`。相关的坐标变换必须由外部 URDF 与 `robot_state_publisher` 提供。

## 节点与发布话题

### 基础数据话题

默认命名空间为 `/d455`。驱动仅发布以下 3 个基础数据话题（点云、IMU、红外流及未对齐的原始深度不属于本驱动输出范围）：

| 默认完整话题 | 类型 | 编码 / 说明 |
|---|---|---|
| `/d455/color/image_raw` | `sensor_msgs/Image` | `BGR8` 格式原始彩色图像，携带彩色帧采样 global time |
| `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `16UC1` 格式对齐深度图像（单位：毫米，0 表示无效值），携带深度帧采样 global time |
| `/d455/color/camera_info` | `sensor_msgs/CameraInfo` | 目标彩色视角的共享相机标定（Latched，随彩色源帧及重连更新） |

> **注**：对齐到彩色视角的深度图像像素坐标系与内参完全对应目标彩色相机。深度消费者应直接订阅并使用 `/d455/color/camera_info`。两路图像均使用统一的 `color_frame_id`。

两路图像保留各自源帧的采样时间，按各自源帧号去重。CameraInfo 是共享标定：初始化和重连预热后以实际彩色帧时间发布，随后随彩色帧更新；深度消费者无需按相同时间戳等待它。不同帧率下，空间对齐不保证两路图像同时采样。对齐后的无效区域仍为 0。

## 启动方式

### 1. Nodelet 方式（推荐）

通过 launch 文件加载 nodelet manager 及驱动 nodelet：

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

如需修改命名空间：

```bash
roslaunch d455_camera d455_camera_nodelet.launch camera:=my_d455
```

### 2. 独立可执行节点方式

```bash
rosrun d455_camera d455_camera_node
```

独立节点在 ROS 初始化前设置默认 `/d455` 命名空间，因此标准相对话题重映射生效。显式 `__ns` 或 `ROS_NAMESPACE` 优先于默认值，包括显式指定根命名空间：

```bash
rosrun d455_camera d455_camera_node __ns:=my_d455
```

## 参数配置（C++ 变量形式）

驱动配置已由 ROS 参数服务器转为纯 **C++ 变量/结构体**（见 `src/camera_config.h` 中的 `CameraConfig` 结构体）。如需修改配置，可直接修改该头文件中的变量值：

| C++ 变量名 | 类型 | 默认值 | 说明与校验 |
|---|---|---|---|
| `serial_no` | std::string | `""` | 指定相机序列号（为空时自动绑定首个可用设备并在运行期锁定） |
| `enable_color` | bool | `true` | 是否启用彩色流（必须为 true，不支持单流运行） |
| `enable_depth` | bool | `true` | 是否启用深度流（必须为 true，不支持单流运行） |
| `color_width` | int | `640` | 彩色图像宽度（必须为正整数） |
| `color_height` | int | `480` | 彩色图像高度（必须为正整数） |
| `color_fps` | int | `30` | 彩色流采样帧率（必须为正整数） |
| `depth_width` | int | `640` | 深度图像宽度（必须为正整数） |
| `depth_height` | int | `480` | 深度图像高度（必须为正整数） |
| `depth_fps` | int | `30` | 深度流采样帧率（必须为正整数） |
| `color_frame_id` | std::string | `"d455_color_optical_frame"` | 两路图像共用的 TF frame；非空，不能以 `/` 开头或包含空白 |
| `global_time_timeout` | double | `5.0` | 设备启动/重连后等待两路 global time 就绪的单调超时时间（秒） |
| `frame_wait_timeout_ms` | int | `100` | 采集循环中单次等待新帧的最长时间（毫秒） |
| `capture_stall_timeout` | double | `5.0` | 任一流持续没有有效新帧后进入恢复；深度需成功对齐才算有效（秒） |
| `reconnect_interval` | double | `1.0` | 失败重连尝试之间的退避间隔；第一次尝试立即执行（秒） |
| `max_reconnect_attempts` | int | `5` | 故障恢复最大重启尝试次数 |
各项参数的配置规范与取值约束已在 `src/camera_config.h` 的结构体注释中逐条详细说明。

## 故障与标定处理

采集和预热使用 `poll_for_frames()` 配合可取消等待，避免 SDK 的等待接口隐式重启 pipeline。重复帧、无效时间戳、无帧、SDK 异常或持续对齐失败均受双流停滞检测约束；时间域丢失、帧号重置或明显时间回退立即进入恢复。

首次选中的相机序列号会固定下来，恢复时不会切换到另一台相机。每次恢复都会重新准备 global time、profile、CameraInfo、深度单位和 align，并清空帧去重状态。重试耗尽后停止采集、释放 pipeline 并输出错误日志；需要卸载/重新加载驱动才会再次尝试，不关闭整个 nodelet manager。

预热、轮询和退避可响应停止请求。SDK 自身的设备枚举、配置解析、启动及停止可能阻塞，`frame_wait_timeout_ms` 不代表整个卸载的耗时上限。

标定支持 `NONE`、普通 `BROWN_CONRADY`（5 项 `plumb_bob`）和 `KANNALA_BRANDT4`（4 项 `equidistant`）。Modified/Inverse Brown–Conrady 仅在全部畸变系数为零时接受；非零系数或其他不兼容模型会明确报错并回滚，不直接替换模型名称。深度按行处理 SDK stride，转换为毫米后四舍五入；正有效值至少为 1，舍入结果超过 65535 时置 0。

## 项目集成与验证

`unidingo_bringup/launch/real_robot.launch` 的 `color_frame_id` 默认传入 `head_realsense_color_optical_frame`，与整机模型中 `prefix="head"` 的 URDF 相机光学 frame 命名一致。单独启动驱动时的默认 frame 仍为 `d455_color_optical_frame`。相机模型位于 URDF 的 PanTilt 条件中；若启动时设置 `head:=false`，外部需要另外提供相机匹配 TF，或同时设置 `perception:=false`。

应用程序直接订阅驱动发布的 3 个基础话题（`/d455/color/image_raw`、`/d455/aligned_depth_to_color/image_raw`、`/d455/color/camera_info`）。驱动不提供点云、IMU、红外或相机控制服务；如需 RGB-D 配对，消费者应根据消息时间戳与标定信息自行实现同步。

整改期间已执行模拟边界回归及 SDK 头文件语法检查。测试文件、测试替身和临时构建产物已按要求删除，不在仓库保留测试入口。上述结果不等同于 ROS 1 完整构建或 D455 实机验证，详见根目录 `D455_CAMERA_REVIEW.md` 的交付记录。

## 依赖说明

- `librealsense2` (Intel RealSense SDK 2.0)
- ROS 1 核心与消息包：
  - `roscpp`
  - `nodelet`
  - `pluginlib`
  - `sensor_msgs`
  - `image_transport`
