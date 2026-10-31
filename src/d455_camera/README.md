# D455 相机

基于 ROS 1 Noetic 和 librealsense2，采集一台 D455 的彩色图和深度图，并将深度图对齐到彩色视角。

## 启动

构建工作区、加载 `devel/setup.bash`，然后：

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

启动文件在 `/d455` 命名空间创建 nodelet manager 并加载相机 nodelet。需要更换命名空间时，在命令末尾加 `camera:=my_d455`。

## 采集流程

1. 打开 `640 × 480、30 FPS` 的 BGR8 彩色流和 Z16 深度流，两路都启用 GLOBAL_TIME。
2. 从彩色流读取内参，生成 `CameraInfo`；从深度传感器读取深度单位，供毫米换算使用。
3. 用 5 ms 的 `WallTimer` 轮询 SDK 帧队列。彩色和深度都进入 GLOBAL_TIME 时间戳域后，开始处理帧组。轮询使用主机 wall clock，消息时间戳取自 SDK 帧。
4. 随彩色帧发布 latched `CameraInfo`。有图像订阅者时才发布对应图像；深度图先经 `rs2::align(RS2_STREAM_COLOR)` 对齐，再换算为毫米。
5. 卸载 nodelet 时停止定时器，由 SDK 管线对象释放相机。

彩色图和深度图保留各自的 SDK 时间戳。像素对齐使两路共用彩色内参，不会把两次采集变成同一时刻。

驱动没有相机重连流程。相机断开或采集异常后，重新执行启动命令恢复采集。

## 输出

| 默认话题 | 内容 |
| --- | --- |
| `/d455/color/image_raw` | BGR8 彩色图 |
| `/d455/aligned_depth_to_color/image_raw` | 对齐到彩色视角的 16UC1 深度图，单位毫米 |
| `/d455/color/camera_info` | 彩色相机内参，随彩色帧更新，保留最近一条给新订阅者 |

深度先换算为毫米并四舍五入。原始值为 0，或舍入后超过 65535 mm，输出 0，表示无效；其余值至少为 1 mm。

彩色图保留 SDK 的行步长；深度图按源帧行步长读取，输出时逐行紧密存储。

三种消息的坐标系都是 `d455_color_optical_frame`。它到机器人其他坐标系的变换由整机模型和 `robot_state_publisher` 提供。

## 配置与依赖

采集参数在 [camera_config.h](src/camera_config.h)：`width`、`height`、`fps` 同时用于彩色和深度，`color_frame_id` 指定共用光学坐标系。改完重新构建。

依赖 librealsense2，以及 ROS 的 `roscpp`、`nodelet`、`pluginlib`、`sensor_msgs`、`image_transport`。
