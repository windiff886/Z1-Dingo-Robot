# D455 相机

本包基于 ROS 1 Noetic 和 librealsense2，采集一台 D455 相机的彩色图和深度图，并把深度图对齐到彩色图像的视角。

## 启动

先构建工作区并加载 `devel/setup.bash`，然后执行：

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

启动文件会在 `/d455` 命名空间下创建 nodelet manager，并加载相机 nodelet。如果需要使用其他命名空间，可以在命令末尾加上 `camera:=my_d455`。

## 采集流程

1. 分别打开 `640 × 480`、`30 FPS` 的 BGR8 彩色流和 Z16 深度流，两路都启用 GLOBAL_TIME。
2. 从彩色流读取内参并生成 `CameraInfo`；从深度传感器读取深度单位，供毫米换算使用。
3. 用一个 5 ms 的 `WallTimer` 轮询 SDK 的帧队列。只有彩色帧和深度帧都进入 GLOBAL_TIME 时间戳域后，才开始处理这一组帧。轮询使用主机的 wall clock，消息时间戳则取自 SDK 帧。
4. 每收到一帧彩色图像就发布一次 latched 的 `CameraInfo`。只有存在图像订阅者时才发布对应图像；深度图先经过 `rs2::align(RS2_STREAM_COLOR)` 对齐，再换算成毫米。
5. 卸载 nodelet 时停止定时器，由 SDK 的管线对象释放相机。

彩色图和深度图各自保留 SDK 给出的时间戳。像素对齐只是让两路图像共用彩色内参，并不会把两次采集变成同一时刻。

驱动没有实现相机重连。如果相机断开或采集出现异常，需要重新执行启动命令才能恢复采集。

## 输出

| 默认话题 | 内容 |
| --- | --- |
| `/d455/color/image_raw` | BGR8 彩色图 |
| `/d455/aligned_depth_to_color/image_raw` | 对齐到彩色视角的 16UC1 深度图，单位毫米 |
| `/d455/color/camera_info` | 彩色相机内参，随彩色帧更新，并保留最近一条给新订阅者 |

深度值会先换算成毫米并四舍五入。如果原始值为 0，或者舍入后超过 65535 mm，就输出 0 表示无效；其余值至少为 1 mm。

彩色图保留 SDK 的行步长；深度图按源帧的行步长读取，输出时逐行紧密存储。

三种消息的坐标系都是 `d455_color_optical_frame`。它到机器人其他坐标系的变换由整机模型和 `robot_state_publisher` 提供。

## 配置与依赖

采集参数定义在 [camera_config.h](src/camera_config.h) 中：`width`、`height` 和 `fps` 同时用于彩色流和深度流，`color_frame_id` 指定共用的光学坐标系。修改后需要重新构建。

本包依赖 librealsense2，以及 ROS 的 `roscpp`、`nodelet`、`pluginlib`、`sensor_msgs` 和 `image_transport`。
