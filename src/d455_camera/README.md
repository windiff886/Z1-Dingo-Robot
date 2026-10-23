# D455 相机启动说明

本包使用 ROS 1 Noetic 和 librealsense2，采集单台 D455 的彩色图与深度图，将深度对齐到彩色相机视角后发布。

## 唯一启动入口

构建工作空间并加载 `devel/setup.bash` 后，通过 [d455_camera_nodelet.launch](launch/d455_camera_nodelet.launch) 启动：

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

入口在 `/d455` 命名空间启动 nodelet manager，并加载相机驱动 nodelet。命名空间可通过 `camera:=my_d455` 修改。

## 启动与采集流程

1. **配置相机**：选择连接的相机，同时开启 `640 × 480、30 FPS` 的 BGR8 彩色流和 Z16 深度流，启用两路传感器的 GLOBAL_TIME。
2. **读取标定**：从实际彩色流读取内参，生成 CameraInfo；从深度传感器读取深度单位，用于转换毫米深度。
3. **获取帧组**：由 nodelet 的串行 WallTimer 回调每 5 毫秒轮询 SDK 帧队列。SDK 完成设备时钟到主机系统时钟的映射后，开始发布两路图像，分别保留彩色帧和深度帧的采样时间。
4. **对齐与发布**：发布彩色图和 CameraInfo；有深度订阅者时，通过 `rs2::align(RS2_STREAM_COLOR)` 将深度图对齐到彩色视角，再转换为毫米单位发布。
5. **停止采集**：卸载 nodelet 时先停止定时回调，再由 SDK 管线对象释放相机资源。

驱动使用主机物理时间，运行时使用 `/use_sim_time=false`。空间对齐使两路图像使用相同的像素坐标和内参，不代表两路传感器同时曝光。

驱动按固定单台 D455 场景运行；采集过程中 SDK 报错直接退出，不做自动重连、看门狗重启或重试。需要重新采集时，重新运行上述启动入口。

## 输出内容

| 默认话题 | 内容 |
| --- | --- |
| `/d455/color/image_raw` | BGR8 彩色图 |
| `/d455/aligned_depth_to_color/image_raw` | 对齐到彩色视角的 16UC1 深度图，单位毫米，0 表示无效深度 |
| `/d455/color/camera_info` | 两路图像共用的彩色相机内参，随彩色帧更新，并保留最近一条消息供新订阅者使用 |

彩色图保留 SDK 的行步长；深度图按源帧行步长读取，转换后逐行紧密存储。深度转换后超出 16 位范围的值发布为 0。

三种消息均使用 `d455_color_optical_frame`。驱动负责发布传感器数据，该光学坐标系到机器人其他坐标系的变换由整机模型和 `robot_state_publisher` 提供。

## 配置与依赖

采集配置集中在 [camera_config.h](src/camera_config.h)：`width`、`height`、`fps` 同时用于彩色流和深度流，`color_frame_id` 指定共用光学坐标系。修改后重新构建。

依赖为 librealsense2，以及 ROS 的 `roscpp`、`nodelet`、`pluginlib`、`sensor_msgs` 和 `image_transport`。
