# D455 相机

本包只保存 D455 的启动配置。驱动使用 Intel 官方的 `realsense2_camera`，源码放在同一工作区的 [`src/realsense2_camera`](../realsense2_camera)，取自 [realsense-ros](https://github.com/IntelRealSense/realsense-ros) 的 `2.3.2` 标签（提交 `f400d682`，Apache-2.0）。这份源码和 `ros-noetic-realsense2-camera` 2.3.3 二进制包是同一版本，未做修改。

## 启动

先构建工作区并加载 `devel/setup.bash`，然后执行：

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

启动文件会 include 官方的 `rs_camera.launch`，在 `/d455` 命名空间下创建 nodelet manager 并加载驱动。可选参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `camera` | `d455` | 命名空间，也是坐标系名前缀 |
| `width` / `height` | `640` / `480` | 彩色流和深度流共用的分辨率 |
| `fps` | `30` | 彩色流和深度流共用的帧率 |

其余参数使用官方默认值。需要调整时，直接在 `launch/d455_camera_nodelet.launch` 里给 `rs_camera.launch` 增加对应的 `<arg>`。

## 输出

| 默认话题 | 内容 |
| --- | --- |
| `/d455/color/image_raw` | 彩色图 |
| `/d455/color/camera_info` | 彩色相机内参 |
| `/d455/aligned_depth_to_color/image_raw` | 对齐到彩色视角的 16UC1 深度图，单位毫米 |
| `/d455/aligned_depth_to_color/camera_info` | 对齐深度图的内参，与彩色相机相同 |

官方驱动还会发布未对齐的 `/d455/depth/*` 和诊断、元数据等话题。

## 坐标系

彩色图和对齐深度图的坐标系都是 `d455_color_optical_frame`。启动文件关闭了驱动自带的 TF（`publish_tf:=false`），这个坐标系到机器人其他部分的变换由整机模型 `unidingo_description/urdf/d455_visual.urdf.xacro` 和 `robot_state_publisher` 提供。

## 畸变参数

D455 彩色相机在 SDK 里报告的畸变模型是 Inverse Brown-Conrady。官方驱动不区分模型，把 5 个系数原样作为 `plumb_bob` 发布，本包沿用这一行为。社区实测认为这组系数实际上是正向 Brown-Conrady，但 Intel 没有正式确认，相关讨论见 [librealsense#7335](https://github.com/IntelRealSense/librealsense/issues/7335) 和 [realsense-ros#2458](https://github.com/IntelRealSense/realsense-ros/issues/2458)。对精度要求高时，建议用棋盘格重新标定彩色相机核对。
