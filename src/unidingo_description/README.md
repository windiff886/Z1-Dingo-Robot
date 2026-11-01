# Unidingo Description

这是 Dingo 底盘、两台 Unitree Z1 机械臂、Pan-Tilt 云台和相机的描述资源包。目录名是 `unidingo_description`，ROS 包名是 `uni_dingo`。

本包提供 URDF、Xacro 和网格资源。[unidingo_rviz_test](../z1_ros/unidingo_rviz_test/README.md) 使用这些资源加载整机模型、发布 TF，并提供双臂 TCP 交互测试。

## 目录

```text
unidingo_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── meshes/
│   ├── base_link.STL
│   ├── leftarm_link.STL
│   └── rightarm_link.STL
└── urdf/
    ├── uni_dingo_dual_arm.urdf.xacro
    ├── z1_arm.urdf.xacro
    ├── uni_dingo.urdf
    ├── d455_only.urdf.xacro
    ├── d455_visual.urdf.xacro
    ├── sensors.gazebo.xacro
    └── 宇树机械臂+dingo.csv
```

| 文件 | 用途 |
| --- | --- |
| `urdf/uni_dingo_dual_arm.urdf.xacro` | 整机模型：包括底盘、双臂、支撑结构，以及可选的云台和相机 |
| `urdf/z1_arm.urdf.xacro` | Z1 机械臂的宏，支持名称前缀、父 link 和可选夹爪 |
| `urdf/uni_dingo.urdf` | 由原始 CAD 导出的底盘和左右安装结构，引用本包的三个 STL；它不是完整的整机模型 |
| `urdf/d455_only.urdf.xacro` | 独立的相机模型，复用外部的 D435 描述 |
| `urdf/d455_visual.urdf.xacro` | 整机使用的本地相机外壳和 `d455_color_optical_frame`，不依赖外部相机宏 |
| `urdf/sensors.gazebo.xacro` | 独立的相机描述宏，包含几何、惯量、光学坐标系和 Gazebo 传感器；主整机模型没有引用它 |
| `urdf/宇树机械臂+dingo.csv` | 由 CAD 导出的质量、惯量、几何和关节参数，作为源数据保留 |

## 依赖

- `xacro`：展开模型。
- `dingo_description`：底盘描述。
- `z1_description`：机械臂常量和网格。
- `pan_tilt_description`：云台描述和网格。
- `realsense2_description`：独立相机模型引用的描述。

展开模型时，ROS 环境必须能够找到相应的描述包。`realsense2_description` 只用于旧的独立相机入口；整机相机使用的是本地的示意模型。

## 展开模型

在已经安装依赖并加载工作区环境的终端中展开整机模型：

```bash
rosrun xacro xacro "$(rospack find uni_dingo)/urdf/uni_dingo_dual_arm.urdf.xacro" > /tmp/uni_dingo.urdf
```

`UnitreeGripper` 和 `PanTilt` 两个参数默认都是 `true`；关闭它们时，模型会分别去掉夹爪和云台分支。安装位置、支撑柱长度等参数都写在整机 Xacro 中。

## 坐标系与模型范围

左右臂各有一个固定的 `tool` 坐标系，带 `left_` / `right_` 前缀，位于各自 `link06` 的 X 正方向 0.051 m 处，姿态保持不变。它和默认 `z1_control.urdf` 的 TCP 一致，夹爪的活动指与它无关。`unidingo_rviz_test/tests/test_model_contract.py` 会离线核对这部分。

安装位置、支撑结构和各组件之间的固定连接来自 CAD 和手工装配，尚未经过整机标定。整机相机外壳和光学中心只用于示意；如果需要用相机和机械臂联合定位，必须通过标定确定实际的相机外参。独立相机入口复用的 D435 描述与整机示意模型也有所不同。
