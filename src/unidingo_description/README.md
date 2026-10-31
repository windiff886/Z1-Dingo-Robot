# Unidingo Description

Dingo 底盘、双 Unitree Z1 机械臂、Pan-Tilt 云台和相机的描述资源包。目录名 `unidingo_description`，ROS 包名 `uni_dingo`。

本包提供 URDF、Xacro 和网格资源。[unidingo_rviz_test](../z1_ros/unidingo_rviz_test/README.md) 使用这些资源加载整机模型，发布 TF，并提供双臂 TCP 交互测试。

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
| `urdf/uni_dingo_dual_arm.urdf.xacro` | 整机模型：底盘、双臂、支撑结构、可选云台和相机 |
| `urdf/z1_arm.urdf.xacro` | Z1 机械臂宏，支持名称前缀、父 link 和可选夹爪 |
| `urdf/uni_dingo.urdf` | 原始 CAD 导出的底盘和左右安装结构，引用本包三个 STL；不是完整整机模型 |
| `urdf/d455_only.urdf.xacro` | 独立相机模型，复用外部 D435 描述 |
| `urdf/d455_visual.urdf.xacro` | 整机用的本地相机外壳和 `d455_color_optical_frame`，不依赖外部相机宏 |
| `urdf/sensors.gazebo.xacro` | 独立相机描述宏，含几何、惯量、光学坐标系和 Gazebo 传感器；主整机模型没有引用 |
| `urdf/宇树机械臂+dingo.csv` | CAD 导出的质量、惯量、几何和关节参数，作为源数据保留 |

## 依赖

- `xacro`：展开模型。
- `dingo_description`：底盘描述。
- `z1_description`：机械臂常量和网格。
- `pan_tilt_description`：云台描述和网格。
- `realsense2_description`：独立相机模型引用的描述。

展开模型时，ROS 环境需要能找到相应描述包。`realsense2_description` 用于旧的独立相机入口；整机相机使用本地示意模型。

## 展开模型

在已安装依赖、加载工作区环境的终端中展开整机模型：

```bash
rosrun xacro xacro "$(rospack find uni_dingo)/urdf/uni_dingo_dual_arm.urdf.xacro" > /tmp/uni_dingo.urdf
```

`UnitreeGripper` 和 `PanTilt` 两个参数默认都是 `true`，关闭时分别去掉夹爪和云台分支。安装位置、支撑柱长度等参数写在整机 Xacro 里。

## 坐标系与模型范围

左右臂各有一个固定 `tool` 坐标系，带 `left_` / `right_` 前缀，位于各自 `link06` 的 X 正方向 0.051 m，姿态不变。它和默认 `z1_control.urdf` 的 TCP 一致，夹爪活动指和它无关。`unidingo_rviz_test/tests/test_model_contract.py` 会离线核对这部分。

安装位置、支撑结构和组件间的固定连接来自 CAD 和手工装配，尚未经过整机标定。整机相机外壳和光学中心用于示意；用于相机与机械臂联合定位时，需要通过标定确定实际相机外参。独立相机入口复用的 D435 描述与整机示意模型也有所不同。
