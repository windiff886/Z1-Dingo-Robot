# Unidingo Description

Dingo 底盘、双 Unitree Z1 机械臂、Pan-Tilt 云台和相机的机器人描述资源包。
目录名为 `unidingo_description`，ROS 包名为 `uni_dingo`。

本包只维护模型资源和必要的包构建信息，不提供启动文件、控制器/PID 配置、
RViz 配置或运行脚本。启动、可视化和控制编排应由调用方的功能包负责。
整机 RViz 末端目标测试入口位于 [unidingo_rviz_test](../z1_ros/unidingo_rviz_test/README.md)。

## 目录与模型

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
|------|------|
| `urdf/uni_dingo_dual_arm.urdf.xacro` | 整机组合模型：底盘、双臂、支撑结构、可选云台及相机 |
| `urdf/z1_arm.urdf.xacro` | 支持名称前缀、父 link 和可选夹爪的 Z1 机械臂宏 |
| `urdf/uni_dingo.urdf` | 原始 CAD 导出的底盘及左右安装结构模型，引用本包三个 STL；不是完整整机模型 |
| `urdf/d455_only.urdf.xacro` | 独立相机模型，当前复用外部 D435 描述 |
| `urdf/d455_visual.urdf.xacro` | 整机使用的本地相机外壳示意及 `d455_color_optical_frame`，无外部相机宏依赖 |
| `urdf/sensors.gazebo.xacro` | 独立相机描述宏，含几何、惯量、光学坐标系及 Gazebo 传感器扩展；主整机模型未引用它 |
| `urdf/宇树机械臂+dingo.csv` | CAD 导出的质量、惯量、几何及关节参数，作为模型源数据保留 |

URDF/Xacro 中的 transmission 和 Gazebo 标签用于模型扩展。

## 描述依赖

- `xacro`：展开 Xacro 模型。
- `dingo_description`：整机模型使用的底盘描述。
- `z1_description`：机械臂常量与网格。
- `pan_tilt_description`：云台描述与网格。
- `realsense2_description`：独立相机模型所引用的描述。

外部描述包需要在 ROS 环境中可被找到。本包不复制这些组件的模型资源。
`realsense2_description` 仅用于旧的独立相机入口，整机相机使用本地示意模型。

## 使用模型

在具备 ROS 1 和上述依赖的环境中，可由调用方展开整机模型：

```bash
rosrun xacro xacro "$(rospack find uni_dingo)/urdf/uni_dingo_dual_arm.urdf.xacro" > /tmp/uni_dingo.urdf
```

整机入口保留 `UnitreeGripper`（默认 `true`）和 `PanTilt`（默认 `true`）参数。
安装位置和支撑结构参数仍定义在整机 Xacro 内。

## 控制坐标系与当前范围

左右臂均有固定 `tool` 坐标系（带 `left_` / `right_` 前缀），位于各自 `link06`
的 X 正方向 0.051 m，姿态不变，与默认 `z1_control.urdf` 的 TCP 一致。
它不随夹爪活动指变化。该一致性由 `unidingo_rviz_test/tests/test_model_contract.py` 离线核对。

组件尺寸、组件间固定连接的相对位置及姿态尚未校准；独立相机模型与整机相机描述
仍有差异。整机相机外壳与光学中心为示意，不代表完成手眼标定。
这些内容需在后续模型校准工作中单独检查。
