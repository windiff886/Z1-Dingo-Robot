# Unidingo Description

Dingo 底盘、双 Unitree Z1 机械臂、Pan-Tilt 云台和相机的机器人描述资源包。
目录名为 `unidingo_description`，ROS 包名为 `uni_dingo`。

本包只维护模型资源和必要的包构建信息，不提供启动文件、控制器/PID 配置、
RViz 配置或运行脚本。启动、可视化和控制编排应由调用方的功能包负责。

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
    ├── sensors.gazebo.xacro
    └── 宇树机械臂+dingo.csv
```

| 文件 | 用途 |
|------|------|
| `urdf/uni_dingo_dual_arm.urdf.xacro` | 整机组合模型：底盘、双臂、支撑结构、可选云台及相机 |
| `urdf/z1_arm.urdf.xacro` | 支持名称前缀、父 link 和可选夹爪的 Z1 机械臂宏 |
| `urdf/uni_dingo.urdf` | 原始 CAD 导出的底盘及左右安装结构模型，引用本包三个 STL；不是完整整机模型 |
| `urdf/d455_only.urdf.xacro` | 独立相机模型，当前复用外部 D435 描述 |
| `urdf/sensors.gazebo.xacro` | 独立相机描述宏，含几何、惯量、光学坐标系及 Gazebo 传感器扩展；主整机模型未引用它 |
| `urdf/宇树机械臂+dingo.csv` | CAD 导出的质量、惯量、几何及关节参数，作为模型源数据保留 |

URDF/Xacro 中已有的 transmission 和 Gazebo 标签属于模型扩展，本次保留。

## 描述依赖

- `xacro`：展开 Xacro 模型。
- `dingo_description`：整机模型使用的底盘描述。
- `z1_description`：机械臂常量与网格。
- `pan_tilt_description`：云台描述与网格。
- `realsense2_description`：独立相机模型所引用的描述。

外部描述包需要在 ROS 环境中可被找到。本包不复制这些组件的模型资源。

## 使用模型

在具备 ROS 1 和上述依赖的环境中，可由调用方展开整机模型：

```bash
rosrun xacro xacro "$(rospack find uni_dingo)/urdf/uni_dingo_dual_arm.urdf.xacro" > /tmp/uni_dingo.urdf
```

整机入口保留 `UnitreeGripper`（默认 `true`）和 `PanTilt`（默认 `true`）参数。
安装位置和支撑结构参数仍定义在整机 Xacro 内。

## 当前范围

本次仅精简包内容，所有 URDF/Xacro、网格和 CAD 参数文件保持原样。
组件尺寸、组件间固定连接的相对位置及姿态尚未校准；独立相机模型与整机相机描述
也未统一。这些内容需在后续模型校准工作中单独检查。
