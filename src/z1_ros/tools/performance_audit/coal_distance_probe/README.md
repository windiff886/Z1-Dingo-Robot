# 持久 Coal 距离查询对照

本工具对比 `coal::distance` 和逐几何对持久 `coal::ComputeDistance`。两者均使用默认 `DefaultGuess`、默认数值容差和有符号距离；不使用 `CachedGuess`，不更新 request 的 guess。每次调用使用新的 request/result。持久对象只复用求解器存储与几何类型分派。

```bash
cmake -S src/z1_ros/tools/performance_audit/coal_distance_probe \
  -B /tmp/z1-coal-distance-probe -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build /tmp/z1-coal-distance-probe -j2
/tmp/z1-coal-distance-probe/probe \
  src/z1_ros/z1_description/urdf/z1_control.urdf
```

依赖使用当前 SDK 的 Pinocchio 4 / Coal 3.0.3；路径可以替换。该目录有独立 CMake，不修改上级构建。`operator new` 计数包含第三方内部构造分配，不计 C malloc 或硬件设备。模型构造和持久求解器创建不在查询计时内。

程序对模型中不同连杆的几何对，使用 300 个限位内随机构型和 2000 个连续构型比较两种查询结果，输出最大距离误差、查询耗时和分配计数。它包含生产中排除的相邻连杆几何对，以覆盖更多形状组合，不复刻生产 AABB 粗筛；查询耗时不能直接当作整步控制耗时。CPU 调度、运行频率和依赖版本都会影响测量结果。
