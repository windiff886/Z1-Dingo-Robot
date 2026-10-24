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

对 Z1 URDF 的51组跨link几何对，测试300个随机限位内构型和2000个连续构型，每轮共117300次逐对结果比较，包含穿透。它有意包含生产中排除的相邻link几何对以覆盖更多shape组合，并不复刻生产AABB粗筛；性能数字是窄相位查询结果，不能直接当作整步控制收益。

`RESULTS.txt` 保留同机五轮原始结果。五轮均 `max_error=0`，随机查询耗时中位0.679296→0.603329微秒，连续0.638909→0.572860微秒；平均new/query约7.08→2.92。CPU调度和频率会影响墙钟耗时；依赖版本不同需要重新验证。仍有第三方内部残余分配。
