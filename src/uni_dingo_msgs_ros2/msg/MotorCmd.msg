# 电机命令消息（对齐 ROS1 unitree_legged_msgs/MotorCmd）
# 仅保留 IOROS 中实际使用的字段，省略未使用的 reserve

uint8 mode           # 电机目标模式
float32 q            # 目标位置 (rad)
float32 dq           # 目标速度 (rad/s)
float32 tau          # 目标力矩 (N*m)
float32 kp           # 弹簧刚度系数
float32 kd           # 阻尼系数
