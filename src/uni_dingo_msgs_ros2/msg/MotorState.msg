# 电机状态消息（对齐 ROS1 unitree_legged_msgs/MotorState）
# 仅保留 IOROS 中实际使用的字段，省略未使用的 q_raw/dq_raw/ddq_raw/reserve

uint8 mode           # 电机当前模式
float32 q            # 当前位置 (rad)
float32 dq           # 当前速度 (rad/s)
float32 ddq          # 当前加速度 (rad/s²)
float32 tau_est      # 估计输出力矩 (N*m)
int8 temperature     # 电机温度
