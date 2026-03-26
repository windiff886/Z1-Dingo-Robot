# BaseState.msg
# State of the mobile base

std_msgs/Header header

# Odometry
geometry_msgs/Pose pose              # Current pose in odom frame
geometry_msgs/Twist velocity         # Current velocity

# Status
bool enabled
bool moving
bool emergency_stopped

# Battery (if available)
float32 battery_voltage
float32 battery_percentage
