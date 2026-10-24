std_msgs/Header header
uint64 session_id
uint64 sequence
# Desired sample time on the ROS clock; zero means header.stamp.
time target_time
geometry_msgs/Pose pose
# Base-frame linear/angular derivatives at the TCP, not body-frame derivatives.
bool has_velocity
geometry_msgs/Twist velocity
bool has_acceleration
geometry_msgs/Accel acceleration
