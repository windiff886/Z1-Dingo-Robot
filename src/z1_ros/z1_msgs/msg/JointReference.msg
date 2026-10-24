std_msgs/Header header
uint64 session_id
uint64 sequence
# Desired sample time on the ROS clock; zero means header.stamp.
time target_time
float64[6] positions
bool has_velocity
float64[6] velocities
bool has_acceleration
float64[6] accelerations
