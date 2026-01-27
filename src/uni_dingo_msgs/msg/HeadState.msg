# HeadState.msg
# State of the pan-tilt head

std_msgs/Header header

# Current angles (radians)
float64 pan
float64 tilt

# Velocity
float64 pan_velocity
float64 tilt_velocity

# Limits
float64 pan_min
float64 pan_max
float64 tilt_min
float64 tilt_max

# Status
bool enabled
bool moving
