# ArmState.msg
# State of a single arm

std_msgs/Header header

string side                           # "left" or "right"

# Joint state
string[] joint_names
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts

# End effector
geometry_msgs/PoseStamped end_effector_pose

# Gripper
uni_dingo_msgs/GripperState gripper

# Status
bool enabled
bool moving
bool in_collision
bool at_limit
string current_named_pose             # "home", "ready", "tucked", or ""
