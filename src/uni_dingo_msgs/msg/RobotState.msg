# RobotState.msg
# Complete state of the Uni-Dingo robot

std_msgs/Header header

# Operating mode
string mode                           # "simulation" or "real"

# Subsystem states
uni_dingo_msgs/BaseState base
uni_dingo_msgs/ArmState left_arm
uni_dingo_msgs/ArmState right_arm
uni_dingo_msgs/HeadState head

# Overall status
uni_dingo_msgs/SubsystemStatus[] subsystem_statuses
bool emergency_stopped
bool all_ready

# Timestamp
time last_update
