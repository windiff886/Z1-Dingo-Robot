uint8 DISABLED=0
uint8 HOLDING=1
uint8 JOINT_GOAL=2
uint8 POSE_GOAL=3
uint8 JOINT_STREAM=4
uint8 POSE_STREAM=5
uint8 IMPEDANCE=6
uint8 STOPPING=7
uint8 AWAIT_RESUME=8
uint8 FAULT=9
uint8 JOINT_TRAJECTORY=10
uint8 CARTESIAN_PATH=11
std_msgs/Header header
uint8 mode
string reason
uint64 session_id
uint64 active_command_id
uint64 feedback_sequence
bool feedback_valid
bool new_feedback
float64 feedback_age
bool has_gripper
# Legacy fields: collision_distance is NaN and collision_pair is empty.
# Geometry queries belong to the planning layer.
float64 collision_distance
string collision_pair
float64 compute_time
uint64 overruns
float64 loop_period
float64 max_loop_period
string model_revision
# Host runtime timing; seconds. A fresh UDP packet is not an MCU sample clock.
uint64 runtime_cycles
uint64 missed_control_slots
float64 exchange_time
float64 max_exchange_time
float64 wakeup_lateness
float64 max_wakeup_lateness
bool realtime_scheduling
bool cpu_affinity_applied
int32 realtime_error
int32 cpu_affinity_error
# Executed finite trajectory duration (seconds) and uniform retiming factor.
float64 trajectory_duration
float64 trajectory_time_scale
