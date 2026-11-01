# One fresh MCU feedback packet. Monotonic seconds belong to the controller host.
# command_* is the previous output actually sent, not a firmware acknowledgment.
std_msgs/Header header
uint64 feedback_sequence
float64 received_at
# Host call boundary before exchange; not a kernel/device transmission timestamp.
float64 sent_at
bool feedback_valid
bool command_enabled
bool has_gripper
float64 gripper_position
bool has_temperature
int8[8] temperature
uint8 mode
uint64 active_command_id
uint64 dropped_samples
float64[6] position
float64[6] velocity
float64[6] effort
float64[6] gravity
float64[6] command_position
float64[6] command_velocity
float64[6] command_effort
float64[6] command_kp
float64[6] command_kd
bool friction_compensation_enabled
# Requested feedforward before the combined host torque/rate limits.
float64[6] friction_feedforward
