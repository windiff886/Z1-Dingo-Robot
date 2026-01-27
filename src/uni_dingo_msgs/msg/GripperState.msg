# GripperState.msg
# State of a gripper

float64 position      # Current position (0=closed, 1=open or radians)
float64 velocity      # Current velocity
float64 effort        # Current effort/force
bool is_grasping      # Whether gripper is grasping an object
bool stalled          # Whether gripper is stalled
