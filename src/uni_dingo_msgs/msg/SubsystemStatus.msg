# SubsystemStatus.msg
# Status of a single subsystem

uint8 STATUS_UNKNOWN = 0
uint8 STATUS_OK = 1
uint8 STATUS_WARNING = 2
uint8 STATUS_ERROR = 3
uint8 STATUS_DISABLED = 4

string name           # Subsystem name (base, left_arm, right_arm, head, perception)
uint8 status          # Current status
bool enabled          # Whether subsystem is enabled
bool ready            # Whether subsystem is ready for commands
string message        # Human-readable status message
