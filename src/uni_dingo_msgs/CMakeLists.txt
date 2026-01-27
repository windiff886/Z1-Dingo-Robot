cmake_minimum_required(VERSION 3.0.2)
project(uni_dingo_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Message files
add_message_files(
  FILES
  RobotState.msg
  BaseState.msg
  ArmState.msg
  HeadState.msg
  GripperState.msg
  SubsystemStatus.msg
)

# Service files
add_service_files(
  FILES
  GetRobotState.srv
  SetEnabled.srv
  EmergencyStop.srv
)

# Action files
add_action_files(
  FILES
  MoveBase.action
  MoveArm.action
  MoveGripper.action
  MoveHead.action
  WholeBodyMove.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)
