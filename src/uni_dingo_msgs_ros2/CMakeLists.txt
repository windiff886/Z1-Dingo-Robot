cmake_minimum_required(VERSION 3.8)
project(uni_dingo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/BaseState.msg"
  "msg/ArmState.msg"
  "msg/HeadState.msg"
  "msg/GripperState.msg"
  "msg/SubsystemStatus.msg"
  "srv/GetRobotState.srv"
  "srv/SetEnabled.srv"
  "srv/EmergencyStop.srv"
  "action/MoveArm.action"
  "action/MoveBase.action"
  "action/MoveHead.action"
  "action/MoveGripper.action"
  "action/WholeBodyMove.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_package()
