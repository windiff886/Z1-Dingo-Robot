cmake_minimum_required(VERSION 3.0.2)
project(z1_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  control_msgs
  actionlib
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    control_msgs
    actionlib
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main control node
add_executable(z1_control_node src/z1_control_node.cpp)
target_link_libraries(z1_control_node ${catkin_LIBRARIES})
add_dependencies(z1_control_node ${catkin_EXPORTED_TARGETS})

install(TARGETS z1_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
