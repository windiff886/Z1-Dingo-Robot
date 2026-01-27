cmake_minimum_required(VERSION 3.0.2)
project(uni_dingo_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf2_ros
  uni_dingo_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    actionlib
    actionlib_msgs
    tf2_ros
    uni_dingo_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/robot_state_publisher.py
  scripts/test_api.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
