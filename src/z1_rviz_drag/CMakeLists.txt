cmake_minimum_required(VERSION 3.0.2)
project(z1_rviz_drag)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/z1_rviz_drag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
