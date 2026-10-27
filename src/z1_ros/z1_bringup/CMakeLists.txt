cmake_minimum_required(VERSION 3.0.2)
project(z1_bringup)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/publish_description.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
