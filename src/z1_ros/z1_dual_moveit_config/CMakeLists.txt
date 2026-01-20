cmake_minimum_required(VERSION 3.0.2)
project(z1_dual_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml" PATTERN "*.srdf" PATTERN "*.xml"
)
