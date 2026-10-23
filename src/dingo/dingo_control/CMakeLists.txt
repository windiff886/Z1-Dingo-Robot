cmake_minimum_required(VERSION 3.0.2)
project(dingo_control)

find_package(catkin REQUIRED)

catkin_package()

# Install launch, configuration and deployment files
install(DIRECTORY launch config deployment
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install CAN/UDP bridge script
install(PROGRAMS scripts/can-udp-bridge.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
