cmake_minimum_required(VERSION 3.8)
project(pan_tilt_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pan_tilt_msgs REQUIRED)
# Pure C++ driver library (no ROS dependency)
add_library(pan_tilt_core STATIC
  src/PanTiltDriver.cpp
  src/modbus_rtu_master.cpp
  src/QThread.cpp
)
target_include_directories(pan_tilt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pan_tilt_core
  pthread
)

# ROS2 driver node
add_executable(pan_tilt_driver_node
  src/pan_tilt_driver_node.cpp
)
target_include_directories(pan_tilt_driver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pan_tilt_driver_node
  pan_tilt_core
)
ament_target_dependencies(pan_tilt_driver_node
  rclcpp
  sensor_msgs
  pan_tilt_msgs
)

install(TARGETS pan_tilt_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
