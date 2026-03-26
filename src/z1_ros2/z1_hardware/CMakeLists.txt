cmake_minimum_required(VERSION 3.8)
project(z1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# z1_sdk third-party library
set(Z1_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

add_library(${PROJECT_NAME} SHARED
  src/z1_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# z1_sdk headers are SYSTEM to suppress warnings from third-party code
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  $<BUILD_INTERFACE:${Z1_SDK_DIR}/include>
)
target_link_directories(${PROJECT_NAME} PUBLIC
  ${Z1_SDK_DIR}/lib
)
target_link_libraries(${PROJECT_NAME}
  unitree_arm_sdk
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface z1_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

# Install the z1_sdk shared library so it can be found at runtime
install(FILES ${Z1_SDK_DIR}/lib/libunitree_arm_sdk.so
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
