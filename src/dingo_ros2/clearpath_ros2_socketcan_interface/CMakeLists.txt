cmake_minimum_required(VERSION 3.8)
project(clearpath_ros2_socketcan_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_library(socketcan_interface src/socketcan_interface.cpp)
add_library(clearpath_ros2_socketcan_interface::socketcan_interface ALIAS socketcan_interface)
ament_target_dependencies(socketcan_interface
  can_msgs
  rclcpp
)
target_compile_features(socketcan_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(socketcan_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(socketcan_interface PRIVATE "CLEARPATH_ROS2_SOCKETCAN_INTERFACE_BUILDING_LIBRARY")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS socketcan_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
  scripts/activate_lifecycle
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright # TODO: Fix this!
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  socketcan_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
