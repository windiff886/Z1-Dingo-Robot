cmake_minimum_required(VERSION 3.0.2)
project(whole_body_manipulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_cmd_vel src/keyboard_cmd_vel.cpp)
target_link_libraries(keyboard_cmd_vel ${catkin_LIBRARIES})
add_dependencies(keyboard_cmd_vel ${catkin_EXPORTED_TARGETS})
