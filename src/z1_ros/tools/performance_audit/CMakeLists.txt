cmake_minimum_required(VERSION 3.16)
project(z1_performance_audit LANGUAGES CXX)

# This target runs entirely offline; it does not instantiate McuTransport.
set(Z1_BUILD_ROS OFF CACHE BOOL "Build ROS adapter" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build existing tests" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../z1_control" z1_control)

add_executable(allocation_audit allocation_audit.cpp)
target_link_libraries(allocation_audit PRIVATE z1_core)
