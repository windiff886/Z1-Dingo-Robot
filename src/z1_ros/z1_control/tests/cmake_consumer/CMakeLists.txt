cmake_minimum_required(VERSION 3.16)
project(z1_core_consumer_check LANGUAGES CXX)
set(Z1_BUILD_ROS OFF CACHE BOOL "" FORCE)
set(Z1_BUILD_CORE ON CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
get_filename_component(Z1_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_subdirectory("${Z1_SOURCE_DIR}" z1_control)
# A sibling client exercises the public dependency scope of z1_core.
add_subdirectory(client)
