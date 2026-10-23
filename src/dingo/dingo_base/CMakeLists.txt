cmake_minimum_required(VERSION 3.0.2)
project(dingo_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_msgs diagnostic_updater dingo_msgs geometry_msgs hardware_interface
  puma_motor_driver puma_motor_msgs roscpp rosgraph_msgs rosserial_server
  sensor_msgs std_msgs)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(dingo_node
  src/dingo_base.cpp
  src/dingo_cooling.cpp
  src/dingo_diagnostic_updater.cpp
  src/dingo_hardware.cpp
  src/dingo_lighting.cpp
  src/dingo_logger.cpp)
target_link_libraries(dingo_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(dingo_node ${catkin_EXPORTED_TARGETS})

install(TARGETS dingo_node
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
