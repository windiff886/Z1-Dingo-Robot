cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  serial
  roscpp
  std_msgs
  pan_tilt_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pan_tilt_driver
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(PanTiltDriverNode
  src/PanTiltDriverNode.cpp
  src/PanTiltDriver.cpp
  src/QThread.cpp
  src/modbus_rtu_master.cpp
)  
target_link_libraries(PanTiltDriverNode
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
add_dependencies(PanTiltDriverNode pan_tilt_msgs_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  add_executable(pan_tilt_lifecycle_test
    tests/lifecycle_test.cpp src/PanTiltDriver.cpp src/QThread.cpp)
  target_include_directories(pan_tilt_lifecycle_test BEFORE PRIVATE tests/stubs)
  target_link_libraries(pan_tilt_lifecycle_test ${CMAKE_THREAD_LIBS_INIT})
  add_test(NAME pan_tilt_lifecycle COMMAND pan_tilt_lifecycle_test)
  set_tests_properties(pan_tilt_lifecycle PROPERTIES TIMEOUT 10)
  add_executable(pan_tilt_serial_startup_test
    tests/serial_startup_test.cpp src/modbus_rtu_master.cpp)
  target_include_directories(pan_tilt_serial_startup_test BEFORE PRIVATE tests/stubs)
  target_link_libraries(pan_tilt_serial_startup_test ${CMAKE_THREAD_LIBS_INIT})
  add_test(NAME pan_tilt_serial_startup COMMAND pan_tilt_serial_startup_test)
  set_tests_properties(pan_tilt_serial_startup PROPERTIES TIMEOUT 10)
endif()
