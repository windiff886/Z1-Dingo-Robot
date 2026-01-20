cmake_minimum_required(VERSION 3.0.2)
project(d455_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nodelet
  pluginlib
  roscpp
  sensor_msgs
)

find_package(realsense2 REQUIRED)

catkin_package(
  LIBRARIES d455_camera_nodelet
  CATKIN_DEPENDS image_transport nodelet pluginlib roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIRS}
)

add_library(d455_camera_nodelet src/d455_camera_nodelet.cpp)
add_dependencies(d455_camera_nodelet ${catkin_EXPORTED_TARGETS})
if(TARGET realsense2::realsense2)
  target_link_libraries(d455_camera_nodelet ${catkin_LIBRARIES} realsense2::realsense2)
else()
  target_link_libraries(d455_camera_nodelet ${catkin_LIBRARIES} ${realsense2_LIBRARY} ${realsense2_LIBRARIES})
endif()

add_executable(d455_camera_node src/d455_camera_node.cpp)
add_dependencies(d455_camera_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(d455_camera_node ${catkin_LIBRARIES})

install(TARGETS d455_camera_nodelet d455_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
