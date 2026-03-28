cmake_minimum_required(VERSION 3.8)
project(d455_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp)
find_library(REALSENSE2_LIBRARY realsense2)

if(REALSENSE2_INCLUDE_DIR AND REALSENSE2_LIBRARY)
  add_executable(d455_camera_node src/d455_camera_node.cpp)
  ament_target_dependencies(d455_camera_node rclcpp sensor_msgs)
  target_include_directories(d455_camera_node PRIVATE ${REALSENSE2_INCLUDE_DIR})
  target_link_libraries(d455_camera_node ${REALSENSE2_LIBRARY})

  install(TARGETS d455_camera_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "librealsense2 development files not found; skipping d455_camera_node build.")
endif()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
ament_package()
