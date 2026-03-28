cmake_minimum_required(VERSION 3.8)
project(z1_controller_ros2 VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")

# ── 依赖 ─────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uni_dingo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(pinocchio REQUIRED)

# ── 头文件路径 ────────────────────────────────────────────────────────────────
# 包含从 z1_controller 原样复制过来的所有头文件
include_directories(
  include                          # 用于 #include "z1_controller_ros2/io_ros2.hpp"
  include/z1_controller_ros2       # 用于 FSM/IOInterface/UnitreeArmModule 等旧头文件
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ── 链接目录（预编译 FSM 库）────────────────────────────────────────────────
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# ── 可执行文件 ────────────────────────────────────────────────────────────────
add_executable(z1_ctrl
  src/z1_ctrl_node.cpp
  src/io_ros2.cpp
)

target_link_libraries(z1_ctrl
  UnitreeArm_${CMAKE_SYSTEM_PROCESSOR}   # libUnitreeArm_x86_64.so 或 aarch64
  pinocchio::pinocchio
  ${Boost_LIBRARIES}
)

ament_target_dependencies(z1_ctrl
  rclcpp
  uni_dingo_msgs
  ament_index_cpp
)

# ── 安装 ──────────────────────────────────────────────────────────────────────
install(TARGETS z1_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装预编译库，确保运行时可以找到
install(DIRECTORY lib/
  DESTINATION lib/${PROJECT_NAME}/lib
)

ament_package()
