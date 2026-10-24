cmake_minimum_required(VERSION 3.16)
project(z1_performance_audit LANGUAGES CXX)

# This target runs entirely offline; it does not instantiate McuTransport.
set(Z1_BUILD_ROS OFF CACHE BOOL "Build ROS adapter" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build existing tests" FORCE)
# Pinocchio's interface contains generator expressions referring to this target.
# Discover it in the parent scope as well as the core's dependency scope.
find_package(coal REQUIRED)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../z1_control" z1_control)

add_executable(performance_audit performance_audit.cpp)
target_link_libraries(performance_audit PRIVATE z1_core)
target_compile_features(performance_audit PRIVATE cxx_std_17)
target_link_options(performance_audit PRIVATE
  "-Wl,--wrap=_ZN2z15Model9clearanceERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEE"
  "-Wl,--wrap=_ZN2z15Model9clearanceERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES5_"
  "-Wl,--wrap=_ZN2z15Model15clearanceSampleERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES5_")

add_executable(runtime_rate_probe runtime_rate_probe.cpp)
target_link_libraries(runtime_rate_probe PRIVATE z1_core)
add_executable(allocation_audit allocation_audit.cpp)
target_link_libraries(allocation_audit PRIVATE z1_core)
add_executable(trajectory_execution_audit trajectory_execution_audit.cpp)
target_link_libraries(trajectory_execution_audit PRIVATE z1_core)
