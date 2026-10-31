add_executable(z1_core_client "${Z1_SOURCE_DIR}/tests/goal_planner_tests.cpp")
target_link_libraries(z1_core_client PRIVATE z1_core)
