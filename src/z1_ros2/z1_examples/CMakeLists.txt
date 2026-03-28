cmake_minimum_required(VERSION 3.8)
project(z1_examples)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY examples
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
