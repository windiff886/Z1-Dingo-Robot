cmake_minimum_required(VERSION 3.0.2)
project(dingo_test)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME dingo_keyboard COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_keyboard_teleop.py)
endif()
