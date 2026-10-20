cmake_minimum_required(VERSION 3.16)
project(z1_transport_preflight LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The bundled Unitree libraries require Linux")
endif()
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|aarch64)$")
  message(FATAL_ERROR "No bundled library for ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/unitree" ABSOLUTE)
add_library(unitree_transport_vendor SHARED IMPORTED)
set_target_properties(unitree_transport_vendor PROPERTIES
  IMPORTED_LOCATION "${VENDOR_DIR}/lib/libUnitreeArm_${CMAKE_SYSTEM_PROCESSOR}.so"
)

# The real IOUDP and LowLevelState implementations remain in the vendor library.
# Only UdpPort is replaced in this executable; it never opens a socket.
add_executable(transport_contract_probe transport_contract_probe.cpp)
target_compile_features(transport_contract_probe PRIVATE cxx_std_17)
target_include_directories(transport_contract_probe PRIVATE
  "${VENDOR_DIR}/include" "${VENDOR_DIR}/include/IOInterface" ${Boost_INCLUDE_DIRS}
)
target_link_libraries(transport_contract_probe PRIVATE
  unitree_transport_vendor Eigen3::Eigen Threads::Threads
)
target_link_options(transport_contract_probe PRIVATE "-Wl,--export-dynamic")

enable_testing()
add_test(NAME vendor_transport_contract COMMAND transport_contract_probe)
set_tests_properties(vendor_transport_contract PROPERTIES TIMEOUT 10)
