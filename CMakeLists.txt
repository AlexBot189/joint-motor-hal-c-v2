cmake_minimum_required(VERSION 3.10)
project(joint-motor-hal-v2 C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

option(BUILD_EXAMPLES "Build motor_hal examples" ON)
option(BUILD_AS_SHARED "Build as shared library" ON)

add_subdirectory(motor_hal)
add_subdirectory(imu_hal)
add_subdirectory(exo_node)
