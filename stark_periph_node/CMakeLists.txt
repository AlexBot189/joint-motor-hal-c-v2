cmake_minimum_required(VERSION 3.10)
project(stark_periph_manager_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ROS "Build ROS adapter" OFF)
option(ENABLE_WEBSERVER "Build WebServer adapter" OFF)

set(MOTOR_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../motor_hal")
set(IMU_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../imu_hal")

# 拉入 motor_hal 和 imu_hal 作为构建依赖
add_subdirectory(${IMU_HAL_DIR} ${CMAKE_BINARY_DIR}/imu_hal)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_loop
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shm
    ${CMAKE_CURRENT_SOURCE_DIR}/src/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/log_helper
    ${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party
    ${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/usr/include
    ${MOTOR_HAL_DIR}/inc
    ${MOTOR_HAL_DIR}/src
    ${IMU_HAL_DIR}/inc
)

link_directories(
    ${MOTOR_HAL_DIR}/build
    ${IMU_HAL_DIR}/build
    ${CMAKE_CURRENT_SOURCE_DIR}/src/log_helper/build
)

set(SOURCES
    src/main.cpp
    src/main_loop/main_loop.cpp

    src/motor/motor_init.cpp
    src/motor/motor_state.cpp
    src/motor/motor_rt_worker.cpp
    src/motor/motor_ctrl.cpp

    src/imu/imu_sensor.cpp

    src/utils/latency_trace.cpp
    src/utils/factory.cpp
    src/shm/shm_mgr.cpp
)

set(HEADERS
    src/stark_shm.h
    src/main_loop/main_loop.h

    src/motor/motor_init.h
    src/motor/motor_state.h
    src/motor/motor_context.h
    src/motor/motor_rt_worker.h
    src/motor/motor_ctrl.h

    src/imu/imu_sensor.h

    src/utils/rt_log.h
    src/utils/latency_trace.h
    src/utils/factory.h
    src/shm/shm_mgr.h

    src/interface/Defines.hpp
    src/interface/IListener.hpp
    src/interface/IMsgInternalDispatcher.hpp
)

add_executable(stark_periph_manager_node ${SOURCES} ${HEADERS})

target_link_libraries(stark_periph_manager_node
    motor_hal
    imu_hal
    log_helper
    pthread
    rt
)

if(ENABLE_ROS)
    find_package(roscpp REQUIRED)
    find_package(std_msgs REQUIRED)
    include_directories(${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
    target_sources(stark_periph_manager_node PRIVATE src/ros/RosAdapter.cpp)
    target_link_libraries(stark_periph_manager_node ${roscpp_LIBRARIES})
    add_definitions(-DENABLE_ROS)
endif()

if(ENABLE_WEBSERVER)
    target_sources(stark_periph_manager_node PRIVATE src/web/WebServer.cpp)
    add_definitions(-DENABLE_WEBSERVER)
endif()
