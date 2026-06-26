cmake_minimum_required(VERSION 3.10)
project(stark_periph_manager_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ROS "Build ROS adapter" OFF)
option(ENABLE_WEBSERVER "Build WebServer adapter" OFF)

set(MOTOR_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../motor_hal")
set(IMU_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../imu_hal")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/main_loop
    ${CMAKE_CURRENT_SOURCE_DIR}/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/shm
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/log_helper
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/usr/include
    ${MOTOR_HAL_DIR}/inc
    ${MOTOR_HAL_DIR}/src
    ${IMU_HAL_DIR}/inc
)

link_directories(
    ${MOTOR_HAL_DIR}/build
    ${IMU_HAL_DIR}/build
    ${CMAKE_CURRENT_SOURCE_DIR}/log_helper/build
)

set(SOURCES
    src/main.cpp
    main_loop/main_loop.cpp

    motor/motor_init.cpp
    motor/motor_state.cpp
    motor/motor_rt_worker.cpp
    motor/motor_ctrl.cpp

    imu/imu_sensor.cpp

    utils/latency_trace.cpp
    utils/factory.cpp
    shm/shm_mgr.cpp
)

set(HEADERS
    exo_shm.h
    main_loop/main_loop.h

    motor/motor_init.h
    motor/motor_state.h
    motor/motor_context.h
    motor/motor_rt_worker.h
    motor/motor_ctrl.h

    imu/imu_sensor.h

    utils/rt_log.h
    utils/latency_trace.h
    utils/factory.h
    shm/shm_mgr.h

    interface/Defines.hpp
    interface/IListener.hpp
    interface/IMsgInternalDispatcher.hpp
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
    target_sources(stark_periph_manager_node PRIVATE ros/RosAdapter.cpp)
    target_link_libraries(stark_periph_manager_node ${roscpp_LIBRARIES})
    add_definitions(-DENABLE_ROS)
endif()

if(ENABLE_WEBSERVER)
    target_sources(stark_periph_manager_node PRIVATE web/WebServer.cpp)
    add_definitions(-DENABLE_WEBSERVER)
endif()
