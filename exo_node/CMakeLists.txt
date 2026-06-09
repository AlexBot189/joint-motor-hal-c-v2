cmake_minimum_required(VERSION 3.10)
project(stark_periph_manager_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ROS "Build ROS adapter" OFF)
option(ENABLE_WEBSERVER "Build WebServer adapter" OFF)

set(MOTOR_HAL_DIR "${CMAKE_SOURCE_DIR}/..")

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/interface
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/log_helper
    ${CMAKE_SOURCE_DIR}/3rd_party
    ${CMAKE_SOURCE_DIR}/3rd_party/usr/include
    ${MOTOR_HAL_DIR}/inc
)

link_directories(
    ${MOTOR_HAL_DIR}/build
    ${CMAKE_SOURCE_DIR}/log_helper/build
)

set(SOURCES
    src/main.cpp
    src/CanDispatcher.cpp
    src/Factory.cpp
    src/exo_shm_mgr.cpp

    core/exo_state_machine.cpp
    core/exo_rt_worker.cpp
    core/exo_motor_ctrl.cpp
    core/exo_mock_sensor.cpp
)

set(HEADERS
    exo_shm.h
    src/exo_shm_mgr.h
    src/CanDispatcher.h
    src/Factory.h

    core/exo_state_machine.h
    core/exo_node_context.h
    core/exo_rt_worker.h
    core/exo_rt_log.h
    core/exo_motor_ctrl.h
    core/exo_mock_sensor.h

    interface/Defines.hpp
    interface/IListener.hpp
    interface/IMsgInternalDispatcher.hpp
)

add_executable(stark_periph_manager_node ${SOURCES} ${HEADERS})

target_link_libraries(stark_periph_manager_node
    motor_hal
    log_helper
    pthread
    rt
    nlohmann_json
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
