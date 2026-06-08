cmake_minimum_required(VERSION 3.10)
project(stark_periph_manager_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 选项
option(ENABLE_ROS "Build ROS adapter" OFF)
option(ENABLE_WEBSERVER "Build WebServer adapter" OFF)

# 依赖路径
set(MOTOR_HAL_DIR "${CMAKE_SOURCE_DIR}/..")

# 包含路径
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/interface
    ${MOTOR_HAL_DIR}/inc
)

# 链接路径
link_directories(
    ${MOTOR_HAL_DIR}/build
)

# 源文件
set(SOURCES
    src/main.cpp
    src/CanDispatcher.cpp
    src/exo_state_machine.cpp
    src/exo_rt_worker.cpp
    src/exo_shm_mgr.cpp
    src/Factory.cpp
)

# 头文件 (用于 IDE 索引, 不影响编译)
set(HEADERS
    exo_shm.h
    src/exo_shm_mgr.h
    src/CanDispatcher.h
    src/exo_state_machine.h
    src/exo_rt_worker.h
    src/Factory.h
    interface/Defines.hpp
    interface/IListener.hpp
    interface/IMsgInternalDispatcher.hpp
)

add_executable(stark_periph_manager_node ${SOURCES} ${HEADERS})

target_link_libraries(stark_periph_manager_node
    motor_hal
    motor_calib
    pthread
    rt
)

# ROS 模块 (编译可选)
if(ENABLE_ROS)
    find_package(roscpp REQUIRED)
    find_package(std_msgs REQUIRED)
    include_directories(${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
    target_sources(stark_periph_manager_node PRIVATE ros/RosAdapter.cpp)
    target_link_libraries(stark_periph_manager_node ${roscpp_LIBRARIES})
    add_definitions(-DENABLE_ROS)
endif()

# WebServer 模块 (编译可选)
if(ENABLE_WEBSERVER)
    # 预留
endif()
