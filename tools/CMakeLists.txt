cmake_minimum_required(VERSION 3.10)
project(motor_tool LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 依赖 motor_hal 库
# 如果独立构建, 需要指定 motor_hal 的头文件和库路径
# 如果作为 motor_hal_c 子目录, 自动继承

set(MOTOR_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

# 库类型 (与根 CMakeLists.txt 保持一致)
option(BUILD_AS_SHARED "Link against shared motor_hal" ON)
set(MOTOR_HAL_BUILD_DIR "${MOTOR_HAL_DIR}/build" CACHE PATH "motor_hal build directory")

if(BUILD_AS_SHARED)
    set(MOTOR_HAL_LIB ${MOTOR_HAL_BUILD_DIR}/libmotor_hal.so)
else()
    set(MOTOR_HAL_LIB ${MOTOR_HAL_BUILD_DIR}/libmotor_hal.a)
endif()

add_executable(motor_tool
    main.c
    command_registry.c
    tool_hal.c
    daemon.c
    cmd_control.c
    cmd_read.c
    cmd_system.c
    cmd_watch.c
    cmd_help.c
    cmd_sensor.c
    cmd_report.c
    cmd_calib.c
    cmd_pdo.c
    cmd_rpdo.c
    cmd_pdo_ctrl.c
)

target_include_directories(motor_tool PRIVATE
    ${MOTOR_HAL_DIR}/inc
    ${MOTOR_HAL_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(motor_tool PRIVATE
    ${MOTOR_HAL_LIB}
    pthread m
)

target_compile_options(motor_tool PRIVATE -Wall -Wextra -O2)
