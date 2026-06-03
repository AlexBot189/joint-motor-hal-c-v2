cmake_minimum_required(VERSION 3.10)
project(motor_tool LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 依赖 motor_hal 库
# 如果独立构建, 需要指定 motor_hal 的头文件和库路径
# 如果作为 motor_hal_c 子目录, 自动继承

set(MOTOR_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

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
)

target_include_directories(motor_tool PRIVATE
    ${MOTOR_HAL_DIR}/inc
    ${MOTOR_HAL_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(motor_tool PRIVATE
    ${MOTOR_HAL_DIR}/build/libmotor_hal.a
    pthread m
)

target_compile_options(motor_tool PRIVATE -Wall -Wextra -O2)
