cmake_minimum_required(VERSION 3.10)
project(motor_hal_c VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 库类型: ON=动态库(.so)  OFF=静态库(.a)
option(BUILD_AS_SHARED "Build motor_hal as shared library" ON)

# ============================================================================
# 库
# ============================================================================

set(MOTOR_HAL_SOURCES
    src/can_driver.c
    src/canopen_frames.c
    src/sdo_client.c
    src/nmt_master.c
    src/heartbeat.c
    src/pdo_handler.c
    src/pdo_mapper.c
    src/feedback_parser.c
    src/motor_hal_startup.c
    src/utils.c
    src/motor_hal.c
)

if(BUILD_AS_SHARED)
    add_library(motor_hal SHARED ${MOTOR_HAL_SOURCES})
    message(STATUS "Building motor_hal as SHARED library")
else()
    add_library(motor_hal STATIC ${MOTOR_HAL_SOURCES})
    message(STATUS "Building motor_hal as STATIC library")
endif()

target_include_directories(motor_hal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(motor_hal PUBLIC pthread m)
target_compile_options(motor_hal PRIVATE -Wall -Wextra -O2)

# 安装
install(TARGETS motor_hal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY inc/ DESTINATION include/motor_hal)

# ============================================================================
# 示例
# ============================================================================

add_executable(motor_example_single   examples/single_motor.c)
target_link_libraries(motor_example_single motor_hal)

add_executable(motor_example_dual     examples/dual_motor.c)
target_link_libraries(motor_example_dual motor_hal)

add_executable(motor_example_mit      examples/mit_control.c)
target_link_libraries(motor_example_mit motor_hal)

add_executable(motor_example_dual_full examples/dual_motor_full.c)
target_link_libraries(motor_example_dual_full motor_hal m)

add_executable(motor_example_fb_cb    examples/feedback_callback.c)
target_link_libraries(motor_example_fb_cb motor_hal m)

# ============================================================================
# 选项
# ============================================================================

option(BUILD_EXAMPLES "Build examples" ON)
