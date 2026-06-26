cmake_minimum_required(VERSION 3.10)
project(imu-hal C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Toolchain (set via -DCMAKE_TOOLCHAIN_FILE for cross-compile)
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "Cross-compiling with ${CMAKE_TOOLCHAIN_FILE}")
endif()

# 目录定义
set(PROJECT_ROOT   ${CMAKE_CURRENT_SOURCE_DIR})
set(INC_DIR        ${PROJECT_ROOT}/inc)
set(SRC_DIR        ${PROJECT_ROOT}/src)
set(DRIVER_DIR     ${SRC_DIR}/driver)
set(IMU_DIR        ${DRIVER_DIR}/icm45608/imu)
set(ICT1531_DIR    ${DRIVER_DIR}/Ict1531x)
set(HAL_DIR        ${SRC_DIR}/hal)
set(TOOLS_DIR      ${SRC_DIR}/tools)
set(DEMO_DIR       ${PROJECT_ROOT}/demo)
set(EXAMPLE_DIR    ${PROJECT_ROOT}/example)

# 依赖
set(GPIOD_DIR ${HAL_DIR}/gpiod)

find_library(GPIOD_SYS gpiod)
if(GPIOD_SYS)
    set(GPIOD_LIB ${GPIOD_SYS})
    message(STATUS "gpiod: ${GPIOD_LIB}")
elseif(EXISTS ${GPIOD_DIR}/lib/libgpiod.so)
    set(GPIOD_LIB ${GPIOD_DIR}/lib/libgpiod.so)
    if(CMAKE_CROSSCOMPILING)
        message(STATUS "gpiod (cross): bundled ${GPIOD_LIB}")
    else()
        message(STATUS "gpiod (native): bundled ${GPIOD_LIB}")
    endif()
else()
    message(WARNING "No system or bundled libgpiod — linking will fail. Install libgpiod-dev.")
    set(GPIOD_LIB gpiod)
endif()
find_library(PTHREAD_LIB pthread REQUIRED)
find_library(M_LIB m REQUIRED)

# 头文件路径
set(INC_DIRS
    ${GPIOD_DIR}/include
    ${DRIVER_DIR}/icm45608
    ${DRIVER_DIR}
    ${HAL_DIR}
    ${SRC_DIR}
    ${TOOLS_DIR}
    ${TOOLS_DIR}/Invn/EmbUtils
    ${INC_DIR}
)

# 源文件

# IMU 驱动
set(IMU_SRCS
    ${IMU_DIR}/inv_imu_driver.c
    ${IMU_DIR}/inv_imu_driver_advanced.c
    ${IMU_DIR}/inv_imu_edmp.c
    ${IMU_DIR}/inv_imu_edmp_wearable.c
    ${IMU_DIR}/inv_imu_edmp_mrm.c
    ${IMU_DIR}/inv_imu_transport.c
    ${IMU_DIR}/inv_imu_i2cm.c
    ${IMU_DIR}/inv_imu_selftest.c
    ${IMU_DIR}/sif_classifier_ir.c
    ${IMU_DIR}/sif_feature_extract_ir.c
)

# 磁力计驱动
set(MAG_SRCS
    ${ICT1531_DIR}/Ict1531x.c
)

# Mag 适配层
set(MAG_ADAPTER_SRCS
    ${SRC_DIR}/invn_mag_ict1531x.c
)

# 工具
set(TOOLS_SRCS
    ${TOOLS_DIR}/Invn/EmbUtils/Message.c
)

# HAL
set(HAL_SRCS
    ${HAL_DIR}/emd_hal.c
)

# 库核心
set(LIB_CORE_SRCS
    ${SRC_DIR}/emd_gaf.c
)

# 目标 1: libimu_hal.so — 动态库
add_library(imu_hal SHARED
    ${IMU_SRCS}
    ${MAG_SRCS}
    ${MAG_ADAPTER_SRCS}
    ${TOOLS_SRCS}
    ${HAL_SRCS}
    ${LIB_CORE_SRCS}
)

target_include_directories(imu_hal
    PUBLIC
        $<BUILD_INTERFACE:${INC_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${INC_DIRS}
)

target_link_libraries(imu_hal
    PUBLIC ${GPIOD_LIB} ${PTHREAD_LIB} ${M_LIB}
)

target_compile_options(imu_hal PRIVATE -Wall -Wextra -O2)

# 安装
install(TARGETS imu_hal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY inc/ DESTINATION include/imu_hal)

# 目标 2: emd-gaf — 可执行程序 (链接 libimu_hal.so)
add_executable(emd-gaf ${DEMO_DIR}/emd-gaf.c)
target_link_libraries(emd-gaf imu_hal)
target_compile_options(emd-gaf PRIVATE -Wall -Wextra -O2)

install(TARGETS emd-gaf DESTINATION bin)

# 目标 3: read_sensor — 示例程序 (链接 libimu_hal.so)
add_executable(read_sensor ${EXAMPLE_DIR}/read_sensor.c)
target_link_libraries(read_sensor imu_hal)
target_compile_options(read_sensor PRIVATE -Wall -Wextra -O2)

install(TARGETS read_sensor DESTINATION bin)
