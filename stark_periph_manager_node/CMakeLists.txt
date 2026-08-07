##########################
#
#  	CMakeLists.txt
#  	description: stark_periph_manager_node 编译入口
#
#  	编译产物:
#  	  1. stark_periph_manager_node  (主二进制)
#  	  2. libmotor_hal.so            (电机 CAN/CANopen 驱动)
#  	  3. libimu_hal.so              (IMU 传感器驱动)
#  	  4. demo_algo                  (算法/调试测试工具)
#
##########################

cmake_minimum_required(VERSION 2.8.12)
project(stark_periph_manager_node)

option(ENABLE_ROS       "启用 ROS 适配器"   OFF)
option(ENABLE_WEBSERVER "启用 Web 调试面板" OFF)

option(USE_CROSS_COMPILE "Use Cross-compile" ON)
if(USE_CROSS_COMPILE)
	set(TARGET_ARCHITECTURE "arm64")
else()
	set(TARGET_ARCHITECTURE "amd64")
endif()

include(eco_config.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/exbot/build-dep/rk3576/build/config)
include(eco_config_global)

message("源码根目录: ${CMAKE_CURRENT_SOURCE_DIR}")

set(SRC_ROOT  "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(MH_DIR    "${SRC_ROOT}/motor_hal")
set(IH_DIR    "${SRC_ROOT}/imu_hal")
set(LOG_DIR   "${SRC_ROOT}/log_helper")
set(THIRD_DIR "${SRC_ROOT}/3rd_party")

# ---- 头文件路径 (主目标) ----
include_directories(
	# stark_periph 业务层
	${SRC_ROOT}
	${SRC_ROOT}/interface
	${SRC_ROOT}/main_loop
	${SRC_ROOT}/motor
	${SRC_ROOT}/shm
	${SRC_ROOT}/imu
	${SRC_ROOT}/foot_pressure
	${SRC_ROOT}/utils
	${SRC_ROOT}/button
	# 第三方头文件
	${THIRD_DIR}
	${THIRD_DIR}/usr/include
	# eco_build 框架路径
	${CUSTOM_INLCUDE_PATH}
	${EROSMSG_INCLUDE_PATH}
	${PACKAGE_INCLUDE_PATH}
	${COMMON_INCLUDE_PATH}
)

# ---- 库搜索路径 ----
link_directories(
	${LOG_DIR}/build
	${IH_DIR}/hal/gpiod/lib
	${CUSTOM_LIBRARY_PATH}
	${PACKAGE_LIBRARY_PATH}
	${COMMON_LIBRARY_PATH}
)

# ════════════════════════════════════════════════════════════
# 目标 1: stark_periph_manager_node (主二进制)
#   eco_build 自动扫描 src/ 下所有 .cpp/.c
#   忽略目录见 eco_config.cmake

include(eco_build)

target_link_libraries(${PROJECT_NAME}
	motor_hal
	imu_hal
	log_helper
	gpiod
	pthread
	rt
)

# ROS 适配器 (条件编译)
if(ENABLE_ROS)
	find_package(roscpp REQUIRED)
	find_package(std_msgs REQUIRED)
	include_directories(${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
	add_definitions(-DENABLE_ROS)
endif()

# Web 调试面板 (crow 头文件已在 include 路径)
if(ENABLE_WEBSERVER)
	add_definitions(-DENABLE_WEBSERVER)
endif()

# ════════════════════════════════════════════════════════════
# 目标 2: libmotor_hal.so — 电机 CAN/CANopen 驱动库

# add_custom_build 目标需要独立的 include 路径 (eco_build 内部可能重置)
include_directories(
	${MH_DIR}
	${MH_DIR}/inc
	${IH_DIR}
	${IH_DIR}/inc
	${IH_DIR}/driver
	${IH_DIR}/driver/icm45608
	${IH_DIR}/driver/icm45608/imu
	${IH_DIR}/driver/Ict1531x
	${IH_DIR}/hal
	${IH_DIR}/hal/gpiod/include
	${IH_DIR}/tools
	${IH_DIR}/tools/Invn/EmbUtils
)

add_custom_build(
	NAME motor_hal
	TYPE shared
	SRCS
		"${MH_DIR}/can_driver.c"
		";${MH_DIR}/canopen_frames.c"
		";${MH_DIR}/sdo_client.c"
		";${MH_DIR}/nmt_master.c"
		";${MH_DIR}/heartbeat.c"
		";${MH_DIR}/pdo_handler.c"
		";${MH_DIR}/pdo_mapper.c"
		";${MH_DIR}/feedback_parser.c"
		";${MH_DIR}/motor_hal_startup.c"
		";${MH_DIR}/utils.c"
		";${MH_DIR}/motor_hal.c"
		";${MH_DIR}/motor_calib.c"
	DEPS "pthread;m"
)

# ════════════════════════════════════════════════════════════
# 目标 3: libimu_hal.so — IMU 传感器驱动库

add_custom_build(
	NAME imu_hal
	TYPE shared
	SRCS
		"${IH_DIR}/emd_gaf.c"
		";${IH_DIR}/invn_mag_ict1531x.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_driver.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_driver_advanced.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_edmp.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_edmp_wearable.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_edmp_mrm.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_transport.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_i2cm.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_selftest.c"
		";${IH_DIR}/driver/icm45608/imu/sif_classifier_ir.c"
		";${IH_DIR}/driver/icm45608/imu/sif_feature_extract_ir.c"
		";${IH_DIR}/driver/Ict1531x/Ict1531x.c"
		";${IH_DIR}/hal/emd_hal.c"
		";${IH_DIR}/tools/Invn/EmbUtils/Message.c"
	DEPS "gpiod;pthread;m"
)

# ════════════════════════════════════════════════════════════
# 目标 4: demo_algo — 调试/算法测试工具

add_custom_build(
	NAME demo_algo
	TYPE bin
	SRCS "${SRC_ROOT}/test/demo_algo.c"
	DEPS "rt;pthread;m"
)

include(eco_install)
