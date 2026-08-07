##########################
#
#  	CMakeLists.txt
#	version: 0.1
#  	description: stark_periph_manager_node 编译入口
#	build: eco_build 自动扫描 src/ 源码, add_custom_build 单独编译 demo_algo
#
##########################

cmake_minimum_required(VERSION 2.8.12)
project(stark_periph_manager_node)

# 编译选项 (必须在 include(eco_config.cmake) 之前设置)
option(ENABLE_ROS       "启用 ROS 适配器"   OFF)
option(ENABLE_WEBSERVER "启用 Web 调试面板" OFF)

# 交叉编译工具链
option(USE_CROSS_COMPILE "Use Cross-compile" ON)
if(USE_CROSS_COMPILE)
	set(TARGET_ARCHITECTURE "arm64")
else()
	set(TARGET_ARCHITECTURE "amd64")
endif()

# 加载本地配置
include(eco_config.cmake)

# 加载全局编译框架
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/exbot/build-dep/rk3576/build/config)
include(eco_config_global)

message("stark_periph_node 源码目录: ${CMAKE_CURRENT_SOURCE_DIR}")

# 外部预编译库路径
set(MOTOR_HAL_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../motor_hal")
set(IMU_HAL_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/../imu_hal")
set(LOG_HELPER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/log_helper")

# 头文件路径
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}/src/interface
	${CMAKE_CURRENT_SOURCE_DIR}/src/main_loop
	${CMAKE_CURRENT_SOURCE_DIR}/src/motor
	${CMAKE_CURRENT_SOURCE_DIR}/src/shm
	${CMAKE_CURRENT_SOURCE_DIR}/src/imu
	${CMAKE_CURRENT_SOURCE_DIR}/src/foot_pressure
	${CMAKE_CURRENT_SOURCE_DIR}/src/utils
	${CMAKE_CURRENT_SOURCE_DIR}/src/button
	${MOTOR_HAL_DIR}/inc
	${MOTOR_HAL_DIR}/src
	${IMU_HAL_DIR}/inc
	${IMU_HAL_DIR}/src/hal/gpiod/include
	${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party
	${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/usr/include
	${CUSTOM_INLCUDE_PATH}
	${EROSMSG_INCLUDE_PATH}
	${PACKAGE_INCLUDE_PATH}
	${COMMON_INCLUDE_PATH}
)

# 库搜索路径
link_directories(
	${MOTOR_HAL_DIR}/build
	${IMU_HAL_DIR}/build
	${LOG_HELPER_DIR}/build
	${IMU_HAL_DIR}/src/hal/gpiod/lib
	${CUSTOM_LIBRARY_PATH}
	${PACKAGE_LIBRARY_PATH}
	${COMMON_LIBRARY_PATH}
)

# 主目标: 自动扫描 src/ 下所有源码 (忽略目录见 eco_config.cmake)
include(eco_build)

# 系统库
target_link_libraries(${PROJECT_NAME}
	motor_hal
	imu_hal
	log_helper
	gpiod
	pthread
	rt
)

# ROS 适配器
if(ENABLE_ROS)
	find_package(roscpp REQUIRED)
	find_package(std_msgs REQUIRED)
	include_directories(${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
	add_definitions(-DENABLE_ROS)
endif()

# Web 调试面板 (crow 头文件已在 include 路径, 无需额外链接)
if(ENABLE_WEBSERVER)
	add_definitions(-DENABLE_WEBSERVER)
endif()

# 测试 demo: 纯 C, 独立编译, 仅链接系统库
add_custom_build(bin demo_algo
	"${CMAKE_CURRENT_SOURCE_DIR}/src/test/demo_algo.c"
	"rt;pthread;m"
)

include(eco_install)
