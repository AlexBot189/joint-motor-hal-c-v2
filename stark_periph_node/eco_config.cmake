##########################
#
#  	eco_config.cmake
#	version: 0.1
#  	description: stark_periph_manager_node 本地编译配置
#
#  	编译目标: 可执行文件 (bin)
#  	源码自动扫描: src/ 下所有 .cpp/.c 文件
#  	忽略目录: test/ros/web/3rd_party/log_helper/doc/config (不参与主目标)
#  	外部依赖: motor_hal, imu_hal, log_helper (预编译静态/动态库)
#  	测试 demo: add_custom_build 单独编译 demo_algo
#
##########################

# 平台标识
set(PROJECT_TYPE_NAME "rk3576")

add_definitions(-DMODULE_NAME="periph_node")

# 调试符号
set(NO_STRICT TRUE)
set(NEED_SYMBOLS TRUE)

# 内存检测 (仅 x86)
set(MEMORY_CHECK_TYPE "no")

# 编译目标类型
set(COMPILE_TARGET_TYPE "bin")

# 依赖的固件库 (motor_hal/imu_hal/log_helper 通过 link_directories + target_link_libraries 链接)
set(DEPENDENCY_HARDWARE_LIST "")

# 依赖的第三方库
set(DEPENDENCY_THIRD_PARTY_LIST "")

# 依赖的系统库 (pthread/rt/gpiod 通过 target_link_libraries 链接)
set(DEPENDENCY_SYSTEM_LIST "")

# ROS 依赖 (条件启用)
if(ENABLE_ROS)
	set(DEPENDENCY_ROS_LIST "roscpp;std_msgs")
else()
	set(DEPENDENCY_ROS_LIST "")
endif()

# 公开头文件
set(PUBLIC_HEADER_FOLDER "")

# 编译时忽略的源码目录 (不参与主目标自动扫描)
set(IGNORE_SOURCES_FOLDER
	"src/test"
	"src/3rd_party"
	"src/log_helper"
	"src/doc"
	"src/config"
)
if(NOT ENABLE_ROS)
	list(APPEND IGNORE_SOURCES_FOLDER "src/ros")
endif()
if(NOT ENABLE_WEBSERVER)
	list(APPEND IGNORE_SOURCES_FOLDER "src/web")
endif()

# 忽略的源文件
set(IGNORE_SOURCES_FILES "")

# 日志等级
set(ECO_CMAKE_LOG_LEVEL 1)

# 平台编译配置
set(LOCAL_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(${BUILD_PLATFORM} STREQUAL "rk3576")
	set(CUSTOM_LIBRARY_PATH "")
	set(CUSTOM_INLCUDE_PATH "")
	# ROS 消息路径
	set(EROSMSG_INCLUDE_PATH "${ECO_WORKSPACE_DIR}/eros/release/include")
elseif(${BUILD_PLATFORM} STREQUAL "x86")
	set(CUSTOM_LIBRARY_PATH "")
	set(CUSTOM_INLCUDE_PATH "/opt/ros/melodic/include")
endif()

# 编译类型
if(${BUILD_TYPE} STREQUAL "debug")
	# debug 配置
else()
	# release 配置
endif()
