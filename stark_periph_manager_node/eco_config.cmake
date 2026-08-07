##########################
#
#  	eco_config.cmake
#  	description: stark_periph_manager_node 统一编译配置
#
#  	编译目标:
#  	  - stark_periph_manager_node  (bin, 主程序, 自动扫描 src/)
#  	  - libmotor_hal.so            (shared, add_custom_build)
#  	  - libimu_hal.so              (shared, add_custom_build)
#  	  - demo_algo                  (bin, add_custom_build)
#
##########################

set(PROJECT_TYPE_NAME "rk3576")

add_definitions(-DMODULE_NAME="periph_node")

set(NO_STRICT TRUE)
set(NEED_SYMBOLS TRUE)
set(MEMORY_CHECK_TYPE "no")

# 主目标类型
set(COMPILE_TARGET_TYPE "bin")

# 依赖 (motor_hal/immu_hal 由 add_custom_build 构建, 此处留空)
set(DEPENDENCY_HARDWARE_LIST "")
set(DEPENDENCY_THIRD_PARTY_LIST "")
set(DEPENDENCY_SYSTEM_LIST "")

if(ENABLE_ROS)
	set(DEPENDENCY_ROS_LIST "roscpp;std_msgs")
else()
	set(DEPENDENCY_ROS_LIST "")
endif()

set(PUBLIC_HEADER_FOLDER "")

# 主目标忽略目录 (不参与 eco_build 自动扫描)
# motor_hal/immu_hal 由 add_custom_build 独立编译
set(IGNORE_SOURCES_FOLDER
	"src/test"
	"src/3rd_party"
	"src/log_helper"
	"src/doc"
	"src/config"
	"src/motor_hal"
	"src/imu_hal"
)
if(NOT ENABLE_ROS)
	list(APPEND IGNORE_SOURCES_FOLDER "src/ros")
endif()
if(NOT ENABLE_WEBSERVER)
	list(APPEND IGNORE_SOURCES_FOLDER "src/web")
endif()

set(IGNORE_SOURCES_FILES "")

set(ECO_CMAKE_LOG_LEVEL 1)

set(LOCAL_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

# 平台配置
if(${BUILD_PLATFORM} STREQUAL "rk3576")
	set(CUSTOM_LIBRARY_PATH "")
	set(CUSTOM_INLCUDE_PATH "")
	set(EROSMSG_INCLUDE_PATH "${ECO_WORKSPACE_DIR}/eros/release/include")
elseif(${BUILD_PLATFORM} STREQUAL "x86")
	set(CUSTOM_LIBRARY_PATH "")
	set(CUSTOM_INLCUDE_PATH "/opt/ros/melodic/include")
endif()

if(${BUILD_TYPE} STREQUAL "debug")
else()
endif()
