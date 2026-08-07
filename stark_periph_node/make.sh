#!/bin/bash
##########################
#
#  	make.sh
#  	description: stark_periph_manager_node 交叉编译入口
#  	用法: ./make.sh [debug|release]
#  	依赖: motor_hal 和 imu_hal 需提前编译完成
#
##########################

CUR_DIR=$(pwd)
export ECO_PROJECT_NAME=${CUR_DIR##*/}
export ECO_WORKSPACE_DIR=~/workspace/project/k850/embuild
export DETAILED_BUILDING_MESSAGE=true
export ECO_PKG_PROJECT_NAME=${ECO_PROJECT_NAME}

# 清理 build 目录
rm -rf build
mkdir -p build

# 编译 (调用 rk3576 编译框架)
/home/exbot/build-dep/rk3576/build/sub_make.sh $@
