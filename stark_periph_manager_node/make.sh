#!/bin/bash
##########################
#
#  	make.sh
#  	description: stark_periph_manager_node 统一编译入口
#  	用法: ./make.sh [debug|release]
#
#  	编译 motor_hal + imu_hal + stark_periph_manager_node + demo_algo
#  	一次编译, 全部产出
#
##########################

CUR_DIR=$(pwd)
export ECO_PROJECT_NAME=${CUR_DIR##*/}
export ECO_WORKSPACE_DIR=~/workspace/project/k850/embuild
export DETAILED_BUILDING_MESSAGE=true
export ECO_PKG_PROJECT_NAME=${ECO_PROJECT_NAME}

rm -rf build
mkdir -p build

/home/exbot/build-dep/rk3576/build/sub_make.sh $@
