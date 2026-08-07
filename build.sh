#!/bin/bash
#==============================================================================
# stark_periph_manager_node 统一编译脚本
# 用法: ./build.sh [debug|release]
#
# 编译 motor_hal + imu_hal + stark_periph_manager_node + demo_algo
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
MANAGER_DIR="$PROJECT_DIR/stark_periph_manager_node"

cd "$MANAGER_DIR"
bash make.sh "$@"
