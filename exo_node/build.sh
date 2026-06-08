#!/bin/bash
#==============================================================================
# stark_periph_manager_node 交叉编译脚本
# 工具链: aarch64-buildroot-linux-gnu (Aarch64, RV1126B)
# 用法: ./build.sh [clean]
#
# 依赖:
#   - motor_hal 库必须先编译 (../build/libmotor_hal.a 或 .so)
#   - 交叉工具链: /opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARENT_DIR="$(dirname "$PROJECT_DIR")"
TOOLCHAIN="$PARENT_DIR/toolchain.cmake"
CMAKE="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"

BUILD_DIR="$PROJECT_DIR/build"

#==============================================================================
# 清理
#==============================================================================
_clean() {
    echo "清理构建目录..."
    rm -rf "$BUILD_DIR"
    echo "  ✓ build/ 已删除"
}

CMD="${1:-build}"

case "$CMD" in
    clean)
        _clean
        exit 0
        ;;
    build)
        ;;
    *)
        echo "用法: $0 [build|clean]"
        exit 1
        ;;
esac

#==============================================================================
# 编译 stark_periph_manager_node
#==============================================================================
echo "=========================================="
echo "  stark_periph_manager_node (exo_node)"
echo "=========================================="

mkdir -p "$BUILD_DIR"
$CMAKE -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DENABLE_ROS=OFF \
    -DENABLE_WEBSERVER=OFF

$CMAKE --build "$BUILD_DIR" -j"$(nproc)"

#==============================================================================
# 结果
#==============================================================================
echo ""
echo "=========================================="
echo "  编译完成!"
echo "=========================================="

echo ""
echo "可执行文件:"
ls -lh "$BUILD_DIR/stark_periph_manager_node" 2>/dev/null || true

echo ""
echo "架构信息:"
/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/aarch64-buildroot-linux-gnu-readelf -h "$BUILD_DIR/stark_periph_manager_node" 2>/dev/null | grep -E "Machine|Class|Type" || true
