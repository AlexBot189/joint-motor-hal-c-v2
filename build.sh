#!/bin/bash
#==============================================================================
# motor_hal 交叉编译脚本
# 用法: ./build.sh [shared|static|clean]
#   shared - 编译动态库(.so) + 工具 (默认, 先清理再编译)
#   static - 编译静态库(.a) + 工具 (先清理再编译)
#   clean  - 仅清理构建目录, 不编译
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN="$PROJECT_DIR/toolchain.cmake"
CMAKE="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"

BUILD_DIR="$PROJECT_DIR/build"
TOOLS_BUILD_DIR="$PROJECT_DIR/tools/build"
EXO_BUILD_DIR="$PROJECT_DIR/exo_node/build"
TEST_BUILD_DIR="$PROJECT_DIR/exo_node/test/build"

#==============================================================================
# 清理函数
#==============================================================================
_clean() {
    echo "清理构建目录..."
    rm -rf "$BUILD_DIR" "$TOOLS_BUILD_DIR" "$EXO_BUILD_DIR" "$TEST_BUILD_DIR"
    echo "  ✓ build/       已删除"
    echo "  ✓ tools/build/ 已删除"
    echo "  ✓ exo_node/build/ 已删除"
    echo "  ✓ exo_node/test/build/ 已删除"
}

CMD="${1:-shared}"

case "$CMD" in
    clean)
        _clean
        exit 0
        ;;
    shared)
        SHARED_FLAG="-DBUILD_AS_SHARED=ON"
        ;;
    static)
        SHARED_FLAG="-DBUILD_AS_SHARED=OFF"
        ;;
    *)
        echo "用法: $0 [shared|static|clean]"
        echo "  shared - 编译动态库(.so) (默认, 清理后编译)"
        echo "  static - 编译静态库(.a) (清理后编译)"
        echo "  clean  - 仅清理构建目录"
        exit 1
        ;;
esac

#==============================================================================
# 编译前清理
#==============================================================================
_clean

#==============================================================================
# [1/4] 编译 motor_hal 库
#==============================================================================
echo "=========================================="
echo "  [1/4] 编译 motor_hal ($CMD)"
echo "=========================================="

mkdir -p "$BUILD_DIR"
$CMAKE -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DBUILD_EXAMPLES=ON \
    $SHARED_FLAG

$CMAKE --build "$BUILD_DIR" -j"$(nproc)"

#==============================================================================
# [2/4] 编译 motor_tool 工具
#==============================================================================
echo ""
echo "=========================================="
echo "  [2/4] 编译 motor_tool"
echo "=========================================="

mkdir -p "$TOOLS_BUILD_DIR"
$CMAKE -S "$PROJECT_DIR/tools" -B "$TOOLS_BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    $SHARED_FLAG

$CMAKE --build "$TOOLS_BUILD_DIR" -j"$(nproc)"

#==============================================================================
# [3/4] 编译 stark_periph_manager_node (exo_node)
#==============================================================================
echo ""
echo "=========================================="
echo "  [3/4] 编译 stark_periph_manager_node"
echo "=========================================="

mkdir -p "$EXO_BUILD_DIR"
$CMAKE -S "$PROJECT_DIR/exo_node" -B "$EXO_BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DENABLE_ROS=OFF \
    -DENABLE_WEBSERVER=OFF \
    $SHARED_FLAG

$CMAKE --build "$EXO_BUILD_DIR" -j"$(nproc)"

#==============================================================================
# [4/4] 编译测试工具 (algo_sim + perf_test)
#==============================================================================
echo ""
echo "=========================================="
echo "  [4/4] 编译测试工具"
echo "=========================================="

mkdir -p "$TEST_BUILD_DIR"
$CMAKE -S "$PROJECT_DIR/exo_node/test" -B "$TEST_BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"

$CMAKE --build "$TEST_BUILD_DIR" -j"$(nproc)"

#==============================================================================
# 结果
#==============================================================================
echo ""
echo "=========================================="
echo "  编译完成!"
echo "=========================================="
echo ""

if [ "$CMD" = "shared" ]; then
    LIB_FILE="$BUILD_DIR/libmotor_hal.so"
else
    LIB_FILE="$BUILD_DIR/libmotor_hal.a"
fi

echo "库文件:"
ls -lh "$LIB_FILE"
echo ""
echo "EXO_SHM_HEADER:"
ls -lh "$PROJECT_DIR/exo_node/exo_shm.h"
echo ""
echo "IMU HAL:"
ls -lh "$EXO_BUILD_DIR/imu_hal/libimu_hal.so" 2>/dev/null || echo "  (内置编译, 已链接到 exo_node)"
echo ""
echo "IMU 示例:"
ls -lh "$EXO_BUILD_DIR/imu_hal/emd-gaf" "$EXO_BUILD_DIR/imu_hal/read_sensor" 2>/dev/null || true
echo ""
echo "工具:"
ls -lh "$TOOLS_BUILD_DIR/motor_tool"

echo ""
echo "exo_node:"
ls -lh "$EXO_BUILD_DIR/stark_periph_manager_node" 2>/dev/null || true

echo ""
echo "测试工具:"
ls -lh "$TEST_BUILD_DIR/algo_sim" "$TEST_BUILD_DIR/perf_test" 2>/dev/null || true

echo ""
echo "示例:"
ls -lh "$BUILD_DIR"/motor_example_* 2>/dev/null

echo ""
echo "架构信息:"
/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/aarch64-buildroot-linux-gnu-readelf -h "$TOOLS_BUILD_DIR/motor_tool" 2>/dev/null | grep -E "Machine|Class|Type" || true
