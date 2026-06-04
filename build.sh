#!/bin/bash
#==============================================================================
# motor_hal 交叉编译脚本
# 用法: ./build.sh [shared|static]
#   shared - 编译动态库(.so) + 工具 (默认)
#   static - 编译静态库(.a) + 工具
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN="$PROJECT_DIR/toolchain.cmake"
CMAKE="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"

BUILD_TYPE="${1:-shared}"

case "$BUILD_TYPE" in
    shared)
        SHARED_FLAG="-DBUILD_AS_SHARED=ON"
        ;;
    static)
        SHARED_FLAG="-DBUILD_AS_SHARED=OFF"
        ;;
    *)
        echo "用法: $0 [shared|static]"
        echo "  shared - 编译动态库(.so) (默认)"
        echo "  static - 编译静态库(.a)"
        exit 1
        ;;
esac

BUILD_DIR="$PROJECT_DIR/build"
TOOLS_BUILD_DIR="$PROJECT_DIR/tools/build"

#==============================================================================
# 清理
#==============================================================================
rm -rf "$BUILD_DIR" "$TOOLS_BUILD_DIR"

#==============================================================================
# 1. 编译 motor_hal 库
#==============================================================================
echo "=========================================="
echo "  [1/2] 编译 motor_hal ($BUILD_TYPE)"
echo "=========================================="

mkdir -p "$BUILD_DIR"
$CMAKE -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DBUILD_EXAMPLES=ON \
    $SHARED_FLAG

$CMAKE --build "$BUILD_DIR" -j"$(nproc)"

#==============================================================================
# 2. 编译 motor_tool 工具
#==============================================================================
echo ""
echo "=========================================="
echo "  [2/2] 编译 motor_tool"
echo "=========================================="

mkdir -p "$TOOLS_BUILD_DIR"
$CMAKE -S "$PROJECT_DIR/tools" -B "$TOOLS_BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    $SHARED_FLAG

$CMAKE --build "$TOOLS_BUILD_DIR" -j"$(nproc)"

#==============================================================================
# 结果
#==============================================================================
echo ""
echo "=========================================="
echo "  编译完成!"
echo "=========================================="
echo ""

if [ "$BUILD_TYPE" = "shared" ]; then
    LIB_FILE="$BUILD_DIR/libmotor_hal.so"
else
    LIB_FILE="$BUILD_DIR/libmotor_hal.a"
fi

echo "库文件:"
ls -lh "$LIB_FILE"

echo ""
echo "工具:"
ls -lh "$TOOLS_BUILD_DIR/motor_tool"

echo ""
echo "示例:"
ls -lh "$BUILD_DIR"/motor_example_* 2>/dev/null

echo ""
echo "架构信息:"
/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/aarch64-buildroot-linux-gnu-readelf -h "$TOOLS_BUILD_DIR/motor_tool" 2>/dev/null | grep -E "Machine|Class|Type" || true
