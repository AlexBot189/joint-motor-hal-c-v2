#!/bin/bash
# imu_hal 编译脚本
#
# 目录结构:
#   inc/            — 公开头文件 (emd_gaf.h, emd_gaf_types.h)
#   src/            — 库源码 (driver/, hal/, emd_gaf.c)
#   demo/           — 可执行程序 (链接 libimu_hal.so)
#   example/        — 示例程序 (链接 libimu_hal.so)
#
# 产物:
#   build/libimu_hal.so    — 动态库
#   build/emd-gaf          — 可执行程序
#   build/read_sensor      — 示例程序
#
# 用法: ./build.sh [build|clean|native]
#   build  - 交叉编译 (默认, aarch64)
#   clean  - 清理构建目录
#   native - 本机编译 (x86_64, 用于测试)
#
# Author: zhiqiang.yang
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN="$PROJECT_DIR/toolchain.cmake"
CMAKE_CROSS="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"
READELF="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/aarch64-buildroot-linux-gnu-readelf"

BUILD_DIR="$PROJECT_DIR/build"

_clean() {
    echo "清理构建目录..."
    rm -rf "$BUILD_DIR"
    echo " ✓ build/ 已删除"
}

CMD="${1:-build}"

case "$CMD" in
    clean)
        _clean
        exit 0
        ;;
    build|native)
        ;;
    *)
        echo "用法: $0 [build|clean|native]"
        echo "  build  - 交叉编译 (aarch64)"
        echo "  clean  - 清理构建目录"
        echo "  native - 本机编译 (x86_64)"
        exit 1
        ;;
esac

_clean

if [ "$CMD" = "native" ]; then
    echo "=========================================="
    echo " imu_hal 本机编译 (x86_64)"
    echo "=========================================="

    mkdir -p "$BUILD_DIR"
    cmake -S "$PROJECT_DIR" -B "$BUILD_DIR"
    cmake --build "$BUILD_DIR" -j"$(nproc)"

    echo ""
    echo "=========================================="
    echo " 编译完成!"
    echo "=========================================="
    echo ""

    ls -lh "$BUILD_DIR"/libimu_hal* "$BUILD_DIR"/emd-gaf "$BUILD_DIR"/read_sensor 2>/dev/null

    echo ""
    echo "产物:"
    echo "  库:          $BUILD_DIR/libimu_hal.so"
    echo "  可执行程序:  $BUILD_DIR/emd-gaf"
    echo "  示例:        $BUILD_DIR/read_sensor"
    echo ""
    echo "运行:"
    echo "  sudo ./build/emd-gaf -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
    echo "  sudo ./build/read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
else
    echo "=========================================="
    echo " imu_hal 交叉编译"
    echo " Target: aarch64 (RV1126B)"
    echo "=========================================="

    mkdir -p "$BUILD_DIR"
    $CMAKE_CROSS -S "$PROJECT_DIR" -B "$BUILD_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"

    $CMAKE_CROSS --build "$BUILD_DIR" -j"$(nproc)"

    echo ""
    echo "=========================================="
    echo " 编译完成!"
    echo "=========================================="
    echo ""

    ls -lh "$BUILD_DIR"/libimu_hal* "$BUILD_DIR"/emd-gaf "$BUILD_DIR"/read_sensor 2>/dev/null

    echo ""
    echo "产物:"
    for f in "$BUILD_DIR"/libimu_hal.so "$BUILD_DIR"/emd-gaf "$BUILD_DIR"/read_sensor; do
        if [ -f "$f" ]; then
            echo "  $f"
            $READELF -h "$f" 2>/dev/null | grep -E "Machine|Class|Type" || true
        fi
    done

    echo ""
    echo "部署:"
    echo "  scp build/libimu_hal.so  root@rv1126b:/usr/lib/"
    echo "  scp build/emd-gaf        root@rv1126b:/usr/bin/"
    echo "  scp build/read_sensor    root@rv1126b:/usr/bin/"
    echo ""
    echo "运行:"
    echo "  ssh root@rv1126b"
    echo "  rmmod inv-mpu-icm45600"
    echo "  emd-gaf -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
fi
