#!/bin/bash
#==============================================================================
# deploy.sh — 编译产物打包, 生成 adb push 部署目录
#
# 用法:
#   ./deploy.sh                     # 打包到 ./stark_deploy/
#   ./deploy.sh push                # 打包后直接 adb push 到主板
#   ./deploy.sh clean               # 清理打包目录
#
# 主板路径:
#   /userdata/stark/lib/  — .so 库文件
#   /userdata/stark/bin/  — 可执行文件
#
# Copyright (c) 2026 zhiqiang.yang
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEPLOY_ROOT="$PROJECT_DIR/stark_deploy"
DEPLOY_LIB="$DEPLOY_ROOT/userdata/stark/lib"
DEPLOY_BIN="$DEPLOY_ROOT/userdata/stark/bin"

TARGET_DEVICE="/userdata/stark"

#==============================================================================
# 清理
#==============================================================================
_clean() {
    echo "清理部署目录..."
    rm -rf "$DEPLOY_ROOT"
    echo "  ✓ $DEPLOY_ROOT 已删除"
}

CMD="${1:-pack}"

case "$CMD" in
    clean)
        _clean
        exit 0
        ;;
    pack|push)
        ;;
    *)
        echo "用法: $0 [pack|push|clean]"
        echo "  pack  - 打包到 ./stark_deploy/ (默认)"
        echo "  push  - 打包后直接 adb push"
        echo "  clean - 清理打包目录"
        exit 1
        ;;
esac

#==============================================================================
# 打包
#==============================================================================
_clean
mkdir -p "$DEPLOY_LIB" "$DEPLOY_BIN"

BUILD_DIR="$PROJECT_DIR/build"
TOOLS_DIR="$PROJECT_DIR/tools/build"
EXO_DIR="$PROJECT_DIR/exo_node/build"
TEST_DIR="$PROJECT_DIR/exo_node/test/build"

echo "=========================================="
echo "  stark 部署打包"
echo "=========================================="
echo ""

missing=0
copied=0

# ── 库文件 → /userdata/stark/lib ──
_copy_lib() {
    local src="$1"
    local name="$2"
    if [ -f "$src" ]; then
        cp "$src" "$DEPLOY_LIB/$name"
        ls -lh "$src" | awk '{printf "  ✓ lib/%-30s %5s\n", "'"$name"'", $5}'
        copied=$((copied + 1))
    else
        printf "  ✗ lib/%-30s (未编译)\n" "$name"
        missing=$((missing + 1))
    fi
}

# ── 可执行文件 → /userdata/stark/bin ──
_copy_bin() {
    local src="$1"
    local name="$2"
    if [ -f "$src" ]; then
        cp "$src" "$DEPLOY_BIN/$name"
        ls -lh "$src" | awk '{printf "  ✓ bin/%-30s %5s\n", "'"$name"'", $5}'
        copied=$((copied + 1))
    else
        printf "  ✗ bin/%-30s (未编译)\n" "$name"
        missing=$((missing + 1))
    fi
}

echo "库文件 (→ $TARGET_DEVICE/lib/):"
_copy_lib "$BUILD_DIR/libmotor_hal.so"     "libmotor_hal.so"
_copy_lib "$EXO_DIR/imu_hal/libimu_hal.so" "libimu_hal.so"

echo ""
echo "工具 (→ $TARGET_DEVICE/bin/):"
_copy_bin "$TOOLS_DIR/motor_tool"                   "motor_tool"
_copy_bin "$EXO_DIR/stark_periph_manager_node"      "stark_periph_manager_node"
_copy_bin "$EXO_DIR/imu_hal/emd-gaf"                "emd-gaf"
_copy_bin "$EXO_DIR/imu_hal/read_sensor"            "read_sensor"

echo ""
echo "测试工具 (→ $TARGET_DEVICE/bin/):"
_copy_bin "$TEST_DIR/algo_sim"                      "algo_sim"
_copy_bin "$TEST_DIR/perf_test"                     "perf_test"

echo ""
echo "示例 (→ $TARGET_DEVICE/bin/):"
for f in "$BUILD_DIR"/motor_example_*; do
    if [ -f "$f" ]; then
        name="$(basename "$f")"
        cp "$f" "$DEPLOY_BIN/$name"
        ls -lh "$f" | awk '{printf "  ✓ bin/%-30s %5s\n", "'"$name"'", $5}'
        copied=$((copied + 1))
    fi
done

echo ""
echo "=========================================="
echo "  打包完成: $copied 个文件"
if [ "$missing" -gt 0 ]; then
    echo "  未编译:   $missing 个 (请先执行 ./build.sh)"
fi
echo "=========================================="
echo ""
echo "部署目录:"
echo "  $DEPLOY_LIB/"
echo "  $DEPLOY_BIN/"
echo ""

#==============================================================================
# adb push
#==============================================================================
if [ "$CMD" = "push" ]; then
    echo "=========================================="
    echo "  adb push → $TARGET_DEVICE"
    echo "=========================================="
    echo ""

    if ! command -v adb &>/dev/null; then
        echo "错误: adb 未安装或不在 PATH 中"
        exit 1
    fi

    adb shell mkdir -p "$TARGET_DEVICE/lib" "$TARGET_DEVICE/bin"

    echo "推送库文件..."
    for f in "$DEPLOY_LIB"/*.so; do
        if [ -f "$f" ]; then
            echo "  $(basename "$f")"
            adb push "$f" "$TARGET_DEVICE/lib/"
        fi
    done

    echo ""
    echo "推送可执行文件..."
    for f in "$DEPLOY_BIN"/*; do
        if [ -f "$f" ]; then
            echo "  $(basename "$f")"
            adb push "$f" "$TARGET_DEVICE/bin/"
        fi
    done

    echo ""
    echo "设置可执行权限..."
    adb shell chmod +x $TARGET_DEVICE/bin/* 2>/dev/null || true

    echo ""
    echo "=========================================="
    echo "  部署完成!"
    echo "=========================================="
fi

echo ""
echo "手动 adb push 命令:"
echo "  adb push $DEPLOY_LIB/  $TARGET_DEVICE/lib/"
echo "  adb push $DEPLOY_BIN/  $TARGET_DEVICE/bin/"
