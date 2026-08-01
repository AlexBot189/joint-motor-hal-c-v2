#!/bin/sh
#==============================================================================
# can_init.sh — 纯内核工具初始化巨蟹电机 (绕过 motor_tool 所有代码)
#
# 用途: 用 cansend + cangen 完整复刻 daemon startup 流程,
#       排除 motor_tool/daemon/motor_hal 代码层面的问题。
#
# 用法:
#   sudo ./can_init.sh <can_iface> <motor_id> [motor_id2]
#   sudo ./can_init.sh can0 1              # 单电机1
#   sudo ./can_init.sh can0 2              # 单电机2
#   sudo ./can_init.sh can0 1 2            # 双电机
#
# 依赖: cansend, cangen (can-utils), ip, sleep, grep
#==============================================================================

set -e

CAN="${1:-can0}"
ID1="${2:-1}"
ID2="${3:-0}"

# ── 颜色输出 ──
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo "${GREEN}[INFO]${NC}  $1"; }
warn()  { echo "${YELLOW}[WARN]${NC}  $1"; }
err()   { echo "${RED}[ERROR]${NC} $1"; }
step()  { echo "${CYAN}[STEP]${NC} $1"; }

# ── CAN ID 计算 ──
SDO_TX1=$(printf "%03X" $((0x600 + ID1)))
if [ "$ID2" -gt 0 ]; then
    SDO_TX2=$(printf "%03X" $((0x600 + ID2)))
fi

trap 'info "Interrupted"; exit 0' INT TERM

#==============================================================================
# 工具函数
#==============================================================================

can_err_count() {
    ip -details -statistics link show "$CAN" 2>/dev/null | \
        grep "berr-counter" | \
        sed 's/.*berr-counter tx \([0-9]*\) rx \([0-9]*\).*/TX=\1 RX=\2/'
}

check_can() {
    ip link show "$CAN" >/dev/null 2>&1 || {
        err "Interface $CAN not found"
        exit 1
    }
    local state
    state=$(ip link show "$CAN" | grep -o "state [A-Z]*" | awk '{print $2}')
    if [ "$state" != "UP" ] && [ "$state" != "UNKNOWN" ]; then
        warn "$CAN is DOWN, bringing up..."
        ip link set "$CAN" up
    fi
}

# SDO 快速构造: sdo_write <node> <index_HEX> <sub> <type> <value_HEX>
# type: 1B=1byte, 2B=2bytes, 4B=4bytes(默认)
sdo_write() {
    local node="$1"        # 1 或 2
    local index="$2"       # 16进制, 如 1017
    local sub="$3"         # 00
    local vtype="${4:-4B}" # 1B/2B/4B
    local value="$5"       # 16进制值

    local tx=$(printf "%03X" $((0x600 + node)))

    case "$vtype" in
        1B)
            # CCS=1 n=3 → 0x2F, 1字节 expedited
            local hex="2F$(echo "$index" | sed 's/\(..\)\(..\)/\2\1/')$(printf "%02X" "$sub")${value}000000"
            ;;
        2B)
            # CCS=1 n=2 → 0x2B, 2字节 expedited
            local val_le="${value:2:2}${value:0:2}"
            local hex="2B$(echo "$index" | sed 's/\(..\)\(..\)/\2\1/')$(printf "%02X" "$sub")${val_le}0000"
            ;;
        *)
            # CCS=1 n=0 → 0x23, 4字节 expedited
            local val_le="${value:6:2}${value:4:2}${value:2:2}${value:0:2}"
            local hex="23$(echo "$index" | sed 's/\(..\)\(..\)/\2\1/')$(printf "%02X" "$sub")${val_le}"
            ;;
    esac

    cansend "$CAN" "${tx}#${hex}"
}

sdo_read() {
    local node="$1"
    local index="$2"
    local sub="$3"

    local tx=$(printf "%03X" $((0x600 + node)))
    local hex="40$(echo "$index" | sed 's/\(..\)\(..\)/\2\1/')$(printf "%02X" "$sub")00000000"

    cansend "$CAN" "${tx}#${hex}"
}

# NMT: nmt <cmd_HEX> <node>
nmt_send() {
    local cmd="$1"
    local node="$2"
    cansend "$CAN" "000#${cmd}$(printf "%02X" "$node")"
}

# DS402: ds402 <node> <cw_HEX>
ds402() {
    sdo_write "$1" 6040 00 2B "${2}0000"
}

#==============================================================================
# 启动单个电机
#==============================================================================

startup_motor() {
    local id="$1"
    step "──────────────────────────────────────"
    step "Motor $id — NMT Start → Operational"
    step "──────────────────────────────────────"

    nmt_send 01 "$id"
    sleep 0.02

    step "Motor $id — SDO: 心跳周期 0x1017 = 2000ms (0x07D0)"
    sdo_write "$id" 1017 00 2B "07D0"
    sleep 0.03

    step "Motor $id — SDO: 关看门狗 0x2650 = 1 (4字节)"
    sdo_write "$id" 2650 00 4B "00000001"
    sleep 0.03

    step "Motor $id — SDO: 读固件版本 0x100A"
    sdo_read "$id" 100A 00
    sleep 0.03

    step "Motor $id — Profile Velocity 0x6081 = 20 (0x00000014)"
    sdo_write "$id" 6081 00 4B "00000014"
    sleep 0.03

    step "Motor $id — Profile Accel 0x6083 = 5000 (0x00001388)"
    sdo_write "$id" 6083 00 4B "00001388"
    sleep 0.03

    step "Motor $id — Profile Decel 0x6084 = 5000 (0x00001388)"
    sdo_write "$id" 6084 00 4B "00001388"
    sleep 0.03

    step "Motor $id — DS402 使能: CW=0x06 (Shutdown)"
    ds402 "$id" "06"
    sleep 0.03

    step "Motor $id — DS402: CW=0x07 (Switch On)"
    ds402 "$id" "07"
    sleep 0.03

    step "Motor $id — DS402: CW=0x0F (Enable Operation)"
    ds402 "$id" "0F"
    sleep 0.15

    step "Motor $id — 传感器透传 0x5503 sub4 = 0x00030004 (1KHz, CANFD_BRS)"
    sdo_write "$id" 5503 04 4B "00030004"
    sleep 0.03

    info "Motor $id — 启动完成 (OPERATION_ENABLED)"
}

poweroff_motor() {
    local id="$1"
    step "Motor $id — 脱使能: CW=0x06 (Shutdown)"
    ds402 "$id" "06"
    info "Motor $id — 已脱使能"
}

#==============================================================================
# 主流程
#==============================================================================

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  CAN 纯内核电机初始化脚本                    ║"
echo "║  绕过 motor_tool/daemon/motor_hal 所有代码   ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

check_can

echo "════════════════════════════════════════"
echo "  CAN: $CAN"
echo "  Motor 1: ID=$ID1 (SDO TX=0x${SDO_TX1})"
if [ "$ID2" -gt 0 ]; then
    echo "  Motor 2: ID=$ID2 (SDO TX=0x${SDO_TX2})"
fi
echo "════════════════════════════════════════"
echo ""

# ── 记录初始 CAN error ──
echo "────────── CAN Error Counters (BEFORE) ──────────"
err_before=$(can_err_count)
echo "  $err_before"
echo ""

# ── Step 1: 启动电机 1 ──
startup_motor "$ID1"

# ── Step 2: 启动电机 2 (如果有) ──
if [ "$ID2" -gt 0 ]; then
    echo ""
    startup_motor "$ID2"
fi

# ── Step 3: 启动 SYNC (cangen) ──
echo ""
step "──────────────────────────────────────"
step "启动 SYNC: cangen $CAN -g 20 -I 0x080 -D 00"
step "──────────────────────────────────────"
info "SYNC PID: cangen 20ms/50Hz, ID=0x080 DLC=0"
echo ""

cangen "$CAN" -g 20 -I 0x080 -D 00 &
SYNC_PID=$!

sleep 1

# ── 记录启动后 CAN error ──
echo ""
echo "────────── CAN Error Counters (AFTER 1s SYNC) ──────────"
err_after=$(can_err_count)
echo "  $err_after"
echo ""

# ── 持续监测 error 增长 ──
echo "══════════════════════════════════════════════════════"
echo "  电机已使能, SYNC 运行中 (cangen PID=$SYNC_PID)"
echo "  在另一终端运行: candump $CAN"
echo ""
echo "  应能看到:"
echo "    0x080          SYNC 帧 (20ms)"
if [ "$ID2" -gt 0 ]; then
    echo "    0x$(printf '%03X' $((0x300 + ID1)))          电机${ID1} 反馈帧"
    echo "    0x$(printf '%03X' $((0x300 + ID2)))          电机${ID2} 反馈帧"
    echo "    0x$(printf '%03X' $((0x700 + ID1)))          电机${ID1} 心跳"
    echo "    0x$(printf '%03X' $((0x700 + ID2)))          电机${ID2} 心跳"
else
    echo "    0x$(printf '%03X' $((0x300 + ID1)))          电机${ID1} 反馈帧"
    echo "    0x$(printf '%03X' $((0x700 + ID1)))          电机${ID1} 心跳"
fi
echo "    0x$(printf '%03X' $((0x680 + ID1)))         电机${ID1} 传感器透传"
echo ""
echo "  按 Ctrl+C 停止 SYNC 并脱使能电机"
echo "══════════════════════════════════════════════════════"
echo ""

# ── 每5秒打印 error 计数器 ──
while true; do
    sleep 5
    now=$(can_err_count)
    echo "[$(date '+%H:%M:%S')] $now"
done

# ── 清理 (不会执行到, trap 处理 Ctrl+C) ──
cleanup() {
    echo ""
    info "════════════════════════════════════════════════"
    info "  清理: 停止 SYNC, 脱使能电机"
    info "════════════════════════════════════════════════"

    # 停止 cangen
    if [ -n "$SYNC_PID" ] && kill -0 "$SYNC_PID" 2>/dev/null; then
        kill "$SYNC_PID" 2>/dev/null
        wait "$SYNC_PID" 2>/dev/null || true
        info "SYNC stopped"
    fi

    # 脱使能
    poweroff_motor "$ID1"
    if [ "$ID2" -gt 0 ]; then
        poweroff_motor "$ID2"
    fi

    # 最终 error 计数
    echo ""
    echo "────────── CAN Error Counters (AFTER CLEANUP) ──────────"
    can_err_count
    echo ""

    info "Done."
    exit 0
}

trap cleanup INT TERM
wait
