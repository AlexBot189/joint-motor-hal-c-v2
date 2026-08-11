#!/bin/sh
#==============================================================================
# isolate_core3.sh — Core 3 RT 隔离脚本
# 
# 用途: 将非 RT 线程移出 Core 3, 确保 RT Worker/CAN Recv/Sync/IMU 独占
# 部署: 加到 /etc/init.d/ 或 stark_periph_manager_node 启动前调用
#
# 内核参数方式 (推荐永久方案):
#   在 U-Boot 中修改 bootargs:
#     setenv bootargs "... isolcpus=3 rcu_nocbs=3 nohz_full=3 irqaffinity=0-2"
#     saveenv
#==============================================================================

TARGET_CPU=3
ALLOWED_CPUS="0-2"   # 非RT任务只允许在 Core 0-2

echo "[isolate_core3] === Core ${TARGET_CPU} RT 隔离 ==="

#──────────────────────────────────────────────────────────
# 1. 移走所有非 RT 线程 (SCHED_OTHER / SCHED_BATCH)
#──────────────────────────────────────────────────────────
echo "[isolate_core3] 扫描 Core ${TARGET_CPU} 上的非RT线程..."

for pid in $(ps -eo pid,psr,cls --no-headers 2>/dev/null | awk -v cpu="$TARGET_CPU" '$2==cpu && $3!="FF" {print $1}'); do
    if [ -d "/proc/$pid" ]; then
        comm=$(cat /proc/$pid/comm 2>/dev/null)
        taskset -pc "$ALLOWED_CPUS" "$pid" 2>/dev/null && \
            echo "  ✓ 移走: $pid ($comm)"
    fi
done

#──────────────────────────────────────────────────────────
# 2. 重点目标: WiFi 固件消息线程
#──────────────────────────────────────────────────────────
for pattern in "wifi_frw_msg" "plat_soc" "wifi_soc" "cfg80211" "wpa_supplicant" "hostapd"; do
    for pid in $(pgrep -f "$pattern" 2>/dev/null); do
        taskset -pc "$ALLOWED_CPUS" "$pid" 2>/dev/null && \
            echo "  ✓ WiFi: $pid ($pattern) → Core ${ALLOWED_CPUS}"
    done
done

#──────────────────────────────────────────────────────────
# 3. 移走内核线程 (名称含 usb/mmc/npu/rga/vop 等)
#──────────────────────────────────────────────────────────
for pid in $(ps -eo pid,psr,comm --no-headers 2>/dev/null | awk -v cpu="$TARGET_CPU" '$2==cpu {print $1,$3}'); do
    set -- $pid
    pid=$1; comm=$2
    case "$comm" in
        irq/*|mmc*|usb*|npu*|rga*|vop*|spi*|i2c*|kworker*|rcu*)
            taskset -pc "$ALLOWED_CPUS" "$pid" 2>/dev/null && \
                echo "  ✓ kthread: $pid ($comm) → Core ${ALLOWED_CPUS}"
            ;;
    esac
done

#──────────────────────────────────────────────────────────
# 4. 验证: Core 3 上只应留 RT 线程
#──────────────────────────────────────────────────────────
echo ""
echo "[isolate_core3] Core ${TARGET_CPU} 上剩余线程:"
ps -eo pid,psr,cls,comm --no-headers 2>/dev/null | awk -v cpu="$TARGET_CPU" '$2==cpu {printf "  %-6s %s  %s\n", $1, $3, $4}'

echo "[isolate_core3] 完成"
