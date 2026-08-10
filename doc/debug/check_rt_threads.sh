#!/bin/sh
# check_rt_threads.sh — 查看 stark_periph_manager_node 线程调度状态
# 用法: chmod +x check_rt_threads.sh && ./check_rt_threads.sh

PID=$(pidof stark_periph_manager_node)

if [ -z "$PID" ]; then
    echo "❌ stark_periph_manager_node 未运行"
    exit 1
fi

echo "=========================================="
echo "  stark_periph_manager_node RT 线程检查"
echo "  PID=$PID"
echo "=========================================="
echo ""

# ── 1. 线程调度策略一览 ──
echo "── 1. 线程调度策略 ──"
printf "%-6s  %-20s  %-3s  %-6s  %s\n" "TID" "NAME" "CLS" "RTPRIO" "说明"
printf "%-6s  %-20s  %-3s  %-6s  %s\n" "---" "----" "---" "------" "----"

ps -eLo tid,cls,rtprio,comm | grep "$PID" | grep -v grep | sort -k2 | while read line; do
    tid=$(echo $line | awk '{print $1}')
    cls=$(echo $line | awk '{print $2}')
    prio=$(echo $line | awk '{print $3}')
    name=$(echo $line | awk '{print $4}')

    case "$cls" in
        FF) cls_label="RT ✅";;
        TS) cls_label="NRT  ";;
        *)  cls_label="$cls";;
    esac

    case "$name" in
        stark_rt)          desc="StarkRtWorker (控制+上报 1KHz)";;
        stark_nrt)         desc="StarkRtWorker (非RT模式)";;
        *)
            case "$prio" in
                85) desc="CAN 接收线程";;
                50) desc="IMU HAL 后台采集";;
                *)  desc="";;
            esac
            ;;
    esac

    printf "%-6s  %-20s  %-6s  %-6s  %s\n" "$tid" "$name" "$cls_label" "$prio" "$desc"
done

echo ""

# ── 2. /proc 详细 ──
echo "── 2. /proc 线程详情 ──"
for t in /proc/$PID/task/*/status; do
    tid=$(basename $(dirname $t))
    name=$(grep "^Name:" $t | awk '{print $2}')
    policy_num=$(grep "^Policy:" $t | awk '{print $2}')
    prio=$(grep "^prio:" $t | awk '{print $2}')

    case $policy_num in
        1) policy="SCHED_FIFO";;
        0) policy="SCHED_OTHER";;
        *) policy="unknown($policy_num)";;
    esac

    printf "  tid=%-6s  name=%-20s  %-12s  prio=%s\n" "$tid" "$name" "$policy" "$prio"

    # 对 RT 线程额外显示更多信息
    if [ "$policy" = "SCHED_FIFO" ]; then
        cpus=$(grep "^Cpus_allowed_list:" $t | awk '{print $2}')
        printf "    → CPU亲和性=%s\n" "$cpus"
    fi
done

echo ""

# ── 3. 统计 ──
rt_count=0
nrt_count=0
for t in /proc/$PID/task/*/status; do
    policy=$(grep "^Policy:" $t | awk '{print $2}')
    if [ "$policy" = "1" ]; then
        rt_count=$((rt_count + 1))
    else
        nrt_count=$((nrt_count + 1))
    fi
done

total=$((rt_count + nrt_count))
echo "── 3. 汇总 ──"
echo "  总线程数: $total"
echo "  RT 线程 (SCHED_FIFO): $rt_count"
echo "  非RT线程 (SCHED_OTHER): $nrt_count"
echo ""
echo "  期望 RT 线程数: 3 (stark_rt=CAN=IMU)"
echo "  期望非RT优先级: stark_rt=90, CAN=85, IMU=50"

# ── 4. 关键项判断 ──
echo ""
echo "── 4. 诊断 ──"

issues=0

# 检查 stark_rt 是否存在
if ! grep -q "Name.*stark_rt" /proc/$PID/task/*/status 2>/dev/null; then
    echo "  ⚠️  stark_rt 线程未找到 (enable_rt 可能为 false 或创建失败)"
    issues=$((issues + 1))
else
    rt_policy=$(grep -l "Name.*stark_rt" /proc/$PID/task/*/status | xargs grep "^Policy:" | awk '{print $2}')
    if [ "$rt_policy" != "1" ]; then
        echo "  ❌ stark_rt 不是 SCHED_FIFO (当前 policy=$rt_policy), 可能是 root 权限不足"
        issues=$((issues + 1))
    fi
fi

# 检查 RT 线程优先级是否合理
for t in /proc/$PID/task/*/status; do
    name=$(grep "^Name:" $t | awk '{print $2}')
    policy=$(grep "^Policy:" $t | awk '{print $2}')
    prio=$(grep "^prio:" $t | awk '{print $2}')

    [ "$policy" != "1" ] && continue

    case "$name" in
        stark_rt)
            [ "$prio" != "90" ] && echo "  ⚠️  stark_rt 优先级=$prio (期望 90)" && issues=$((issues + 1))
            ;;
    esac
done

# 检查 RT 预算
rt_runtime=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null)
if [ "$rt_runtime" = "-1" ]; then
    echo "  ✅ RT 预算: 无限制 (sched_rt_runtime_us=-1)"
elif [ -n "$rt_runtime" ] && [ "$rt_runtime" != "950000" ]; then
    echo "  ℹ️  RT 预算: ${rt_runtime}μs (默认 950000=95%)"
fi

# 检查是否 root
if [ "$(id -u)" = "0" ]; then
    echo "  ✅ 当前用户: root"
else
    echo "  ⚠️  当前用户非 root, SCHED_FIFO 可能静默失败"
    issues=$((issues + 1))
fi

if [ $issues -eq 0 ]; then
    echo "  ✅ 所有检查通过"
fi

echo ""
echo "=========================================="
