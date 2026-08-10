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

# ── 1. ps 一览 ──
echo "── 1. 线程调度策略 ──"
printf "%-6s  %-18s  %-14s  %-6s  %s\n" "TID" "NAME" "POLICY" "PRIO" "说明"
printf "%-6s  %-18s  %-14s  %-6s  %s\n" "---" "----" "------" "----" "----"

# 收集所有 TID
TIDS=$(ls /proc/$PID/task/ | sort -n)

for tid in $TIDS; do
    # 线程名
    name=$(grep "^Name:" /proc/$PID/task/$tid/status 2>/dev/null | awk '{print $2}')
    [ -z "$name" ] && name="-"

    # 调度信息 — 用 chrt, 最可靠
    chrt_out=$(chrt -p $tid 2>/dev/null)
    policy=$(echo "$chrt_out" | grep "scheduling policy" | sed 's/.*: //')
    rtprio=$(echo "$chrt_out" | grep "scheduling priority" | awk '{print $NF}')

    # 如果没有 chrt 或权限不够, 回退到 ps
    if [ -z "$policy" ]; then
        policy="-"
        rtprio="-"
    fi

    # 标记 RT / NRT
    case "$policy" in
        SCHED_FIFO|SCHED_RR) tag="✅";;
        SCHED_OTHER)         tag="  ";;
        *)                   tag="";;
    esac

    # 说明
    desc=""
    case "$name" in
        stark_rt)      desc="$tag StarkRtWorker (控制+上报 1KHz)";;
        stark_nrt)     desc="StarkRtWorker (非RT模式)";;
    esac
    [ -z "$desc" ] && [ "$policy" = "SCHED_FIFO" ] && desc="$tag RT线程" && rtprio_n=$rtprio
    [ -z "$desc" ] && desc=""

    printf "%-6s  %-18s  %-14s  %-6s  %s\n" "$tid" "$name" "$policy" "$rtprio" "$desc"
done

echo ""

# ── 2. RT 线程详情 ──
echo "── 2. RT 线程详情 (CPU亲和性) ──"
for tid in $TIDS; do
    policy=$(chrt -p $tid 2>/dev/null | grep "scheduling policy" | sed 's/.*: //')
    if [ "$policy" = "SCHED_FIFO" ] || [ "$policy" = "SCHED_RR" ]; then
        name=$(grep "^Name:" /proc/$PID/task/$tid/status | awk '{print $2}')
        cpus=$(grep "^Cpus_allowed_list:" /proc/$PID/task/$tid/status | awk '{print $2}')
        rtprio=$(chrt -p $tid 2>/dev/null | grep "scheduling priority" | awk '{print $NF}')
        printf "  tid=%-6s  name=%-18s  policy=%s  prio=%s  cpu=%s\n" \
               "$tid" "$name" "$policy" "$rtprio" "$cpus"
    fi
done

echo ""

# ── 3. 统计 ──
rt_count=0
nrt_count=0
for tid in $TIDS; do
    policy=$(chrt -p $tid 2>/dev/null | grep "scheduling policy" | sed 's/.*: //')
    if [ "$policy" = "SCHED_FIFO" ] || [ "$policy" = "SCHED_RR" ]; then
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

# ── 4. 诊断 ──
echo ""
echo "── 4. 诊断 ──"
issues=0

# 检查 stark_rt
stark_tid=$(grep -l "Name:.*stark_rt" /proc/$PID/task/*/status 2>/dev/null | head -1 | xargs dirname | xargs basename)
if [ -z "$stark_tid" ]; then
    echo "  ⚠️  stark_rt 线程未找到 → enable_rt 可能为 false"
    issues=$((issues + 1))
else
    policy=$(chrt -p $stark_tid 2>/dev/null | grep "scheduling policy" | sed 's/.*: //')
    rtprio=$(chrt -p $stark_tid 2>/dev/null | grep "scheduling priority" | awk '{print $NF}')
    if [ "$policy" != "SCHED_FIFO" ]; then
        echo "  ❌ stark_rt (tid=$stark_tid) = $policy, 不是 SCHED_FIFO"
        issues=$((issues + 1))
    elif [ "$rtprio" != "90" ]; then
        echo "  ⚠️  stark_rt 优先级=$rtprio (期望 90)"
        issues=$((issues + 1))
    fi
fi

# 期望的 RT 线程
expected="stark_rt 90|CAN_RECV 85|IMU_HAL 50"
echo "  期望 RT 线程: stark_rt(90), CAN(85), IMU(50)"
echo "  实际 RT 线程: $rt_count 个"

# RT 预算
rt_runtime=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null)
if [ "$rt_runtime" = "-1" ]; then
    echo "  ✅ RT 预算: 无限制"
else
    echo "  ℹ️  RT 预算: ${rt_runtime}μs (95% 后限流)"
fi

# root
[ "$(id -u)" = "0" ] && echo "  ✅ root" || echo "  ⚠️  非root"

if [ $issues -eq 0 ] && [ $rt_count -ge 3 ]; then
    echo "  ✅ 所有检查通过"
fi

echo ""
echo "=========================================="
