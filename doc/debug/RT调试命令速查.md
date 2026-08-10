# RT 线程调试命令速查

> stark_periph_manager_node 实时线程状态检查

---

## 查所有线程调度策略

```bash
PID=$(pidof stark_periph_manager_node) && for tid in $(ls /proc/$PID/task/ | sort -n); do echo -n "tid=$tid "; chrt -p $tid; done
```

输出示例：
```
tid=842  SCHED_FIFO  prio=90   ← stark_rt (StarkRtWorker)
tid=722  SCHED_FIFO  prio=85   ← CAN 接收
tid=835  SCHED_FIFO  prio=50   ← IMU HAL
tid=700  SCHED_OTHER prio=0    ← 主线程
...
```

---

## 带线程名的版本

```bash
PID=$(pidof stark_periph_manager_node) && for tid in $(ls /proc/$PID/task/ | sort -n); do name=$(grep "^Name:" /proc/$PID/task/$tid/status | awk '{print $2}'); policy=$(chrt -p $tid 2>/dev/null | grep "policy:" | sed 's/.*: //'); prio=$(chrt -p $tid 2>/dev/null | grep "priority:" | awk '{print $NF}'); printf "tid=%-6s name=%-18s %-12s prio=%s\n" "$tid" "$name" "$policy" "$prio"; done
```

---

## ps 一览

```bash
PID=$(pidof stark_periph_manager_node) && ps -eLo pid,tid,cls,rtprio,comm | grep $PID
# FF=SCHED_FIFO  TS=SCHED_OTHER
```

---

## 看 CPU 亲和性

```bash
for tid in $(ls /proc/$(pidof stark_periph_manager_node)/task/ | sort -n); do name=$(grep "^Name:" /proc/$(pidof stark_periph_manager_node)/task/$tid/status | awk '{print $2}'); cpus=$(grep "^Cpus_allowed_list:" /proc/$(pidof stark_periph_manager_node)/task/$tid/status | awk '{print $2}'); echo "tid=$tid name=$name cpus=$cpus"; done
```

---

## 看内存锁定状态

```bash
cat /proc/$(pidof stark_periph_manager_node)/status | grep Vm
```

---

## RT 预算检查

```bash
cat /proc/sys/kernel/sched_rt_runtime_us   # -1=无限制, 950000=95%
```

---

## 说明书

| 命令 | 作用 |
|------|------|
| `chrt -p <tid>` | 查单个线程调度策略和优先级 |
| `ps -eLo pid,tid,cls,rtprio,comm` | 查所有线程的 class + RT 优先级 |
| `/proc/PID/task/TID/status` | 查线程名(Name)、CPU亲和性(Cpus_allowed_list)、内存(Vm*) |
| `/proc/sys/kernel/sched_rt_runtime_us` | RT 组调度预算，-1 无限制 |

---

## RT 收益验证流程

> 目标：证明 SCHED_FIFO 确实产生了实时收益，而非"配了但没效果"。
>
> 核心原理：RT 的收益不是让代码跑得更快，是让线程**不被其他任务抢占**。
> 验证方法：对比"有负载"场景下 RT/非RT 的**周期超限次数**差异。

### 步骤 1：制造 CPU 负载

```bash
# 在 Core 3 上跑一个死循环，模拟其他任务和 stark_rt 抢 CPU
taskset -c 3 stress-ng --cpu 1 --timeout 120s &
STRESS_PID=$!
```

如果没有 `stress-ng`，用纯 shell：

```bash
# Core 3 上空转 120 秒
taskset -c 3 sh -c 'while true; do :; done' &
STRESS_PID=$!
sleep 120 && kill $STRESS_PID
```

### 步骤 2：RT 模式测试

```bash
# 确认 enable_rt=true 启动 stark_node（正常启动）
# 等系统进入 RUNNING 状态后

# 启动负载
taskset -c 3 stress-ng --cpu 1 --timeout 60s &

# 60 秒后检查周期超限
sleep 60

# 读取 overrun（SHM 或日志）
grep "overrun" /var/log/stark_node.log | tail -5
# 或者通过 Web 调试接口
curl http://device:8080/api/latency 2>/dev/null
```

### 步骤 3：非 RT 模式对照

```bash
# 修改 config.json: "enable_rt": false
# 重启 stark_node
# 重复步骤 2
```

### 步骤 4：对比结论

| 场景 | 预期 overrun | 说明 |
|------|-------------|------|
| RT + 无负载 | ~0 | stark_rt 独占 Core 3 |
| RT + 有负载 | ~0 或极少 | SCHED_FIFO 90 不怕 stress-ng |
| 非RT + 无负载 | ~0 | 系统空，SCHED_OTHER 也够 |
| **非RT + 有负载** | **明显增多** | CFS 调度器让 stress-ng 抢走了 CPU |

> 如果 RT+有负载 和 非RT+有负载 的 overrun 一样少 → RT 没生效。
> 如果 RT+有负载 明显少于 非RT+有负载 → RT 有效。

---

### 进阶：用 trace-cmd 精确测量周期抖动

```bash
# 记录 60 秒的调度事件 (只抓 stark_rt 的 TID)
RT_TID=$(grep -l "Name.*stark_rt" /proc/$(pidof stark_periph_manager_node)/task/*/status | head -1 | xargs dirname | xargs basename)
trace-cmd record -e sched_switch -P $(pidof stark_periph_manager_node) sleep 60

# 看 stark_rt 两次唤醒之间的时间间隔
trace-cmd report | grep "stark_rt" | head -20
```

---

### 间接指标：fb_age

```bash
# fb_age = CAN帧到达 → RT worker读到 的延迟
# RT 模式下稳定在 ~100-200μs
# 非RT 模式下偶尔飙到 3-5ms（被抢占期间）

# 通过 Web 调试接口读取
curl http://device:8080/api/latency 2>/dev/null | grep fb_age
```
