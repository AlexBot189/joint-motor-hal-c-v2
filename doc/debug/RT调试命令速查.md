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
