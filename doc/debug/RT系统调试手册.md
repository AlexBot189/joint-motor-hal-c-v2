# RT 系统调试手册

> stark_periph_manager_node PREEMPT_RT 调试、诊断、问题记录

---

## 更新记录

| 版本 | 日期 | 更新内容 |
|------|------|---------|
| V1.0 | 2026-08-01 | 初始版本：RT 线程调试命令、RT 收益验证流程 |
| V1.1 | 2026-08-11 | 新增：WS73 WiFi PREEMPT_RT 兼容问题诊断与修复；CAN 延时分析；线程优先级可配置；全系统核心分配方案 |

---

## 快速诊断

### 一键检查所有线程

```bash
ps -eLo pid,tid,psr,cls,pri,comm | grep -E "stark_periph|wifi_frw|wifi_rx|usb_wifi|irq/.*can|irq/.*btn"
# FF=SCHED_FIFO  TS=SCHED_OTHER   PSR=所在CPU核
```

### 查 stark 进程所有线程

```bash
PID=$(pidof stark_periph_manager_node) && for tid in $(ls /proc/$PID/task/ | sort -n); do echo -n "tid=$tid "; chrt -p $tid; done
```

输出示例：
```
tid=754  SCHED_FIFO  prio=90   ← RT Worker
tid=861  SCHED_FIFO  prio=85   ← CAN 接收
tid=871  SCHED_FIFO  prio=80   ← SYNC 定时器
tid=880  SCHED_FIFO  prio=55   ← IMU HAL
tid=736  SCHED_OTHER prio=0    ← 主线程
```

### 带线程名 + CPU 亲和性

```bash
PID=$(pidof stark_periph_manager_node)
for tid in $(ls /proc/$PID/task/ | sort -n); do
    name=$(grep "^Name:" /proc/$PID/task/$tid/status | awk '{print $2}')
    policy=$(chrt -p $tid 2>/dev/null | grep "policy:" | sed 's/.*: //')
    prio=$(chrt -p $tid 2>/dev/null | grep "priority:" | awk '{print $NF}')
    cpus=$(grep "^Cpus_allowed_list:" /proc/$PID/task/$tid/status | awk '{print $2}')
    printf "tid=%-6s name=%-18s %-12s prio=%-4s cpus=%s\n" "$tid" "$name" "$policy" "$prio" "$cpus"
done
```

---

## 调度和优先级

### 查所有线程 class + RT 优先级

```bash
PID=$(pidof stark_periph_manager_node) && ps -eLo pid,tid,cls,rtprio,comm | grep $PID
# FF=SCHED_FIFO  TS=SCHED_OTHER
```

### 查单个线程

```bash
chrt -p <tid>
```

### RT 预算检查

```bash
cat /proc/sys/kernel/sched_rt_runtime_us   # -1=无限制, 950000=95%
```

### 内存锁定

```bash
cat /proc/$(pidof stark_periph_manager_node)/status | grep Vm
```

---

## 全系统线程/核心分配

### 设计目标

```
RV1126B, 4×Cortex-A35, PREEMPT_RT

Core 0  系统 + 全部 IRQ       Core 1  应用层
  CAN IRQ                       主线程 main_loop   SCHED_OTHER
  GPIO IRQ                      GPIO Monitor       SCHED_OTHER
  WiFi 中断                     语音处理            SCHED_FIFO 45
  WiFi kthread  SCHED_FIFO 50   IoT 通信           SCHED_OTHER
  内核线程                      日志/Web            SCHED_OTHER

Core 2  算法                    Core 3  RT 独占 (isolcpus=3)
  助力控制算法  SCHED_FIFO 85    RT Worker      SCHED_FIFO 90
  步态分析      SCHED_FIFO 60    CAN Recv       SCHED_FIFO 85
  状态分析      SCHED_FIFO 40    SYNC Timer     SCHED_FIFO 80
                                 IMU HAL        SCHED_FIFO 55
```

### 优先级层级 (config.json 可配)

```json
"rt": {
    "control_priority": 90,    // RT Worker
    "recv_priority":    85,    // CAN 接收
    "sync_priority":    80,    // SYNC 定时器
    "imu_priority":     55,    // IMU HAL (高于 WiFi 50)
    "control_period_us": 1000,
    "sync_period_us":   1000,
    "cpu_affinity": [3]
}
```

**运行时不读 config 时的默认值**：control=90, recv=85, sync=80, imu=55, period=1000µs。

### isolcpus 内核参数

```bash
# U-Boot bootargs 追加:
isolcpus=3 rcu_nocbs=3 nohz_full=3 irqaffinity=0-2
```

| 参数 | 作用 |
|------|------|
| `isolcpus=3` | Core 3 禁止普通任务调度 |
| `rcu_nocbs=3` | RCU 回调不在 Core 3 执行 |
| `irqaffinity=0-2` | 所有硬中断只在 Core 0-2 |

### 临时绑 WiFi 线程移出 Core 3

```bash
for p in $(pgrep -f "wifi_frw\|wifi_rx\|usb_wifi"); do taskset -pc 0-2 $p; done
```

---

## RT 收益验证

核心原理：RT 的收益不是让代码跑得更快，是让线程**不被其他任务抢占**。

### 步骤 1：制造 CPU 负载

```bash
# 在 Core 3 上跑一个死循环
taskset -c 3 stress-ng --cpu 1 --timeout 120s &
# 没有 stress-ng 时:
taskset -c 3 sh -c 'while true; do :; done' &
```

### 步骤 2：RT 模式测试

```bash
# config.json: "enable_rt": true
# 启动 stark_node, 进入 RUNNING
taskset -c 3 stress-ng --cpu 1 --timeout 60s &
sleep 60
grep "overrun" /var/log/stark_node.log | tail -5
```

### 步骤 3：非 RT 模式对照

```bash
# config.json: "enable_rt": false, 重启 stark_node, 重复步骤 2
```

### 结论

| 场景 | 预期 overrun | 说明 |
|------|-------------|------|
| RT + 无负载 | ~0 | stark_rt 独占 Core 3 |
| RT + 有负载 | ~0 或极少 | SCHED_FIFO 90 不怕 stress-ng |
| 非RT + 无负载 | ~0 | 系统空 |
| **非RT + 有负载** | **明显增多** | CFS 被抢占 |

---

## CAN 延时分析

### 延时指标含义

```
CAN 数据到达延时 = CAN 帧时间戳 → RT Worker 读到 fb_cache 的时间差
  正常: <500µs
  异常: >1000µs  (WiFi 抢占 Core 3 时 max 可达 6962µs)

反馈处理延时 = RT Worker 读反馈 → 写 SHM 完成
  正常: 2-7µs  (极快, 不构成瓶颈)
```

### 查 CAN 延时

```bash
# SHM 字段 (perf_test 或 Web 接口):
#   can_delay_avg_us  — 平均 CAN 到达延时
#   can_delay_max_us  — 最大 CAN 到达延时
#   fb_proc_avg_us    — 平均反馈处理延时

curl http://device:8080/api/latency 2>/dev/null
```

### CAN 延时尖峰根因

Core 3 上同时运行了 4 个 RT 线程 + WiFi kthread：

```
RT Worker  SCHED_FIFO 90  ← 1KHz PDO 控制
CAN Recv   SCHED_FIFO 85  ← CAN 帧接收+分发
SYNC       SCHED_FIFO 80  ← SYNC 帧
IMU        SCHED_FIFO 55  ← IMU 传感器
WiFi       SCHED_FIFO 50  ← 固件消息处理 (2-5ms) ← 尖峰来源
```

WiFi 固件消息（认证/扫描/连接）一次处理 2-5ms，虽优先级最低，但在 RT 线程休眠间隙仍可长时间占用 Core 3，阻塞 CAN Recv 线程。

**解决方案**：
1. WiFi 驱动 patch 移除 Core 3 绑定
2. `isolcpus=3` 内核参数禁止非 RT 任务上 Core 3

---

## WS73 WiFi PREEMPT_RT 兼容问题

> **驱动版本**: Lierda UB37/DB37 v1.10.113  
> **内核**: Linux 6.1.141-rt36  
> **修复 patch**: `tools/ws73_preempt_rt_fix.patch`

### Oops #1: `migrate_enable` WARNING

**Call trace:**
```
migrate_enable → rt_spin_unlock → __local_bh_enable
  → mac_vap_intrrupt_enable [wifi_soc]
```

**根因**: `hmac_vap.c` 中 `mac_vap_intrrupt_disable/enable` 在 PREEMPT_RT 上锁嵌套不一致：

```c
// 非 RT: preempt_disable + local_bh_disable 都操作 preempt_count → 安全
// PREEMPT_RT: local_bh_disable → migrate_disable (独立的计数器!)
void mac_vap_intrrupt_disable(void) {
    preempt_disable();     // preempt_count++
    local_bh_disable();    // migrate_disable() — 不是 preempt_count!
}
void mac_vap_intrrupt_enable(void) {
    local_bh_enable();     // migrate_enable() → 检查迁移状态 → WARNING
    preempt_enable();      // preempt_count--
}
```

**修复**: 在 `CONFIG_PREEMPT_RT` 下跳过 `local_bh_disable/enable`，RT 内核中 softirq 已线程化，不需要 BH 保护。

### Oops #2: `try_to_take_rt_mutex` 野指针

**Call trace:**
```
write() → can_send() → __dev_queue_xmit() → __local_bh_disable_ip()
  → rt_spin_lock() → try_to_take_rt_mutex() → 野指针
```

**根因**: RT Worker (SCHED_FIFO 90) 和主线程 (SCHED_OTHER) 同时 `write()` 同一个 CAN socket fd。PREEMPT_RT 下 `spin_lock` 变成 `rt_mutex`，并发导致锁状态损坏。

**状态**: WiFi patch 修复后未再出现，暂时不处理。如再现，在 `can_driver_send()` 加 `pthread_mutex` 序列化。

### WiFi 线程硬绑 Core 3

`frw_thread.c` 中 `wifi_frw_msg` 和 `wifi_frw_txdata` 两处硬编码：
```c
osal_kthread_set_affinity(th->thread, OSAL_CPU_3);
```

**修复**: patch 删除这两行，让 WiFi 线程由内核调度器自由分配。

### 诊断命令

```bash
# WiFi 线程调度
ps -eLo pid,tid,psr,cls,pri,comm | grep wifi

# Core 3 线程分布
ps -eLo pid,tid,psr,cls,pri,comm | awk '$3==3' | sort -k4,4r

# 内核日志
dmesg | grep -A30 "WARNING\|Oops\|Call trace"
```

---

## 常用命令速查

| 命令 | 作用 |
|------|------|
| `chrt -p <tid>` | 查单个线程调度策略和优先级 |
| `ps -eLo pid,tid,cls,rtprio,comm` | 查所有线程的 class + RT 优先级 |
| `ps -eLo pid,tid,psr,cls,pri,comm` | 同上 + 所在 CPU 核 |
| `/proc/PID/task/TID/status` | 线程名(Name)、CPU亲和性(Cpus_allowed_list)、内存(Vm*) |
| `/proc/sys/kernel/sched_rt_runtime_us` | RT 组调度预算，-1 无限制 |
| `dmesg \| grep -A30 "WARNING"` | 内核警告/crash 日志 |
| `taskset -cp <cpu_list> <pid>` | 动态修改线程 CPU 亲和性 |
| `cat /proc/interrupts \| grep can` | CAN 中断计数 |
| `cat /sys/kernel/debug/gpio` | GPIO 状态和 IRQ 计数 |

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `doc/design/03_实时系统设计.md` | 实时系统线程模型、延迟预算 |
| `tools/ws73_preempt_rt_fix.patch` | WiFi 驱动 PREEMPT_RT 修复 patch |
| `tools/isolate_core3.sh` | Core 3 隔离脚本 |
| `src/config/stark_config.json` | RT 优先级/周期/CPU 配置 |
