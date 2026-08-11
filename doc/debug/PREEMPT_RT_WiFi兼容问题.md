# PREEMPT_RT 下 WS73 WiFi 驱动兼容问题

> **日期**: 2026-08-11  
> **环境**: RV1126B, Linux 6.1.141-rt36, WS73 (Lierda UB37/DB37) WiFi 驱动 v1.10.113  
> **现象**: WiFi 加载/运行时 kernel oops; CAN 数据到达延时 max 6962 µs

---

## 1. 问题概述

在 PREEMPT_RT 内核下加载 WS73 WiFi 驱动, 出现两种 kernel oops。同时 WiFi 线程抢占 RT 线程所在 Core 导致 CAN 延时尖峰。

| 症状 | 触发条件 | 严重程度 |
|------|---------|---------|
| `migrate_enable` WARNING → kernel oops | WiFi 连接时 mac_vap 操作 | 🔴 内核警告, 可能 panic |
| `try_to_take_rt_mutex` 野指针 crash | CAN 发送 + WiFi 并发时 | 🔴 内核 crash |
| CAN 到达延时 max 6962 µs | WiFi 固件消息处理时 | 🟡 实时性破坏 |

---

## 2. 根因分析

### 2.1 Oops #1: `migrate_enable` WARNING

**Call trace:**
```
migrate_enable → rt_spin_unlock → __local_bh_enable
  → mac_vap_intrrupt_enable [wifi_soc]
```

**根因**: 驱动 `hmac_vap.c` 中的 `mac_vap_intrrupt_disable/enable` 函数对:

```c
// hmac_vap.c — PREEMPT_RT 不兼容的锁模式
void mac_vap_intrrupt_disable(void) {
    preempt_disable();     // 非RT: preempt_count++;  RT: preempt_count++
    local_bh_disable();    // 非RT: preempt_count+=SOFTIRQ;  RT: migrate_disable() ← 不同机制!
}
void mac_vap_intrrupt_enable(void) {
    local_bh_enable();     // RT: migrate_enable() → 可能触发 WARNING
    preempt_enable();      // preempt_count--
}
```

**原理**:
- 非 RT 内核: `preempt_disable` 和 `local_bh_disable` 都操作 `preempt_count`, 嵌套关系正确
- PREEMPT_RT: `local_bh_disable` 变成 `migrate_disable`, 与 `preempt_disable` 是完全独立的两套计数器
- `migrate_enable` 做迁移检查时发现计数不一致 → WARNING

### 2.2 Oops #2: `try_to_take_rt_mutex` 野指针 (CAN 发送路径)

**Call trace:**
```
write() → can_send() → __dev_queue_xmit() → __local_bh_disable_ip()
  → rt_spin_lock() → try_to_take_rt_mutex() → 💥 野指针
```

**根因**: 多个上下文（RT FIFO 90 + SCHED_OTHER + SCHED_FIFO 85）同时 `write()` 同一个 CAN socket fd:
- RT Worker (SCHED_FIFO 90) — 每 1ms 发 PDO 帧
- 主线程 (SCHED_OTHER) — SDO 配置写
- CAN Recv (SCHED_FIFO 85) — heartbeat 响应

在 PREEMPT_RT 下, CAN 设备驱动的 `spin_lock` 变成 `rt_mutex`, 不同调度上下文的并发访问可能导致锁状态损坏。

> **注意**: 这个问题在 WiFi 线程移出 Core 3 后更易触发, 因为 RT Worker 不再被 WiFi 抢占, CAN 发送密度增大。

### 2.3 CAN 延时尖峰: WiFi 抢占 Core 3

**证据**:
```bash
# WiFi 两个核心线程都硬编码绑在 Core 3
ps -eLo pid,tid,psr,cls,pri,comm | grep wifi
 467 467 2 FF 90 wifi_frw_msg      # 固件消息处理
 468 468 0 FF 90 wifi_frw_txdata   # 发送数据处理
```

frw_thread.c 中两处硬编码:
```c
osal_kthread_set_affinity(th->thread, OSAL_CPU_3);   // frw_task_process()
osal_kthread_set_affinity(thread, OSAL_CPU_3);        // frw_thread_create()
```

Core 3 同时运行:
```
RT Worker  │ SCHED_FIFO 90  ← 1KHz 控制循环
CAN Recv   │ SCHED_FIFO 85  ← CAN 帧接收
Sync Timer │ SCHED_FIFO 80  ← 同步定时器
IMU HAL    │ SCHED_FIFO 50  ← IMU 传感器
wifi_frw   │ SCHED_FIFO 50  ← WiFi 固件消息 (几ms 处理时间)  ← 尖峰来源!
```

WiFi 固件消息处理 (认证/扫描/连接) 一次可达 2-5ms, SCHED_FIFO 50 虽低于 motor control 线程, 但在这些线程休眠/阻塞间隙仍可长时间占用 Core 3, 导致 CAN Recv 线程被延迟 — 最坏 6962µs。

**验证方法**:
```bash
# CAN 延时
perf_test 或 SHM 读取 can_delay_max_us 字段

# Core 3 线程分布
ps -eLo pid,tid,psr,cls,pri,comm | awk '$3==3' | sort -k4,4r
```

---

## 3. 修复方案

### Patch 文件

位置: `tools/ws73_preempt_rt_fix.patch` (修改驱动源码)

### 3.1 修改 1: hmac_vap.c — 修复 Oops #1

```c
// PREEMPT_RT 下跳过 local_bh_disable/enable
// RT 内核中 softirq 已线程化, BH-disable 不提供额外保护
void mac_vap_intrrupt_enable(void) {
#if defined(_PRE_OS_VERSION_LINUX) && (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#ifndef CONFIG_PREEMPT_RT
    local_bh_enable();    // PREEMPT_RT: 跳过
#endif
    preempt_enable();
#endif
}

void mac_vap_intrrupt_disable(void) {
#if defined(_PRE_OS_VERSION_LINUX) && (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#ifndef CONFIG_PREEMPT_RT
    local_bh_disable();   // PREEMPT_RT: 跳过
#endif
    preempt_disable();
#endif
}
```

### 3.2 修改 2: frw_thread.c — 移除 Core 3 绑定

```c
// frw_task_process() — 删除
-    osal_kthread_set_affinity(th->thread, OSAL_CPU_3);

// frw_thread_create() — 删除
-    osal_kthread_set_affinity(thread, OSAL_CPU_3);
```

让 WiFi 线程由内核调度器自由分配, 不再抢占 Core 3。

### 3.3 应用方法

```bash
cd Lierda_UB37_DB37_driver_1.10.113
patch -p1 < /path/to/ws73_preempt_rt_fix.patch
# 重新编译 .ko, 替换目标板上的 wifi_soc.ko
```

---

## 4. 诊断命令

### 4.1 确认 PREEMPT_RT 内核

```bash
uname -a | grep PREEMPT_RT
```

### 4.2 查 WiFi 线程调度和核分布

```bash
ps -eLo pid,tid,psr,cls,pri,comm | grep wifi
# FF=SCHED_FIFO, PSR=所在CPU核
```

### 4.3 查 Core 3 上所有线程

```bash
ps -eLo pid,tid,psr,cls,pri,comm | awk '$3==3' | sort -k4,4r
```

### 4.4 查 CAN 延时 (SHM)

```bash
# can_delay 字段: 0x300 CAN 帧时间戳 → RT Worker 读到的时间差 (µs)
# 正常 <500µs, WiFi 抢占时 max 可达 6962µs
perf_test  # 或通过 Web 接口读取
```

### 4.5 查内核日志 (oops)

```bash
dmesg | grep -A30 "WARNING\|Oops\|Call trace"
```

### 4.6 临时绑 WiFi 线程到 Core 1 (不重启验证)

```bash
for p in $(pgrep -f wifi_frw); do taskset -cp 1 $p; done
```

---

## 5. 相关文件

| 文件 | 说明 |
|------|------|
| `tools/ws73_preempt_rt_fix.patch` | WiFi 驱动 PREEMPT_RT 修复 patch |
| `tools/isolate_core3.sh` | Core 3 隔离脚本 (备用) |
| `doc/debug/RT调试命令速查.md` | RT 线程调试命令 |
| `doc/design/03_实时系统设计.md` | 实时系统线程模型设计 |

---

## 6. 参考

- 驱动源码: `Lierda_UB37_DB37_driver_1.10.113.tar.gz` (v1.10.113)
- 利尔达 UB37/DB37 WS73 WiFi 模组
- Linux PREEMPT_RT 文档: `Documentation/locking/rt-mutex-design.rst`
