# exo_node 实时系统架构设计

> 外骨骼机器人 RV1126B CANFD 控制框架  
> 日期: 2026-06-08 | 状态: 设计定稿，待实现

---

## 1. 框架目标

```
┌──────────────────────────────────────────────────────────────────┐
│                        框架目标                                   │
│                                                                  │
│  算法进程: 只关心步态计算，读写 SHM，不感知 CAN/SDO/PDO/使能/模式  │
│  motor_node: 电机全生命周期管理，对上层提供透明控制接口             │
│                                                                  │
│  设计原则:                                                        │
│  1. 使能只在启动阶段做一次，控制路径不再重复                        │
│  2. 安全监控全部内聚在 RT 线程，数据局部性最优                      │
│  3. 一种机制多种用途 (seq_begin = 命令传输 + 握手 + 心跳)          │
│  4. CLI 用软约定，exo_node API 用硬约束                            │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. 进程架构

```
┌──────────────────────────────────────────────────────────────────┐
│                   算法进程 (同事负责, SCHED_FIFO)                   │
│                                                                    │
│  1KHz 循环:                                                        │
│    read  shm→fb_buffer[active]   ← 电机反馈 + IMU + 传感器         │
│    compute 步态 → 力矩                                             │
│    write shm→mailbox.cmd         ← 目标力矩                        │
│    write shm→mailbox.seq_begin++ ← 原子递增                        │
│                                                                    │
│  需要: #include "exo_shm.h"                                       │
│  不需要: motor_hal / CANopen / SDO / PDO / 使能 / 校准             │
└────────────────────────┬───────────────────────────────────────────┘
                         │  /dev/shm/exo_shm  (唯一共享内存)
                         │  算法写: mailbox
                         │  算法读: fb_buffer[active]
═════════════════════════╪═══════════════════════════════════════════
                         │
┌────────────────────────┴───────────────────────────────────────────┐
│                    motor_node (系统侧, 你的工作)                    │
│                                                                    │
│  ┌─ 主线程 (SCHED_OTHER) ───────────────────────────────────────┐ │
│  │  状态机: INIT→DISCOVERY→READY→CALIBRATING→ENABLED→RUNNING    │ │
│  │  ROS Service / GPIO 事件驱动                                  │ │
│  │  process pending startups                                    │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌─ RT 工作线程 (SCHED_FIFO 90, 1KHz) ──────────────────────────┐ │
│  │  ① read mailbox → motor_hal_multi_ctrl() → PDO write         │ │
│  │  ② read fb_cache → 每5周期 write SHM double buffer (200Hz)   │ │
│  │  ③ safety check: seq_begin / 编码器 / 过温 / CAN 断线       │ │
│  │  ④ 首 cmd 检测 → state_transition(RUNNING)                   │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌─ CAN 接收线程 (SCHED_FIFO 85, HAL 内部) ─────────────────────┐ │
│  │  select()→recv()→_dispatch_frame                              │ │
│  │   → SDO 响应 → sdo_push_response (condvar)                    │ │
│  │   → 反馈帧   → fb_cache + fb_cb                              │ │
│  │   → 传感器帧 → sensor_cache                                  │ │
│  │   → Bootup帧 → bootup_received + pending_startup              │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  依赖: libmotor_hal.a + libmotor_calib.a                           │
│  可选: ROS / WebServer (CMake 编译控制, 生产关闭)                   │
└────────────────────────────────────────────────────────────────────┘
```

---

## 3. 分层职责

```
┌─ 算法层 ─────────────────────────────────────────────┐
│  读写 SHM, 步态计算                                    │
│  不感知电机细节                                        │
├─ exo_node 框架层 (motor_node) ────────────────────────┤
│  状态机、安全监控、SHM 管理、启动流程                   │
│  控制命令: 透明切模式 + 写目标 (对算法隐藏使能/模式)    │
├─ motor_hal 层 (libmotor_hal.a) ───────────────────────┤
│  CANFD 驱动、SDO/PDO/NMT/SYNC/反馈/透传               │
│  单次操作, 不做组合逻辑                                │
├─ 硬件层 ──────────────────────────────────────────────┤
│  RV1126B + CANFD + 巨蟹关节电机 + ICM45608 IMU         │
└───────────────────────────────────────────────────────┘
```

### 控制命令对上层透明

```
算法调用:          exo_node 内部:              HAL 操作:
─────────────────────────────────────────────────────────
set_torque(500) → check enabled              motor_hal_set_mode(CURRENT)
                  set_mode(CURRENT) 幂等      motor_hal_sdo_write(0x6071, 500)
                  
set_position(90°)→ check enabled              motor_hal_set_mode(PP)
                  set_mode(PP) 幂等            motor_hal_sdo_write(0x607A)
                                              motor_hal_sdo_write(0x6040, 0x004F)
                  
set_velocity(50) → check enabled              motor_hal_set_mode(PV)
                  set_mode(PV) 幂等            motor_hal_sdo_write(0x60FF)
```

**使能只做一次**（daemon startup 阶段），控制命令不再碰 Shutdown/SwitchOn/EnableOp。

---

## 4. SHM 布局 (精简版)

```c
// /dev/shm/exo_shm  (64KB)

typedef struct {
    /* ===== 双 Buffer 反馈区 (motor_node 写, 算法读) ===== */
    uint32_t          active_idx;          // atomic, 0 或 1
    feedback_frame_t  fb_buffer[2];        // 显式 struct, 含电机+IMU+传感器

    /* ===== Mailbox 命令区 (算法写, motor_node 读) ===== */
    struct {
        uint64_t        seq_begin;         // ★ 一种机制三种用途: 命令/握手/心跳
        motor_command_t cmd;               // 覆盖写, 只保留最新
        uint64_t        seq_end;
    } mailbox;

    /* ===== 状态区 (motor_node 写, 算法读) ===== */
    uint8_t  motor_online;                 // bit0=右 bit1=左 bit2=右膝 bit3=左膝
    uint8_t  calib_state;                  // 0=空闲 1=校准中 2=完成 3=超时
    uint8_t  motor_enabled;                // bit 对应电机使能状态
    uint8_t  motor_severity;               // 0=OK 1=WARN(降额) 2=FAULT(停机)
    uint8_t  fault_reason;                 // 诊断用枚举
    uint8_t  node_state;                   // exo_node 状态机当前状态

    uint8_t  _pad[4013];                   // 对齐 64KB
} exo_shm_t;
```

### 精简对比

| 字段 | v2.0 原设计 | v2.1 |
|------|-----------|------|
| algo_heartbeat | ✅ | ❌ 删除 (seq_begin 代替) |
| cmd_seq | ✅ | ❌ 删除 (seq_begin 代替) |
| algo_ready | ✅ | ❌ 删除 (第一个 cmd 代替) |
| motor_fault | 8 bit | severity(1B) + reason(1B) |
| feedback | ~140B blob | 显式 feedback_frame_t struct |

---

## 5. 状态机

```
┌──────┐   ┌───────────┐   ┌───────┐   ┌─────────────┐   ┌─────────┐   ┌─────────┐
│ INIT │ → │ DISCOVERY │ → │ READY │ → │ CALIBRATING │ → │ ENABLED │ → │ RUNNING │
└──────┘   └───────────┘   └───────┘   └─────────────┘   └─────────┘   └─────────┘
                │               │              │               │             │
                └───────────────┴──────────────┴───────────────┴──────┬──────┘
                                                                      ↓
                                                                  ┌───────┐
                                                                  │ FAULT │
                                                                  └───────┘
                                                                      │
                                                          (人工恢复) → READY
```

| 状态 | 触发 | 做什么 | 超时 | LED |
|------|------|--------|:--:|-----|
| INIT | 进程启动 | open CAN, create SHM, add motors, recv_start | - | 蓝闪 |
| DISCOVERY | INIT 完成 | 等 bootup → motor_startup_full | 5s | 蓝常 |
| READY | 全部在线 | 等校准命令 | - | 蓝常 |
| CALIBRATING | 按键/Service | setzero → poll ±1° | 10s | 黄闪 |
| ENABLED | 校准成功 | 使能电机, torque=0, 等算法 | 10s(无cmd) | 黄常 |
| RUNNING | 第一个 cmd | RT 线程转发, 安全监控激活 | - | 绿常 |
| FAULT | 安全检测 | DS402 Shutdown | - | 红闪 |

### 实现方式

```c
void state_transition(exo_state_t new) {
    exit_hooks[g_state]();    // 清理旧状态
    g_state = new;
    ECO_INFO("State → %s", state_name(new));
    enter_hooks[g_state]();   // 设置新状态
}

// 事件驱动: GPIO 回调/ROS Service 里直接调
void on_calib_button() {
    if (g_state == STATE_READY)
        state_transition(STATE_CALIBRATING);
}
```

**不在独立线程中运行**。主线程同步跑 INIT→DISCOVERY→READY，之后全事件驱动。

---

## 6. 线程模型

| 线程 | 调度 | 优先级 | 周期 | 职责 |
|------|------|:---:|------|------|
| RT 工作线程 | SCHED_FIFO | 90 | 1KHz | 控制 + 上报 + 安全 (三合一) |
| CAN 接收线程 | SCHED_FIFO | 85 | 事件 | 收帧 + 分发 (HAL 内部) |
| 主线程 | SCHED_OTHER | 0 | 事件 | 状态机 + ROS Service |
| 日志线程 | SCHED_OTHER | 0 | 按需 | ring buffer drain → log_helper |

**RT 工作线程单周期时序 (< 50μs)**:
```
read mailbox.seq_begin + cmd       < 1μs
multi_ctrl(cmd) → PDO 64B 广播     ~20μs  (1帧发完所有电机)
read fb_cache (PI mutex)           < 10μs
write SHM double buffer             < 5μs  (每5周期, 200Hz等效)
safety checks                       < 10μs
─────────────────────────────────────────
合计                               < 50μs  (1ms预算的5%)
```

### CPU 隔离建议

```bash
# kernel cmdline
isolcpus=2,3    # core2→RT工作线程, core3→CAN recv
                 # core0/1→主线程 + 算法进程
```

---

## 7. 安全监控 (全部内聚在 RT 工作线程)

每 1KHz 周期检查:

```
  seq_begin 200ms 不变           → torque=0,   severity=WARN
  seq_begin 500ms 不变           → Shutdown,   severity=FAULT
  position 3s 不变 (编码器异常)    → Shutdown,   severity=FAULT
  temperature > 80°C              → torque=0(该电机), severity=WARN
  CAN 2s 无帧                     → Shutdown,   severity=FAULT
  cmd 同一值 500ms (输出停滞)      → torque=0,   severity=WARN
```

**当前阶段不拆安全进程**。数据全在 motor_node 内存中，拆走需要 SHM 桥接。电机自带看门狗（0x1017 + 0x2650）已是硬件级安全网。

量产阶段升级为独立安全 MCU (STM32/GD32) 直连急停 GPIO。

---

## 8. 启动流程 (五步框架)

```
步骤 1 [自动]  motor_tool daemon can0 &
              → tool_init, add_motors(1-4), recv_start
              → 进入 accept 循环, 等待 bootup

步骤 2 [手动]  [物理给电机上电]
              → 电机发送 bootup 0x701 (一次性)

步骤 3 [自动]  recv 线程收到 bootup
              → pending_startup = true
              → 主线程 process_pending_startups()
              → motor_startup_full (心跳+关狗+固件+使能+NMT+TPDO)
              → state: DISCOVERY → READY

步骤 4 [手动]  motor_tool calib start 1 2        # 校准
              motor_tool sensor config 1 250      # 开启透传
              motor_tool sensor config 2 250
              → state: READY → CALIBRATING → ENABLED
              ★ 校准+透传完成, 电机就绪 ★

步骤 5 [透明]  motor_tool torque 1 500           # 控制
              motor_tool abs 1 9000
              → state: ENABLED → RUNNING (第一个cmd)
              → 控制命令透明: 不关心使能/模式/时序
```

---

## 9. 日志架构

```
┌─ motor_hal 层 (纯 C) ─────────────────────────┐
│  RT 路径: ring buffer (50行C, lock-free)      │
│  非RT:     log_fn 回调 → motor_node 统一输出    │
└────────────────────────────────────────────────┘
┌─ motor_node 层 (C++) ──────────────────────────┐
│  RT 工作线程: ring buffer                      │
│  日志线程:    ring buffer drain → ECO_INFO      │
│  主线程/ROS:  ECO_INFO/ECO_WARN/ECO_ERROR       │
│  统一输出:    log_helper (量产验证)              │
└─────────────────────────────────────────────────┘
```

---

## 10. 文件规划

```
exo_node/
├── CMakeLists.txt              # 编译控制 (ENABLE_ROS=OFF/ON)
├── exo_shm.h                   # ★ 算法和系统侧共享的唯一头文件
├── src/
│   ├── main.cpp                # 入口: 状态机同步执行 + main loop
│   ├── exo_state_machine.cpp   # 7状态 enter/exit + state_transition
│   ├── exo_rt_worker.cpp       # RT 工作线程 (控制+上报+安全)
│   ├── exo_log.cpp             # ring buffer + log_helper 适配
│   ├── exo_shm.cpp             # SHM open/close/mmap
│   └── exo_calib.cpp           # 校准流程封装
├── ros/
│   ├── RosAdapter.cpp          # ROS Service/Topic (编译可选)
│   └── RosAdapter.h
└── tools/
    └── motor_tool              # 已有, 调试用 CLI

依赖:
  libmotor_hal.a    — CANFD HAL
  libmotor_calib.a  — 校准模块
  log_helper        — 量产日志库 (需 RV1126B 交叉编译版本)
```

---

## 11. 关键设计决策索引

| 决策 | 结论 | 详见 |
|------|------|------|
| 线程合并 | 4→2 (控制+上报+安全 三合一) | known_issues.md Q1 |
| 心跳精简 | seq_begin 一种机制三种用途 | known_issues.md Q2 |
| 故障分类 | severity(动作) + reason(诊断) | known_issues.md Q3 |
| 安全进程 | 当前不拆, 电机关门狗兜底 | known_issues.md Q4 |
| 状态机 | 7状态 event-driven, 无独立线程 | known_issues.md Q5 |
| 日志系统 | motor_hal 回调注入 + log_helper | known_issues.md Q6 |
| SHM 反馈帧 | 显式 struct, 含 IMU, 编译期对齐 | known_issues.md Q7 |
| ABA 保护 | uint64_t seq, 永不回绕 | known_issues.md Q8 |
| 使能分离 | daemon 启动时做一次, 控制不再重复 | tool_hal.c fix |
| CRIT-1 死锁 | auto-startup 移出 recv 线程 | motor_hal.c fix |
