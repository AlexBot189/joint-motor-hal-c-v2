# Motor HAL — 关节模组 CANopen/CANFD 硬件抽象层

纯 C 关节电机控制库，基于 CANopen CiA 402 协议，适配**无锡巨蟹智能驱动**一体化通讯模组。

## 目录

- [快速开始](#快速开始)
- [架构概览](#架构概览)
- [API 参考](#api-参考)
  - [生命周期](#1-生命周期)
  - [电机管理](#2-电机管理)
  - [启动与停用](#3-启动与停用)
  - [实时控制](#4-实时控制)
  - [参数配置](#5-参数配置)
  - [状态查询](#6-状态查询)
  - [反馈缓存](#7-反馈缓存)
  - [接收线程](#8-接收线程)
  - [回调系统](#9-回调系统)
  - [全局控制](#10-全局控制)
  - [工具函数](#11-工具函数)
- [核心类型](#核心类型)
- [控制模式](#控制模式)
- [DS402 状态机](#ds402-状态机)
- [错误码](#错误码)
- [使用模式](#使用模式)
  - [简单模式 (poll)](#简单模式)
  - [推荐模式 (recv 线程 + 控制线程)](#推荐模式)
  - [RT 实时控制模式](#rt-实时控制模式)
- [编译与链接](#编译与链接)
- [motor_tool 命令行工具](#motor_tool-命令行工具)

---

## 快速开始

```c
#include "motor_hal.h"

int main(void) {
    // 1. 创建 HAL
    motor_hal_t *hal = motor_hal_create();
    motor_hal_init(hal, "can0", 1000000, 5000000);

    // 2. 注册电机
    motor_config_t cfg = {0};
    cfg.node_id = 1;
    cfg.disable_watchdog = true;
    cfg.auto_enable = true;
    cfg.bootup_timeout_ms = 5000;
    motor_hal_add_motor(hal, &cfg);

    // 3. 启动接收线程 (v2 推荐)
    motor_hal_recv_start(hal);

    // 4. 上电启动
    motor_hal_startup(hal, 1, 5000);

    // 5. 控制循环
    while (running) {
        motor_feedback_t fb;
        motor_hal_get_feedback(hal, 1, &fb);       // 读反馈 (线程安全, 非阻塞)
        motor_hal_set_position(hal, 1, 30.0f);     // 发位置
    }

    // 6. 清理
    motor_hal_destroy(hal);
}
```

---

## 架构概览

```
应用层 (你的代码)
  │
  ├── 控制命令: motor_hal_set_position / set_velocity / mit_control ...
  │      ↓ PDO 帧 (低延迟, 直接控制电机)
  │
  ├── 参数配置: motor_hal_set_pid / set_mode / set_limits ...
  │      ↓ SDO 帧 (一问一答, 任意线程安全)
  │
  ├── 反馈: motor_hal_get_feedback / get_state / get_position ...
  │      ↑ 读缓存 (PI mutex, 几十ns) / SDO 查询 (200ms)
  │
  └── 接收线程 (motor_hal_recv_start):
         阻塞等待 CAN 帧 → 自动分发:
           SDO 响应 → 队列 (条件变量)
           PDO 反馈 → 缓存 + 回调
           Bootup/Heartbeat → 状态标志
           EMCY → 错误回调
```

**关键设计原则**:
- 控制线程只读缓存、只发 PDO，不调用 `recv`/`poll`/`select`
- 接收线程是唯一读 socket 的地方，阻塞 `recv` 不消耗 CPU
- SDO 在任意线程中同步调用，内部通过条件变量等待响应
- 所有反馈缓存用 PI mutex 保护，防止 RT 优先级倒置

---

## API 参考

### 1. 生命周期

| 函数 | 说明 |
|------|------|
| `motor_hal_create()` | 创建 HAL 实例，初始化内部数据结构。返回 NULL 表示内存不足。 |
| `motor_hal_destroy(hal)` | 销毁 HAL，自动停止接收线程、脱使能所有电机、关闭 CAN 接口。 |
| `motor_hal_init(hal, iface, arb, data)` | 打开 CANFD 接口。`iface` 如 `"can0"`，`arb`=仲裁段波特率(1M)，`data`=数据段波特率(5M)。返回 0 成功。 |

**初始化前必须确保 CAN 接口已配置**:
```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

### 2. 电机管理

| 函数 | 说明 |
|------|------|
| `motor_hal_add_motor(hal, cfg)` | 注册一个电机节点。`cfg` 被内部拷贝，可栈上分配。最多 16 个电机。 |
| `motor_hal_remove_motor(hal, node_id)` | 移除电机（紧凑数组，后续电机前移）。 |

`motor_config_t` 各字段含义:

| 字段 | 类型 | 说明 |
|------|------|------|
| `node_id` | uint8 | CAN 节点 ID (1~127) |
| `heartbeat_ms` | uint32 | 心跳周期 ms，默认 2000 |
| `profile_accel` | uint16 | 加速度，RPM/s，电机端 |
| `profile_decel` | uint16 | 减速度，RPM/s，电机端 |
| `profile_velocity` | uint16 | 轨迹速度，RPM，输出端 |
| `pos_limit_pos` | float | 正限位，度 |
| `pos_limit_neg` | float | 负限位，度 |
| `disable_watchdog` | bool | **推荐 true**：上电后关看门狗，省去喂狗开销 |
| `auto_enable` | bool | startup 时自动走 DS402 使能流程 |
| `bootup_timeout_ms` | int | 等待 Bootup 帧的超时时间，默认 5000 |

### 3. 启动与停用

| 函数 | 说明 |
|------|------|
| `motor_hal_startup(hal, id, timeout)` | **完整启动流程**：等 Bootup → 配心跳 → 关看门狗 → 读固件版本 → DS402 使能 → 等抱闸释放。返回 0 成功。 |
| `motor_hal_wait_bootup(hal, id, timeout)` | 仅等待 Bootup 帧，不使能。 |
| `motor_hal_enable(hal, id)` | DS402 使能：Shutdown → SwitchOn → EnableOp。 |
| `motor_hal_disable(hal, id)` | DS402 脱使能：发 Shutdown 命令。 |
| `motor_hal_fault_reset(hal, id)` | 故障复位：发 FaultReset → 回到 SwitchOnDisabled。 |

> **启动时序要求**: 调用 `motor_hal_startup` 前必须先调用 `motor_hal_recv_start`，否则 SDO 响应无人接收。

### 4. 实时控制

全部通过 PDO 帧下发，**非阻塞**，RT 安全。

| 函数 | 说明 | 适用模式 |
|------|------|---------|
| `motor_hal_set_position(hal, id, angle_deg)` | 设置目标角度（-180°~180°），内部自动转为编码器 count 值。 | PP / CSP |
| `motor_hal_set_velocity(hal, id, rpm)` | 设置目标速度，电机端 RPM。正=正转，负=反转。 | PV / CSV |
| `motor_hal_set_torque(hal, id, current_ma)` | 设置目标 Iq 电流，mA。 | 电流环 |
| `motor_hal_mit_control(hal, id, pos, vel, kp, kd, torque)` | MIT 阻抗控制：力位混合。适合协作/柔顺场景。 | MIT |
| `motor_hal_ctrl_raw(hal, id, mode, t1, t2, ff)` | 通用 PDO 控制：指定模式，裸参数。最高灵活性。 | 任意 |
| `motor_hal_stop(hal, id)` | 立即停止运动（发送 target=当前位置）。 | 任意 |

**控制模式切换**: 调用 `motor_hal_set_mode` 切换模式后再发送对应控制命令。模式会触发电机内部控制环切换，有一定延时。

**MIT 阻抗控制参数说明**:
- `position` / `velocity`: 目标位置 / 速度
- `kp` / `kd`: 刚度 / 阻尼系数（值越大越"硬"）
- `torque`: 前馈力矩

### 5. 参数配置

通过 SDO 读写电机参数，**同步阻塞**（约 50~200ms），任意线程可调。

| 函数 | 说明 |
|------|------|
| `motor_hal_set_mode(hal, id, mode)` | 切换控制模式。mode 取值见 [控制模式](#控制模式)。 |
| `motor_hal_set_accel_decel(hal, id, accel, decel)` | 设置加减速度，RPM/s。 |
| `motor_hal_set_profile_velocity(hal, id, rpm)` | 设置位置模式下的最大轨迹速度，RPM。 |
| `motor_hal_set_pid(hal, id, pid)` | 写入全部 6 个 PID 参数（电流/速度/位置环的 P/I）。 |
| `motor_hal_read_pid(hal, id, &pid)` | 读取全部 PID 参数。 |
| `motor_hal_save_flash(hal, id)` | 将当前参数保存到 Flash（掉电不丢失）。 |
| `motor_hal_set_zero(hal, id)` | 触发零位标定（当前位置记为零点）。 |
| `motor_hal_set_limits(hal, id, pos_deg, neg_deg)` | 设置软限位。 |
| `motor_hal_disable_watchdog(hal, id)` | 关闭看门狗（推荐，避免喂狗开销）。 |

### 6. 状态查询

通过 SDO 同步查询，阻塞 50~200ms。

| 函数 | 返回值 | 说明 |
|------|--------|------|
| `motor_hal_get_state(hal, id)` | `motor_state_t` | DS402 状态 |
| `motor_hal_get_statusword(hal, id)` | `uint16_t` | 状态字原始值 |
| `motor_hal_get_position(hal, id)` | `int32_t` | 当前编码器位置（counts） |
| `motor_hal_get_velocity(hal, id)` | `int32_t` | 当前速度（通过 SDO 读 0x606C） |
| `motor_hal_get_current(hal, id)` | `int32_t` | 当前电流，mA |
| `motor_hal_sdo_read_u32(hal, id, idx, sub, &val)` | int | **通用 SDO 读**，读任意对象字典，返回 uint32 |

> **注意**: 运行时的状态查询用 `motor_hal_get_feedback` 读缓存（非阻塞），不要用 SDO 查询（阻塞）。

### 7. 反馈缓存

| 函数 | 说明 |
|------|------|
| `motor_hal_get_feedback(hal, id, &fb)` | 读取最近一次反馈帧数据。线程安全拷贝，**非阻塞**。 |

`motor_feedback_t` 各字段:

| 字段 | 类型 | 说明 |
|------|------|------|
| `position` | int16 | 负载端编码器位置 (-32768~32767，对应 -180°~180°) |
| `velocity` | int16 | 电机端速度，RPM |
| `current_iq` | int16 | Q 轴电流，mA |
| `error_code` | uint16 | 错误码，0 表示正常 |
| `temperature` | int16 | 线圈温度，原始值（×0.1 = °C） |
| `mode` | uint8 | 当前控制模式 |
| `status_byte` | uint8 | bit7=使能, bit6=抱闸释放, bit5=错误, bit4=到位 |
| `timestamp_us` | uint64 | 接收时间戳，微秒 |

### 8. 接收线程

**v2 推荐**：用接收线程代替手动 poll。

| 函数 | 说明 |
|------|------|
| `motor_hal_recv_start(hal)` | 启动接收线程。线程内部阻塞 `recv`，自动分发 SDO/PDO/EMCY。 |
| `motor_hal_recv_stop(hal)` | 停止接收线程（阻塞等待线程退出）。 |
| `motor_hal_recv_is_running(hal)` | 查询接收线程是否在运行。 |

**接收线程的生命周期**:
```
motor_hal_init → motor_hal_recv_start → [startup/控制/查询] → motor_hal_destroy (自动 stop)
```

调用 `motor_hal_recv_start` 后，**不要再调 `motor_hal_poll`**（它会检测到 recv 线程在运行，直接 return）。

### 9. 回调系统

| 函数 | 说明 |
|------|------|
| `motor_hal_set_feedback_cb(hal, id, cb, ctx)` | 注册反馈回调。每收到反馈帧就调用，在**接收线程上下文**中执行。 |
| `motor_hal_set_error_cb(hal, id, cb, ctx)` | 注册错误回调。EMCY 帧或反馈帧错误位触发。 |
| `motor_hal_set_state_cb(hal, id, cb, ctx)` | 注册状态变更回调。初始化/enable/disable/fault 时触发。 |

**回调签名**:
```c
// 反馈: 每帧触发, 在接收线程中执行 — 不要做耗时操作!
void on_feedback(uint8_t node_id, const motor_feedback_t *fb, void *ctx);

// 错误: 异步触发
void on_error(uint8_t node_id, uint16_t error_code, void *ctx);

// 状态变更
void on_state(uint8_t node_id, motor_state_t old, motor_state_t new, void *ctx);
```

**回调使用建议**:
- 反馈回调中直接填充算法输入缓冲区（零拷贝）
- 不要在里面做 SDO 操作（会死锁）或长时间计算
- 错误回调里记录日志，逐级升级处理

### 10. 全局控制

| 函数 | 说明 |
|------|------|
| `motor_hal_nmt_broadcast(hal, cmd)` | 发送 NMT 广播命令，影响总线上所有节点。`cmd` 可取 `NMT_CMD_START/STOP/PRE_OP/RESET_NODE/RESET_COMM`。 |
| `motor_hal_sync(hal)` | 发送 SYNC 帧（0 字节，极低开销）。用于触发同步 PDO 或喂狗。 |
| `motor_hal_multi_ctrl(hal, cmds, count)` | **多轴广播控制**：一帧 CANFD（64 字节）同时控制最多 8 个电机。 |

```c
// 多轴广播示例: 一帧控制两个关节
multi_axis_cmd_t cmds[2] = {
    { .node_id=1, .mode=MOTOR_MODE_CSP, .enable=true, .target1=pos1 },
    { .node_id=2, .mode=MOTOR_MODE_CSP, .enable=true, .target1=pos2 },
};
motor_hal_multi_ctrl(hal, cmds, 2);
```

### 11. 工具函数

| 函数 | 说明 |
|------|------|
| `motor_counts_to_deg(counts)` | 编码器 count → 角度（°），inline |
| `motor_deg_to_counts(degrees)` | 角度（°）→ 编码器 count，inline |
| `motor_temp_to_c(raw)` | 温度 raw → °C（×0.1），inline |
| `motor_ma_to_a(ma)` | 电流 mA → A，inline |

---

## 核心类型

### motor_config_t — 电机配置
```c
typedef struct {
    uint8_t  node_id;             // CAN 节点 ID (1~127)
    uint32_t heartbeat_ms;        // 心跳周期 (默认 2000)
    uint16_t profile_accel;       // 加速度 RPM/s
    uint16_t profile_decel;       // 减速度 RPM/s
    uint16_t profile_velocity;    // 轨迹速度 RPM
    float    pos_limit_pos;       // 正限位 (°)
    float    pos_limit_neg;       // 负限位 (°)
    bool     disable_watchdog;    // 关看门狗 (推荐 true)
    bool     auto_enable;         // 自动使能
    int      bootup_timeout_ms;   // 启动超时
} motor_config_t;
```

### motor_feedback_t — 反馈数据
```c
typedef struct {
    int16_t  position;       // 编码器位置
    int16_t  velocity;       // RPM
    int16_t  current_iq;     // mA
    uint16_t error_code;     // 错误码
    int16_t  temperature;    // 温度 raw
    uint8_t  mode;           // 当前模式
    uint8_t  status_byte;    // bit 状态
    uint64_t timestamp_us;   // 时间戳
} motor_feedback_t;
```

### motor_pid_t — PID 参数
```c
typedef struct {
    uint16_t current_p, current_i;     // 电流环
    uint16_t velocity_p, velocity_i;   // 速度环
    uint16_t position_p, position_i;   // 位置环
} motor_pid_t;
```

### multi_axis_cmd_t — 多轴广播命令
```c
typedef struct {
    uint8_t   node_id;
    motor_mode_t mode;
    bool      enable;
    bool      release_brake;
    bool      clear_error;
    int16_t   target1;       // 位置/速度/电流
    uint16_t  target2;       // 加减速
    int16_t   feedforward;   // 前馈
} multi_axis_cmd_t;
```

---

## 控制模式

```c
typedef enum {
    MOTOR_MODE_PROFILE_POS  = 1,  // PP:  轮廓位置 — 梯形速度曲线到目标位置
    MOTOR_MODE_PROFILE_VEL  = 2,  // PV:  轮廓速度 — 梯形加速度曲线到目标速度
    MOTOR_MODE_CSP          = 3,  // CSP: 循环同步位置 — 上位机插补，每周期给目标位置
    MOTOR_MODE_CSV          = 4,  // CSV: 循环同步速度 — 上位机插补，每周期给目标速度
    MOTOR_MODE_CURRENT      = 5,  // 电流环 — 直接控制 Iq 电流
    MOTOR_MODE_MIT          = 6,  // MIT:  阻抗控制 — 力位混合，适合柔顺交互
} motor_mode_t;
```

**模式选择建议**:
- 关节点到点运动 → PP
- 速度控制（如轮式）→ PV
- 精密轨迹跟踪 → CSP（按周期发位置）
- 力控/柔顺 → MIT

---

## DS402 状态机

```
电源接通
  └→ NOT_READY → SWITCH_ON_DISABLED
                    │ CW_SHUTDOWN (0x06)
                    └→ READY_TO_SWITCH_ON
                         │ CW_SWITCH_ON (0x07)
                         └→ SWITCHED_ON
                              │ CW_ENABLE_OP (0x0F)
                              └→ OPERATION_ENABLED  ← 可控制
                                   │ CW_SHUTDOWN
                                   └→ READY_TO_SWITCH_ON
任何状态发生故障 → FAULT → CW_FAULT_RESET (0x80) → SWITCH_ON_DISABLED
```

---

## 错误码

| 错误码 | 含义 | 可能原因 |
|--------|------|---------|
| 0x0001 | 过压 | 供电电压过高 |
| 0x0002 | 欠压 | 供电电压不足 |
| 0x0004 | 过温 | 线圈温度过高 |
| 0x0008 | 堵转 | 电机卡死或负载过大 |
| 0x0010 | 过载 | 电流持续超过额定 |
| 0x0020 | 电流采样异常 | 硬件故障 |
| 0x0040 | 正限位触发 | 超出正向软限位 |
| 0x0080 | 负限位触发 | 超出负向软限位 |
| 0x0100 | 编码器超时 | 编码器通信故障 |
| 0x0200 | 超速 | 超过最大速度限制 |
| 0x0400 | 电角度初始化失败 | 上电自检失败 |
| 0x1000 | 位置误差过大 | 跟随误差超限 |
| 0x2000 | 编码器故障 | 编码器硬件异常 |

**状态字节快速检测**（feedback.status_byte，无需解析错误码）:
- `status_byte & 0x80` → 电机是否使能
- `status_byte & 0x40` → 抱闸是否释放
- `status_byte & 0x20` → **是否有错误**
- `status_byte & 0x10` → 是否到达目标位置

---

## 使用模式

### 简单模式

最低学习成本，适合调试和简单场景。

```c
motor_hal_init(hal, "can0", 1000000, 5000000);
motor_hal_add_motor(hal, &cfg);
motor_hal_recv_start(hal);         // 启动接收线程
motor_hal_startup(hal, 1, 5000);

while (running) {
    motor_feedback_t fb;
    motor_hal_get_feedback(hal, 1, &fb);
    motor_hal_set_position(hal, 1, target_deg);
    usleep(5000);                  // 200Hz
}
```

### 推荐模式

接收线程 + 反馈回调，零拷贝，效率最高。

```c
// 全局状态
static float g_current_angle = 0;

// 反馈回调 — 在接收线程中执行
static void on_feedback(uint8_t id, const motor_feedback_t *fb, void *ctx) {
    g_current_angle = motor_counts_to_deg(fb->position);
    if (fb->status_byte & 0x20) {
        fprintf(stderr, "[Motor %d] Error: 0x%04X\n", id, fb->error_code);
    }
}

// 错误回调
static void on_error(uint8_t id, uint16_t code, void *ctx) {
    // Level 1: 记录日志
    // Level 2: 连续错误 → fault_reset
    // Level 3: 复位失败 → 安全停机
}

int main(void) {
    motor_hal_t *hal = motor_hal_create();
    motor_hal_init(hal, "can0", 1000000, 5000000);

    motor_config_t cfg = { .node_id = 1, .disable_watchdog = true,
                           .auto_enable = true, .bootup_timeout_ms = 5000 };
    motor_hal_add_motor(hal, &cfg);

    // 注册回调 (必须在 recv_start 之前)
    motor_hal_set_feedback_cb(hal, 1, on_feedback, NULL);
    motor_hal_set_error_cb(hal, 1, on_error, NULL);

    // 启动接收线程 (必须在 startup 之前)
    motor_hal_recv_start(hal);
    motor_hal_startup(hal, 1, 5000);

    // 控制循环 — 只读缓存, 只发 PDO, 不触网卡
    while (running) {
        motor_hal_set_position(hal, 1, gait_calculate(g_current_angle));
        usleep(5000);
    }

    motor_hal_destroy(hal);
}
```

### RT 实时控制模式

接收线程和控制线程分离，控制线程跑 `SCHED_FIFO`。

```c
// 接收线程 (prio=78): motor_hal_recv_start 内部已实现

// 控制线程 (prio=80, 自行创建):
void *rt_control_thread(void *arg) {
    motor_hal_t *hal = (motor_hal_t*)arg;

    // 设置 RT 优先级
    struct sched_param sp = { .sched_priority = 80 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    while (running) {
        // 读反馈 — PI mutex, O(1), 几十 ns
        motor_feedback_t fb;
        motor_hal_get_feedback(hal, 1, &fb);

        // 步态算法
        float target = gait_algorithm(fb.position, fb.velocity);

        // 发控制 — write() 非阻塞
        motor_hal_set_position(hal, 1, target);

        // 周期睡眠
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}
```

**RT 模式关键点**:
- 控制线程**不调 poll/recv/select**，只读缓存 + 发 PDO
- 反馈缓存用 PI mutex（PTHREAD_PRIO_INHERIT），防止优先级倒置
- PREEMPT_RT 内核下接收线程的阻塞 recv 唤醒延迟微秒级

### 错误处理推荐策略

```c
static int g_error_count = 0;

static void on_error(uint8_t id, uint16_t code, void *ctx) {
    g_error_count++;
    motor_hal_t *hal = (motor_hal_t*)ctx;

    if (g_error_count < 3) {
        // Level 1: 单次错误，记录，继续
        log_warn("Motor %d err 0x%04X (count=%d)", id, code, g_error_count);
    } else if (g_error_count < 5) {
        // Level 2: 连续错误，尝试复位
        log_error("Motor %d err 0x%04X, fault reset", id, code);
        motor_hal_fault_reset(hal, id);
        motor_hal_enable(hal, id);
    } else {
        // Level 3: 不可恢复，安全停机
        log_fatal("Motor %d unrecoverable, emergency stop", id);
        motor_hal_disable(hal, id);
        trigger_emergency_stop();
    }
}
```

---

## 编译与链接

### 本机编译
```bash
mkdir build && cd build
cmake ..
make -j4
# 产物: libmotor_hal.a (静态库)
```

### 交叉编译 (RV1126B)
```bash
mkdir build_rv && cd build_rv
cmake .. -DCROSS_RV1126=ON -DCMAKE_SYSROOT=/opt/rv1126/sysroot
make -j4
```

### 应用链接
```bash
gcc -o my_robot my_robot.c -Ipath/to/motor_hal_c/inc \
    -Lpath/to/motor_hal_c/build -lmotor_hal -lpthread -lm
```

### 依赖
- Linux (SocketCAN)
- pthread
- math (libm)
- **无其他任何外部依赖**

### CAN 接口准备
```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

# 验证
ip -details link show can0
```

---

## motor_tool 命令行工具

框架附带一个命令行调试工具 `motor_tool`。

```bash
# 启动 daemon
motor_tool daemon can0 &

# 控制
motor_tool speed 1 5000        # 电机1: 50 RPM
motor_tool abs 2 4500          # 电机2: 45°
motor_tool rel 1 1000          # 电机1: 相对+10°
motor_tool stop                # 双电机停止

# 读取
motor_tool read all 0          # 读双电机全部状态
motor_tool read temp 1         # 读温度

# 监控
motor_tool watch 200           # 200ms 持续监控

# 停止
motor_tool stop                # 停止 daemon
```

编译工具: `cd tools/build && cmake .. && make`

---

## 交叉引用

- 硬件: RV1126B
- 驱动板: 无锡巨蟹智能驱动一体化通讯模组
- 协议: CANopen CiA 402 over CANFD
- 物理: 标准帧 11bit, 仲裁 1Mbps, 数据 5Mbps
