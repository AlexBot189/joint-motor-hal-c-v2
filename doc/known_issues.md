# Known Issues & Design Decisions

> motor_hal_c + motor_tool + motor_node 项目问题和设计决策记录  
> 最后更新: 2026-06-08

---

## 代码审查发现的问题 (2026-06-05)

### 🔴 CRIT-1: 接收线程中触发 auto-startup 死锁

**状态**: ❌ 未修复 (触发条件需同时满足 auto_enable=true + motor_state==NOT_READY + bootup 帧到达)

**现象**: `_dispatch_frame` case 0x700 中检测到 bootup 时直接调用 `motor_startup_full`(SDO 操作), 但 SDO 响应需要接收线程自己通过 `sdo_push_response` 入队, 形成死锁。

**风险**: 当前 daemon 中 auto_enable=false 由 startup 命令显式调用, 不触发。但未来如果改配置为 auto_enable=true 会触发。

---

### 🔴 CRIT-2: motor_calib.c 控制字 size=3 错误

**状态**: ❌ 未修复

**现象**: 控制字 0x6040 是 UINT16 (2字节), 代码中多处写了 `size=3`。

**位置**: `motor_calib.c` — `motor_calib_start`, `motor_calib_poll`, `motor_calib_exit`

**影响**: 驱动板可能拒绝 (SDO ABORT), 校准流程失败。

**修复**: 全部改为 `size=2`。

---

### 🟢 CRIT-3: COB_SYNC == COB_EMCY_BASE 宏冲突

**状态**: ✅ 已确认不是 bug

**分析**: CANopen 标准规定 SYNC 和 EMCY 共享 function code 0001b。SYNC 通过 node-id=0 区分。`_dispatch_frame` case 0x080 中 `canopen_extract_node(f->id, 0x080)` 对 SYNC 返回 node=0, `_find_motor` 返回 NULL, 安全跳过。GD32 源码中也无 EMCY 处理代码。

---

### 🟡 MED-1: sdo_mode_map 值需确认

**状态**: ✅ 已确认不需要改

**结论**: 阳志强确认巨蟹使用自己的 mode 映射值 (与 CiA 402 标准值不同)。

---

### 🟡 MED-2: _find_motor 无锁竞态

**状态**: ✅ 已修复 (2026-06-05)

**修复**: 7个实时控制函数全部加了 `hal->lock` 保护。临界区 ~1-3μs, PI mutex 无竞争时 ~50ns, 不影响 RT 实时性。

---

### 🟡 MED-3: timeout_ms 未传递给 bootup 等待

**状态**: ✅ 已确认不是 bug (设计如此)

**结论**: v4 设计意图: bootup 试探固定 500ms, 真正的"电机在线"验证走步骤 2 的同步 SDO。阳志强确认 "startup 不需要传递 timeout_ms"。

---

### 🟡 MED-4: heartbeat_set_period fire-and-forget

**状态**: ✅ 已规避 (v4 已改用同步 SDO)

**结论**: v4 的 `motor_startup_full` 步骤 2 已改用同步 `sdo_write_simple` 写心跳。旧函数仍存在但当前路径不触发。

---

### 🟡 MED-5: watch/report 线程退出未 join

**状态**: ❌ 未修复

**现象**: daemon 退出时 watch/report 线程未 join, `motor_hal_destroy` 可能销毁线程正在使用的 HAL 结构体。

**风险**: daemon 正常退出 (`motor_tool stop`) 时低频触发。

---

### 🟢 LOW-1: CANFD BRS 灵活性

**状态**: ✅ 已修复 (2026-06-05)

**修复**: `canfd_frame_t` 新增 `use_brs` 字段。SDO 帧 `is_fd=false` → 不加 BRS。PDO 帧 `is_fd=true` → 加 BRS 切 5Mbps。

---

### 🟢 LOW-2: can_driver_set_filter EFF_FLAG

**状态**: ✅ 已确认不需要改

**结论**: 项目只用 11 位标准帧, 不影响。

---

### 🟢 LOW-3: MIT torque int16 类型

**状态**: ✅ 已修复 (2026-06-05)

**修复**: `canopen_mit_pdo_build` 参数 torque 从 `uint16_t` → `int16_t`。`motor_hal_mit_control` 中 `int16_t torq = (int16_t)(torque * 1000.0f)` 保留负力矩符号。

---

### 🟢 LOW-4: PDO 映射通用接口

**状态**: ✅ 已修复 (2026-06-05 + 2026-06-07)

**修复**:
- `motor_hal_pdo_map()` 通用映射 API, 支持 RPDO/TPDO/PDO1/PDO2
- 更新为巨蟹文档时序: `disable others → trans_type → clear → map → count → enable`
- CLI: `tpdo/rpdo` 快捷映射, `tpdo_map/rpdo_map` 详细映射, `rpdo_send` 发送

---

## 功能新增 (2026-06-05 ~ 2026-06-07)

### motor_hal_rpdo_send
- 发送标准 RPDO 帧, COB 固定 0x200+node
- 数据小端, DLC ≤ 8

### motor_hal_set_tpdo_cb
- 注册标准 TPDO 原始帧回调
- 有回调时跳过硬编码解析, 由用户按自己的映射解析

### PDO 实时控制 CLI
- `motor_tool pdo <id> pos|vel|cur|csp <val>` — 单轴 PDO
- `motor_tool multi pos|vel|cur|csp <id1:val1> <id2:val2> ...` — 多轴广播, 自动回显反馈
- `motor_tool mit <id> <pos> <vel> <kp> <kd> <torque>` — MIT 阻抗

---

## 设计决策与 FAQ

### Q: auto_enable 和 motor_hal_startup 是否重复？
**A**: 不重复。`auto_enable` 控制 `motor_startup_full` 内部是否自动走 DS402 使能。`auto_enable=true` 一步到位, `auto_enable=false` 只配置不使能。

### Q: 使能逻辑在哪一层？
**A**: motor_tool (CLI) 层。`tool_torque_sdo/tool_speed_sdo/tool_abs_sdo` 内部都做了 `Shutdown → SwitchOn → EnableOp` 完整时序。HAL 层的 `motor_hal_sdo_write` 是纯 SDO 写, 不含使能逻辑。上层直接用 HAL API 需要自己调 `motor_hal_enable`。

### Q: motor_hal_set_position 是 PDO 还是 SDO？
**A**: PDO (自定义帧 0x100+ID, 7B)。SDO 位置控制用 `motor_hal_set_pos_target + motor_hal_set_pos_ctrl`。

### Q: 多控报文 (0x200) 需不需要映射？
**A**: 不需要。巨蟹自定义 PDO (0x100/0x110/0x200) 固件内置支持。只有标准 CANopen PDO (0x180+ID/0x200+ID) 才需要 SDO 映射。

### Q: 多控报文反馈是什么？
**A**: 每个电机各回一帧 0x300+ID (12字节), 不是 64 字节聚合帧。接收线程自动捕获存缓存。

### Q: RPDO 能同时下发电流+位置+速度吗？
**A**: 技术上可以, 但没意义。电机同一时刻只工作在一种控制模式, 只看当前模式对应的目标参数。正确做法是 `CW + 一个目标参数`。TPDO 多参数合理 (观测值同时存在)。

### Q: watch / report / sensor 区别？
**A**: 
- watch: 读反馈帧缓存 (0x300), 零配置
- report: 读反馈+传感器缓存, 需先开透传
- sensor: 读透传帧缓存 (0x680), 需 SDO 配置

### Q: MIT 会被默认启用吗？
**A**: 不会。`MOTOR_MODE_MIT` 只在 `motor_hal_mit_control()` API 中, 无一启动代码/daemon 默认调用。

### Q: 映射为什么有 8 个 TPDO + 8 个 RPDO？
**A**: CiA 301 标准规定的预定义连接集数量 (0x1800~0x1803, 0x1400~0x1403)。巨蟹文档关闭多余的 (1801~1803, 1401~1403), 保留 PDO1。默认一个 TPDO1 + 一个 RPDO1 够用, 多个通道用于不同频率的数据分离。

### Q: SYNC 帧是做什么的？
**A**: 触发从站 PDO 上报 + 同步多个电机。SDO 没有 SYNC 概念 (SDO 是点对点问答)。

### Q: CANFD 64 字节能力用在哪？
**A**: 标准 TPDO 映射 (0x180+ID) 在 CANFD 下可达 64 字节。0x300 反馈帧是固定 12 字节, 不可改。多控报文 (0x200) 也用满 64 字节。

---

## 待修复问题

| 优先级 | 问题 | 位置 |
|--------|------|------|
| 🔴 | motor_calib.c 控制字 size=3 | `src/motor_calib.c` |
| 🔴 | auto-startup 死锁 | `src/motor_hal.c` _dispatch_frame |
| 🟡 | watch/report 线程 daemon 退出未 join | `tools/cmd_watch.c`, `tools/cmd_report.c` |

---

## MIT 控制律

```
τ = Kp × (θ_target - θ_actual) + Kd × (θ'_target - θ'_actual) + τ_ff

Kp: 位置刚度 [0~500]  (越大越硬)
Kd: 速度阻尼  [0~5]    (越大阻力越大)
τ_ff: 前馈力矩 (可正可负)
```

## PID 参数对应

| 参数 | OD Index | 含义 |
|------|----------|------|
| CP | 0x2532 | 电流环 P 增益 |
| CI | 0x2533 | 电流环 I 增益 |
| VP | 0x2534 | 速度环 P 增益 |
| VI | 0x2535 | 速度环 I 增益 |
| PP | 0x2536 | 位置环 P 增益 |
| PI | 0x2537 | 位置环 I 增益 |

> 写入 OD 2532~2537 只改 RAM。需 `motor_tool save <id>` 写 Flash 才能掉电保留。

---

## SHM + RT 架构设计讨论 (2026-06-08)

> 参与者: 阳志强、张君宝  
> 背景: RV1126B 四核 A35, 当前 2 髋关节电机, 未来扩展 4 电机(髋+膝)

---

### Q1: 4 RT 线程能否合并为 2 个？

**结论**: ✅ 合并。4 个 RT 线程 → 2 个(RT 工作线程 + CAN 接收线程)。

**原设计**:
| 线程 | 策略 | 优先级 | 周期 |
|------|------|:---:|------|
| RT 控制线程 | SCHED_FIFO | 90 | 1KHz |
| CAN 接收线程 | SCHED_FIFO | 85 | 事件驱动 |
| RT 上报线程 | SCHED_FIFO | 80 | 200Hz |
| 安全监控线程 | SCHED_FIFO | 75 | 100Hz |

**简化为**:
| 线程 | 策略 | 优先级 | 周期 |
|------|------|:---:|------|
| RT 工作线程 | SCHED_FIFO | 90 | 1KHz |
| CAN 接收线程 | SCHED_FIFO | 85 | 事件驱动(HAL 内部) |

**RT 工作线程单周期(1KHz)任务**:
1. read mailbox.cmd → motor_hal_set_torque() → PDO write
2. read fb_cache → 每 5 周期 write SHM double buffer (200Hz 效果)
3. safety check: seq_begin 不变检测、编码器异常、过温、CAN 断线
4. 首次检测到 seq_begin 变化 → state_transition(RUNNING)

**时序预算验证(4 电机)**:
```
read mailbox.cmd              < 1μs
multi_ctrl(cmd, 4) → PDO       ~20μs  (1 帧 64B 广播, 非 4 次 write)
read fb_cache (PI mutex)       < 10μs
write SHM double buffer        < 5μs   (每 5 周期)
safety checks                  < 10μs
─────────────────────────────────────
合计                          < 50μs

周期预算 1000μs → 占用 < 5%, 余量充足
```

**优化理由**:
- 减少 2 次 RT 上下文切换, 每毫秒省 ~15μs
- 消除控制线程和上报线程对 fb_cache 的锁竞争
- 安全检测从 100Hz 提升到 1KHz 逐周期检查, 故障响应快 10 倍
- RV1126B 四核 A35 核心数充足, 但上下文切换和锁竞争开销不随核心数增加而消失

**multi_ctrl 扩展性**:
4 电机场景用 `motor_hal_multi_ctrl()` 广播, 1 帧 64 字节 CANFD 全部发完(最多支持 8 轴)。
不是 4 次单独 PDO write, 开销几乎不增加。

---

### Q2: 三套心跳/序列号能否精简？

**结论**: ✅ 删除 algo_heartbeat 和 cmd_seq, 只保留 mailbox seq_begin。

**原设计**: algo_heartbeat(100Hz 递增), cmd_seq(mailbox 写计数), seq_begin/seq_end(mailbox 快照)

**简化后**: 仅 mailbox seq_begin 一种机制, 三种用途:
1. 命令传输 — 算法每周期写 cmd + 递增 seq
2. 握手信号 — motor_node 检测到 seq_begin 第一次非零 → 切 RUNNING
3. liveness 心跳 — seq_begin 200ms 不变 → 算法失联

**算法侧硬性契约**: 算法必须每个控制周期都写 mailbox 并递增 seq, 即使 torque=0 也不例外。
如果算法想暂停控制, 发 torque=0 而不是不发。seq_begin 的连续性就是 liveness 的可靠指标。

**SHM 字段变化**: algo_heartbeat(删除) + cmd_seq(删除) + algo_ready(删除, 改用 seq_begin)

---

### Q3: motor_fault bitfield 如何简化？

**结论**: ✅ 严重级别 + 原因 双字段分离, 替代 8-bit 精确 bitfield。

**原设计**: 8 个 bit 分别标记不同故障, 但 bit0(心跳200ms)和 bit2(命令200ms)都触发 torque=0, 含义重叠。

**新方案**:
```c
// 动作分类 — 决定做什么
typedef enum {
    MOTOR_OK    = 0,  // 正常运行
    MOTOR_WARN  = 1,  // 降额: torque=0, 保持使能(可自动恢复)
    MOTOR_FAULT = 2,  // 停机: DS402 Shutdown(需人工干预)
} motor_severity_t;

// 原因枚举 — 诊断用
typedef enum {
    FAULT_NONE             = 0,
    FAULT_ALGO_TIMEOUT,        // 算法失联(seq_begin 200ms 不变)
    FAULT_CMD_STALL,           // 输出停滞(同一 cmd 重复 500ms)
    FAULT_CAN_OFFLINE,         // CAN 断线(2s 无帧)
    FAULT_ENCODER_FAULT,       // 编码器异常(position 恒定 3s)
    FAULT_OVERTEMP,            // 驱动器过温(> 80°C)
    FAULT_HANDSHAKE_TIMEOUT,   // 握手超时(ENABLED 状态 10s 无 cmd)
    FAULT_CALIB_TIMEOUT,       // 校准超时
} fault_reason_t;
```

SHM 中放两个字段:
```c
uint8_t motor_severity;  // 0/1/2 — 决定动作
uint8_t fault_reason;    // 枚举 — 诊断信息
```

8 个 bit 缩成 2 字节, 信息量反而增加了(枚举可扩展, 不限于 8 个原因)。

---

### Q4: 故障监控是否单独安全进程？

**结论**: ❌ 当前阶段不拆。全部安全逻辑内聚在 RT 工作线程。

**场景分析**:

| 场景 | 安全进程路径 | 可靠性 |
|------|------------|--------|
| A: 安全进程走 CAN/PDO 停机 | motor_node 崩溃 → CAN 静默 → 安全进程发不了 PDO | ❌ 形同虚设 |
| B: 安全进程走独立 GPIO 急停 | motor_node 崩溃 → 拉 GPIO → 驱动板硬件停机 | ✅ 真正冗余 |

场景 A 下, 安全进程和 motor_node 共享同一个故障面(CAN 总线), 分离不增加安全性。
场景 B 才是安全分离的意义所在。

**分阶段策略**:

| 阶段 | 方案 | 说明 |
|------|------|------|
| 当前(开发测试, 2电机髋关节) | RT 工作线程内聚 + 内核 watchdog `/dev/watchdog` | motor_node 周期性 ioctl WDIOC_KEEPALIVE; 进程 crash → 内核重启 → GPIO 复位 → 驱动板硬件停机 |
| 量产(4电机全下肢, 有人穿戴) | 独立安全 MCU(STM32/GD32) 直连急停 GPIO | MCU 监控 RV1126B 心跳 + 冗余 CAN 读反馈, 不依赖 RV1126B 操作系统 |

**不拆的核心理由**:
1. 安全监控需要的所有数据(fb_cache, mailbox, HAL 内部状态)全在 motor_node 内存中。拆到独立进程需要 SHM 桥接, 多一次拷贝和延迟。
2. 电机自带看门狗已是硬件级安全网: motor_startup_full 配置 0x1017 心跳 + 0x2650 看门狗, motor_node crash → SYNC 停发 → 电机看门狗超时 → 自动安全停机。
3. 安全逻辑和动作执行天然耦合: 检测到过热 → 调 motor_hal_set_torque(0), 这个 API 在 motor_node 里。

---

### Q5: 是否需要状态机？什么状态？怎么实现？

**结论**: ✅ 需要状态机(逻辑抽象), ❌ 不需要状态机线程(用事件驱动 state_transition())。

**7 状态定义**:
```
INIT → DISCOVERY → READY → CALIBRATING → ENABLED → RUNNING
  ↑                                                    ↓
  └────────────────────────────────────────── FAULT ───┘
```

| 状态 | 含义 | 触发 | 做什么 |
|------|------|------|--------|
| INIT | 进程启动 | 自动 | open CAN, create SHM, add motors, recv_start |
| DISCOVERY | 探测在线 | INIT 完成 | 等 bootup → motor_startup_full(配心跳+关狗+读固件) |
| READY | 电机就绪 | 全部在线 | 等校准命令(GPIO/ROS Service) |
| CALIBRATING | 执行校准 | 按键/Service | 设零位 → poll 验证 ±1° |
| ENABLED | 等待算法 | 校准成功 | 使能电机, torque=0, 等第一个 mailbox cmd |
| RUNNING | 运控中 | 第一个 cmd | RT 线程 forwarding cmd, 安全监控激活 |
| FAULT | 故障停机 | 安全检测 | DS402 Shutdown, 等恢复命令 |

**新增 DISCOVERY 的理由**: 原设计把"进程初始化"和"电机在线探测"混在 INIT 里。
分开后 DISCOVERY 专门调 `motor_startup_full`, INIT 只做硬件无关的初始化。

**真实应用**:
- LED 指示灯: 蓝闪(INIT) → 蓝常(DISCOVERY/READY) → 黄闪(CALIBRATING) → 黄常(ENABLED) → 绿常(RUNNING) → 红闪(FAULT)
- 物理按键: READY 按 1 次=校准, ENABLED 按 1 次=使能/失能, RUNNING 按 1 次=急停→FAULT
- ROS Service 权限控制: enable 服务只在 ENABLED 状态接受, 其他状态拒绝

**实现方式(不走独立线程)**:
```c
typedef enum {
    STATE_INIT, STATE_DISCOVERY, STATE_READY,
    STATE_CALIBRATING, STATE_ENABLED, STATE_RUNNING, STATE_FAULT
} exo_state_t;

// 每个状态的进入函数(可阻塞, 非 RT)
static void enter_init(void);
static void enter_discovery(void);  // 等 bootup → motor_startup_full
static void enter_ready(void);      // 空, 等外部事件
static void enter_calibrating(void);
static void enter_enabled(void);
static void enter_running(void);
static void enter_fault(void);

// 统一状态切换
void state_transition(exo_state_t new_state) {
    exit_hooks[g_state]();   // 离开旧状态(清理)
    g_state = new_state;
    ECO_INFO("State: %s", state_name(g_state));
    enter_hooks[g_state]();  // 进入新状态(设置)
}

// 主线程
int main() {
    state_transition(STATE_INIT);       // 同步跑完 INIT → DISCOVERY → READY
    rt_worker_start();                  // 启动 RT 线程(torque=0, 等 cmd)
    main_event_loop();                  // ROS Service / GPIO 事件驱动状态切换
}
```

**设计原则**:
| 原则 | 说明 |
|------|------|
| 同步执行 | enter_*() 可以阻塞(非 RT), 等 SDO 响应等硬件事件 |
| 超时保护 | 每个阻塞操作都要超时, 不能无限等 |
| 幂等安全 | 重复调用同一个 enter 不出问题 |
| 状态记录 | 每次 transition 打 ECO_INFO 日志 |

**禁止的路径**:
- INIT → RUNNING (跳过在线检测+校准+使能)
- FAULT → RUNNING (需要经过恢复流程)
- RUNNING → CALIBRATING (运行中不能校准)

---

### Q6: 日志系统 — moodycamel vs log_helper vs ring buffer？

**结论**: ✅ motor_node 层引入 log_helper(ECO_ 宏), RT 路径走 ring buffer 过渡。

**log_helper 特征**(来自 petrobot_periph_manager 分析):
- 头文件: `<log_helper/LogHelper.h>`
- 12 种宏: ECO_INFO/ECO_WARN/ECO_ERROR/ECO_DEBUG/ECO_INFO_NEW/ECO_DEBUG_NEW/ECO_ERROR_STREAM 等
- 5 级日志: TRACE/DEBUG/INFO/WARN/ERROR
- ECO_LOG_THROTTLE 限速日志(控制循环中用)
- MODULE_NAME 编译时标记模块
- 量产验证(petrobot 线上在跑)
- **预编译 .so, 无源码, C++ 库**

**分层策略**:
```
┌─ motor_hal 层 (纯 C) ─────────────────┐
│ RT 路径: 不用日志, 只写 ring buffer   │
│ 非RT: 通过回调注入 motor_node 日志     │
│ 回调接口: motor_hal_set_log_fn()      │
└────────────────────────────────────────┘
┌─ motor_node 层 (C++) ──────────────────┐
│ RT 工作线程: ring buffer (50行C)      │
│ 日志输出线程: log_helper (ECO_*)      │
│ 主线程/ROS: log_helper (ECO_*)        │
└────────────────────────────────────────┘

数据流:
RT 线程 → ring buffer (lock-free, <1μs) → 日志输出线程 → ECO_INFO → log_helper → file/syslog
非RT 代码 → ECO_INFO/WARN/ERROR 直接调用 → log_helper
```

**motor_hal 日志回调**(10 行代码, 不引入 C++ 依赖):
```c
// motor_hal.h
typedef enum { HAL_LOG_DEBUG, HAL_LOG_INFO, HAL_LOG_WARN, HAL_LOG_ERROR } hal_log_level_t;
typedef void (*hal_log_fn)(hal_log_level_t level, const char *file, int line, const char *msg);
void motor_hal_set_log_fn(hal_log_fn fn);  // 默认 = 内部 fprintf

// motor_node 注入:
static void hal_to_eco(hal_log_level_t lv, const char *f, int ln, const char *msg) {
    switch (lv) {
    case HAL_LOG_WARN:  ECO_WARN("[HAL] %s:%d %s", f, ln, msg); break;
    case HAL_LOG_ERROR: ECO_ERROR("[HAL] %s:%d %s", f, ln, msg); break;
    default:            ECO_INFO("[HAL] %s %s", f, msg); break;
    }
}
motor_hal_set_log_fn(hal_to_eco);
```

**RT 日志 ring buffer**(替代 moodycamel::ConcurrentQueue, 50 行纯 C):
```c
#define LOG_BUF_SIZE 256
#define LOG_ENTRIES 128

static char    log_ring[LOG_ENTRIES][LOG_BUF_SIZE];
static _Atomic uint32_t log_wr;  // RT 线程写
static uint32_t log_rd;          // 日志线程读(单消费者, 不用 atomic)

// RT 线程写(lock-free, <1μs):
void rt_log_push(const char *msg) {
    uint32_t next = (atomic_load(&log_wr) + 1) % LOG_ENTRIES;
    if (next == log_rd) return;  // 满则丢弃, RT 不能阻塞等
    memcpy(log_ring[atomic_load(&log_wr)], msg, LOG_BUF_SIZE);
    atomic_store(&log_wr, next);
}

// 非 RT 日志线程读:
void rt_log_drain(void) {
    uint32_t w = atomic_load(&log_wr);
    while (log_rd != w) {
        ECO_INFO("RT: %s", log_ring[log_rd]);
        log_rd = (log_rd + 1) % LOG_ENTRIES;
    }
}
```

**ABI 兼容性**:
rk3576(ARMv8.2-A, Aarch64) 和 RV1126B(ARMv7-A, Aarch32), log_helper .so 一定不兼容。
需要重新交叉编译 RV1126B 版本。如果短期拿不到, 先用 ring buffer + fprintf 过渡, 接口预留 log_helper 注入点。

**为什么不用 moodycamel::ConcurrentQueue**:
- 3747 行 C++ 模板, 为 256 字节日志引入太重
- RT 单写单读场景, 50 行 C ring buffer 完全够用
- 减少编译依赖, 降低模板膨胀

---

### Q7: SHM 反馈帧需要显式 struct 定义

**结论**: ✅ 用显式 typed struct, 禁止 `~140B blob`。

**问题**: v2.0 设计文档写 `fb_buffer[0] (~140B)` 但没有定义内部字段。算法和系统侧必须字节级对齐。

**方案**:
```c
typedef struct {
    // 电机反馈 (来自 0x300 反馈帧, 大端)
    struct {
        int16_t position;       // ° × 100  (编码器角度)
        int16_t velocity;       // RPM
        int16_t current_iq;     // mA (Q轴电流)
        int16_t temperature;    // °C × 10
        uint8_t status_byte;    // bit7:使能 bit6:抱闸 bit5:错误 bit4:到位
        uint8_t mode;           // 当前控制模式
        uint8_t error_code;     // 故障码 (高4位=故障类型, 低4位=子码)
        uint8_t _pad;
    } motor[2];                 // [0]=右(ID=1) [1]=左(ID=2)

    // 透传传感器 (来自 0x680, 小端 bit-packed)
    struct {
        uint16_t hall_adc0;     // 线性霍尔A, 0~4095
        uint16_t hall_adc1;
        uint16_t hall_adc2;
        uint16_t force_raw;     // DF181力矩, 0~16383
        uint16_t knee_adc;      // 膝关节电位器, 0~4095
        uint8_t  key_landing;   // 着地开关
        uint8_t  data_valid;    // 力矩有效标志
    } sensor[2];

    // IMU (来自 SPI 直连 ICM45608)
    struct {
        float    roll, pitch, yaw;
        float    acc_x, acc_y, acc_z;
        float    gyro_x, gyro_y, gyro_z;
        uint64_t timestamp_us;
    } imu;

    uint64_t timestamp_us;      // 组装时刻
} feedback_frame_t;

// 编译期强制对齐检查
_Static_assert(sizeof(feedback_frame_t) <= 256, "feedback_frame too large");
_Static_assert(offsetof(feedback_frame_t, motor[1].position) % 2 == 0, "motor alignment");
```

**注意事项**:
- arm-linux-gnueabihf (Aarch32) 下 `float` 是 4 字节对齐, struct 整体 4 字节对齐
- 算法进程和 motor_node 用同一份 `exo_shm.h`, 编译期保证一致
- 如果未来加膝关节电机, motor[4] 而不是 motor[2], 扩展预留到 512B

---

### Q8: Mailbox Snapshot 的 ABA 问题

**结论**: ✅ 显式声明 uint64_t writer_seq, 实际永不回绕, 无需额外 ABA 保护。

**分析**:
```
writer_seq 是 64 位原子递增:
2^64 次写 ÷ 1000Hz = 2^64 ÷ 1000 ÷ 86400 ÷ 365 ≈ 5.85 亿年
```

ARMv8 A35 下 `atomic_fetch_add(&writer_seq, 1, release)` 是单条 LDADD 指令, 天然原子。
Reader 端 `last_seq` 同理 uint64_t, 不需要额外 ABA 标记。

**要求**: `exo_shm.h` 中所有 seq 变量**必须**显式声明 `uint64_t`, 禁止:
```c
unsigned long writer_seq;  // ❌ 在 32-bit Aarch32 下是 32 位!
```
```c
uint64_t writer_seq;       // ✅ 显式 64 位
uint64_t last_seq;         // reader 缓存的比较值
```

---

### 已确认合理设计 (保留)

阳志强 2026-06-08 审查确认以下设计合理, 无需修改:

| # | 设计 | 理由 |
|---|------|------|
| 1 | 双进程 SHM 架构 | 算法 crash 不影响 motor_node → 安全停机, 机器人通行做法 |
| 2 | Double Buffer + release/acquire | ARMv8 弱序下保证无撕裂读, 单写单读最优解 |
| 3 | Mailbox Lock-Free Snapshot (seq_begin/seq_end) | 算法写中途 reader 跳过, 无锁安全 |
| 4 | SDO/PDO 分离 | SDO 走阻塞等响应(队列+condvar), PDO 走非阻塞 write(sockfd), 互不干扰 |
| 5 | CMake 编译控制 ROS/WebServer | 生产环境不编入, 减小攻击面和资源占用 |
| 6 | 电机自带看门狗 (0x1017 + 0x2650) | motor_node crash → SYNC 停发 → 电机看门狗超时 → 硬件级安全停机 |
| 7 | 非 RT 消费者从 SHM pull (RosListener/WsListener) | 自己去 SHM 读双 Buffer, 不依赖 push 通知 |

---

## 最终架构修订摘要

### 线程模型
```
RT 工作线程 (prio=90, 1KHz): 控制+上报+安全 三合一
CAN 接收线程 (prio=85, HAL 内部): 唯一 recv 入口
主线程 (SCHED_OTHER): 事件驱动状态机, ROS Service
日志输出线程 (SCHED_OTHER): ring buffer drain → log_helper
```

### SHM 精简
| 字段 | 原设计 | 修改后 | 理由 |
|------|--------|--------|------|
| active_idx | ✅ | ✅ | 双 Buffer 切换 |
| fb_buffer[2] | ~140B blob | 显式 struct(含 IMU) | 跨进程对齐 |
| mailbox | ✅ | ✅ | 无锁快照 |
| algo_heartbeat | ✅ | ❌ 删除 | seq_begin 代替 |
| cmd_seq | ✅ | ❌ 删除 | seq_begin 代替 |
| algo_ready | ✅ | ❌ 删除 | 第一个 cmd 代替 |
| motor_fault | 8 bit | severity(1B) + reason(1B) | 动作+原因分离 |
| motor_online | ✅ | ✅ | 电机在位 |
| calib_state | ✅ | ✅ | 校准进度 |
| node_state | ✅ | 合并到 severity | 状态机管理 |

### 状态机
```
7 状态: INIT → DISCOVERY → READY → CALIBRATING → ENABLED → RUNNING → FAULT
实现: event-driven state_transition(), 无独立线程
```

### 日志
```
motor_hal(C): 回调注入 → motor_node 统一输出
motor_node(C++): log_helper(ECO_*) + RT ring buffer
```

---

## 待办事项

| 优先级 | 事项 | 状态 |
|--------|------|:--:|
| 🔴 | 获取 RV1126B 版本 log_helper .so 或确认替代方案 | 待定 |
| 🟡 | 确认反馈帧 SHM struct 字节对齐(跨 ARMv7 进程) | 待实现 |
| 🟡 | 内核 watchdog 配置 `/dev/watchdog` 测试 | 待测试 |
| 🟢 | CPU isolation (isolcpus=2,3) + RT 线程 affinity | 优化项 |
