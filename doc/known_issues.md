# Known Issues & Design Decisions

> motor_hal_c + motor_tool 项目问题和设计决策记录  
> 最后更新: 2026-06-07

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
