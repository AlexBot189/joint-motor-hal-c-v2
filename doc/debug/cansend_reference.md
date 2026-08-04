# MIT / 力矩控制 调试手册

> 版本: V1.0 | 日期: 2026-08-04  
> 适用: joint-motor-hal-c-v3, KWS CANFD V2 协议  
> CAN 接口: can0 | CAN FD: arb=1M, data=5M, BRS on  
> 电机节点: M1=NodeID 1, M2=NodeID 2

---

## 1. 协议日志系统

### 1.1 架构

```
motor_hal.c (纯C)                  motor_init.cpp (C++)
     │                                    │
     ├─ PROTO_SEND(fmt, ...)              │
     ├─ PROTO_RECV(node, fmt, ...)        │
     │       │                            │
     │       └── g_proto_log_cb ──────→  motor_hal_set_log_callback()
     │                                       │
     │                                  ECO_INFO_NEW("[PROTO] {}", msg)
     │                                       │
     │                                  spdlog async logger
```

### 1.2 开关控制

**config.json:**
```json
{
    "report": {
        "log_onoff": true
    }
}
```

| `log_onoff` | 效果 |
|-------------|------|
| `false` (默认) | 回调不注册, `g_proto_log_cb == NULL`, 所有 PROTO 宏为空操作, **零开销** |
| `true` | 注册回调, PROTO 日志输出到 ECO_INFO_NEW |

启动日志确认:
```
[CanDispatcher] log_onoff ENABLED    ← 已开启
```

### 1.3 降频策略

| 路径 | 策略 | 频率 | 说明 |
|------|------|------|------|
| MIT 发送 9 行块 | `_mit_skip[node] % 50` | ~20Hz/电机 | 同一次调用要么全打9行要么全不打, 不碎片 |
| MIT_MULTI 发送 | `_multi_skip % 50` | ~20Hz | 全局计数器 |
| 接收帧 (0x6A0/0x690/0x6C0) | `PROTO_RECV_EVERY_N = 50` | ~10Hz/帧类型 | 每节点独立计数 |
| 事件路径 (标定/迁移) | 不降频 | 始终打印 | 低频事件, 无需限流 |

### 1.4 日志洪流问题

**现象:** 开启 `log_onoff` 后下发 MIT 控制, 透传 report 数据变慢。

**根因:** MIT 1KHz × 2电机 × 9行 = 18,000条 ECO_INFO_NEW/秒 → spdlog 异步队列(8192)塞满 → enqueue() 阻塞 RT worker → report 周期拉长 → Ctrl+C 后恢复。

**解决:** 50:1 降频后 ~360条/秒, 远低于队列容量。降频只影响日志频率, 不影响 CAN 帧发送。

---

## 2. MIT 控制帧格式 (KWS CANFD V2)

### 2.1 帧结构

```
CAN ID:  0x110 + node_id (M1=0x111, M2=0x112)
DLC:     9 bytes
Flags:   CAN FD + BRS

Byte[0]: byte0 (enable|brake|mode 位域)
Byte[1]: position[15:8]   (高字节)
Byte[2]: position[7:0]    (低字节)
Byte[3]: velocity[11:4]   (高8位)
Byte[4]: velocity[3:0] << 4 | kp[11:8]   (跨字节拼接)
Byte[5]: kp[7:0]          (低8位)
Byte[6]: kd[11:4]         (高8位)
Byte[7]: kd[3:0] << 4 | torque[11:8]     (跨字节拼接)
Byte[8]: torque[7:0]      (低8位)
```

### 2.2 编码公式

```c
// 对称量 (pos, vel, torque): 映射到 0~FS 全量程
pos_raw = round((pos_rad + pmax) / (2 * pmax) * 65535)
vel_raw = round((vel_rads + vmax) / (2 * vmax) * 4095)
tq_raw  = round((tau_nm + tmax) / (2 * tmax) * 4095)

// 非对称量 (kp, kd): 映射到 0~FS
kp_raw = round(kp / kpmax * 4095)
kd_raw = round(kd / kdmax * 4095)
```

### 2.3 缩放参数 (启动时从 OD 读取)

| OD | 参数 | V2 默认值 | 含义 |
|----|------|-----------|------|
| 0x2542 | pmax (×0.01 rad) | 3.14 | 位置量程 |
| 0x2543 | vmax (×0.01 rad/s) | 3.14 | 速度量程 |
| 0x2544 | kpmax (Nm/rad) | 50 | 刚度量程 |
| 0x2545 | kdmax (Nm·s/rad) | 50 | 阻尼量程 |
| 0x2546 | tmax (Nm) | 20 | 力矩量程 |

---

## 3. 接收帧验证

### 3.1 透传帧一览

| 帧 | CAN ID | DLC | 内容 | 模式条件 |
|----|--------|-----|------|---------|
| 0x300 | 0x300+node | 12/16 | pos/vel/Iq/温度/torque/状态/错误 | 始终发送 |
| 0x680 | 0x680+node | 8 | Hall ADC/力矩/膝关节/开关 | mode=0/2 |
| 0x690 | 0x690+node | 8 | 位置 S32 LE + 速度 S32 LE | mode=1/2 |
| 0x6A0 | 0x6A0+node | 8 | Iq/母线电流/温度/错误码 | mode=1/2 |
| 0x6B0 | 0x6B0+node | 8 | SPI 力矩 S24 LE + valid/error | mode=1/2 |
| **0x6C0** | **0x6C0+node** | **32** | **以上四帧聚合** | **mode=3** |

### 3.2 0x6C0 聚合帧 32 字节布局

```
Byte 0-7   → 原 0x680: Hall ADC0/1/2, DF181力矩CH0, 膝关节AD, 开关, 有效位
Byte 8-15  → 原 0x690: 位置 S32 LE + 速度 S32 LE
Byte 16-23 → 原 0x6A0: Iq S16, 母线电流 S16, 温度 S16, 错误码 U16
Byte 24-31 → 原 0x6B0: SPI 力矩 S24, valid, error
```

### 3.3 当前模式配置

```
[main] sensor passthrough: motor 1 period_div=4 bus=CANFD BRS mode=3 force=1
```

- `mode=3`: 聚合模式, 仅发 0x6C0
- `force=1`: CMD_SPI 力矩传感器
- `period_div=4`: 周期 = 4 × 0.25ms = 1ms = 1KHz (V1.2 基准 0.25ms)

---

## 4. demo_algo 调试命令

### 4.1 MIT 阻抗控制

```bash
# 基本格式
./demo_algo mit <kp> <kd> [pos_deg] [vel_rpm] [torque_Nm]

# 零位置 MIT (无前馈)
./demo_algo mit 30 5

# 指定平衡点 20°
./demo_algo mit 30 5 20 0 0

# 带 5Nm 前馈力矩
./demo_algo mit 30 5 0 0 5

# 纯前馈力矩 (kp=kd=0, 等效力矩控制)
./demo_algo mit 0 0 0 0 10
```

### 4.2 力矩环控制

```bash
# 正弦力矩扫描 (0.05N.m 为单位)
./demo_algo torque_ctrl <val>

# 例: ±10Nm = val=200 (200 × 0.05 = 10Nm)
./demo_algo torque_ctrl 200
```

### 4.3 MIT 多轴广播

```bash
# 双电机一帧 (0x210)
./demo_algo mit_multi <kp> <kd> [pos] [vel] [tq]

./demo_algo mit_multi 30 5 0 0 8
```

### 4.4 标定

```bash
# 力矩零漂标定 (电机失能, 关节机械零位)
./demo_algo calib_torque_zero 1

# 力矩标定 (挂已知负载)
./demo_algo calib_torque 2 17.15
```

### 4.5 周期上报

```bash
# 查看透传数据
./demo_algo report
```

---

## 5. 三层验证方法

### 5.1 启动确认

```bash
# 1. 确认 log_onoff 开启
grep "log_onoff ENABLED" /path/to/log

# 2. 确认 MIT scales 读取成功
grep "MIT scales" /path/to/log
# 期望: pmax=3.14 vmax=3.14 kpmax=50 kdmax=50 tmax=20 (ret=0)

# 3. 确认传感器模式
grep "sensor passthrough" /path/to/log
# 期望: mode=3 force=1 period_div=4
```

### 5.2 MIT 发送帧验证

下发命令后, 日志出现三层打印:

**Layer1 — 编码验证:**
```log
[PROTO] [MIT_SEND] M1 ========== Layer1: phys → raw ==========
[PROTO] [MIT_SEND] M1   phys: pos=0.0° vel=0RPM kp=30.0 kd=5.0 tq=5.00Nm
[PROTO] [MIT_SEND] M1   raw:  pr=32767 vr=2048 kr=2457 dr=410 tr=2559
[PROTO] [MIT_SEND] M1   scales: pmax=3.14 vmax=3.14 kpmax=50 kdmax=50 tmax=20
```

验收点:
- phys 值是否匹配输入参数
- raw 值手工验证公式 (如 pr=32767 对应 pos=0°, 差 1 count 为浮点舍入, 可忽略)
- scales 是否匹配驱动板 OD 读取值

**Layer2 — 字节打包验证:**
```log
[PROTO] [MIT_SEND] M1 ========== Layer2: byte packing (DLC=9) ==========
[PROTO] [MIT_SEND] M1   [0]B0=0x2B | [1-2]pos_H:L=0x7F:0xFF (raw=32767)
[PROTO] [MIT_SEND] M1   [3]vel_H=0x80 | [4]vel_L:kp_H=0x09 (vel=2048 kp=2457)
[PROTO] [MIT_SEND] M1   [5]kp_L=0x99 | [6]kd_H=0x19 | [7]kd_L:tq_H=0xA9 (kd=410)
[PROTO] [MIT_SEND] M1   [8]tq_L=0xFF (tq_raw=2559)
```

验收点:
- vel[11:4] 在 Byte[3], vel[3:0] 在 Byte[4:7:4]
- kp[11:8] 在 Byte[4:3:0], kp[7:0] 在 Byte[5]
- kd[11:4] 在 Byte[6], kd[3:0] 在 Byte[7:7:4]
- tq[11:8] 在 Byte[7:3:0], tq[7:0] 在 Byte[8]

**Layer3 — CAN 帧 hex dump:**

```log
[PROTO] [MIT_SEND] M1 ========== Layer3: CAN frame (CAN FD + BRS) ==========
[PROTO] [MIT_SEND] M1   CAN ID=0x111 DLC=9 FD=1 BRS=1
[PROTO] [MIT_SEND] M1   hex: 2B 7F FF 80 09 99 19 A9 FF
```

验收点: 用 `candump` 抓包, 逐字节对比 Layer3 hex:

```bash
# 终端1: 抓包
candump can0,111:9FFFFFFFF,#FFFFFFFF -tA

# 终端2: 下发命令
./demo_algo mit 30 5
```

### 5.3 接收帧验证

```bash
# 确认 mode=3 生效后只看到 0x6C0 聚合帧
grep "6C0_RECV\|6A0_RECV\|690_RECV" /path/to/log
```

```log
[PROTO] [6C0_RECV] M1 hall=(2048,2047,2050) force=512 knee=2045 key=0 valid=1 | pos=32767 cnt vel=100 RPM | Iq=2.5A bus=3.10A temp=35.2°C err=0x0000 | spi_raw=12345 spi_v=1 spi_e=0
```

验收点:
- 模式切换后不再有 0x6A0/0x690 (独立帧停止)
- Hall ADC 值在正常范围 (2048 附近)
- 静止时 Iq≈0, 母线电流小
- spi_valid=1 表示 SPI 力矩传感器通信正常

---

## 6. 常见问题排查

### 6.1 log_onoff 开启后日志不出现

检查:
1. config.json 中 `"log_onoff": true` 是否正确
2. 启动日志是否有 `log_onoff ENABLED`
3. 如为 false, `g_proto_log_cb == NULL`, 所有 PROTO 宏为空操作

### 6.2 下发 MIT 后日志不打印

- MIT 降频 50:1, 需要等待 ~50 次调用 (约 50ms) 才出现
- 检查 `raw` 是否全 0 (全 0 会被 `motor_hal_mit_control` 拒绝)
- 检查电机是否在线: `./demo_algo stat`

### 6.3 接收帧数据全零或异常

- 确认传感器透传已配置: `mode=3 force=1` 出现在启动日志
- 00xC0 帧的 DLC 是否 32 (CAN FD 驱动问题会导致后 24 字节为 0, 见 doc/canfd_debug.md)
- 接收降频 50:1, 部分帧不打印是正常的

### 6.4 Ctrl+C 后电机没有失能

- **MIT / torque_ctrl 的 bug**: `motor_hal_mit_control()` 不检查 `m->enabled`,
  estop 后残余 MIT 命令在同周期内 re-enable 电机
- speed 模式不受影响 (usleep(5000) 让 mailbox 有足够时间排空)
- 解决办法: 发一个 `./demo_algo disable 1` 手动失能, 或切到 speed 模式再 Ctrl+C

### 6.5 candump 对比不一致

逐字节检查公式:
```python
# pos=0°, tmax=20
pos_raw = round((0 + 3.14) / (2 * 3.14) * 65535)  # = 32767 或 32768
tq_raw  = round((5 + 20) / (2 * 20) * 4095)        # = 2559

# 字节打包
byte1 = (pos_raw >> 8) & 0xFF   # pos_H
byte2 = pos_raw & 0xFF          # pos_L
byte7 = ((kd_raw & 0x0F) << 4) | ((tq_raw >> 8) & 0x0F)  # kd_L:tq_H
byte8 = tq_raw & 0xFF           # tq_L
```

---

## 7. 控制律参考

### 7.1 MIT 模式

```
τ_output = Kp × (θ_ref - θ_actual) + Kd × (ω_ref - ω_actual) + τ_ff
```

| 命令示例 | 预期现象 |
|---------|---------|
| `mit 30 5 0 0 5` | 电机受 5Nm 前馈推动, 平衡在 ~9.6° (30×(0-θ)= -5) |
| `mit 30 5 20 0 0` | 电机停在 20°, 手推开松手弹回 |
| `mit 0 0 0 0 10` | 纯前馈力矩, 电机匀速旋转 (无刚度约束) |
| `mit 50 10 0 0 0` | 高刚度零位置, 手推阻力感明显 |

### 7.2 力矩环模式

```
τ_output = target_torque (0x100 mode=6)
```

- 力矩单位: 0.05N.m (target1=200 → 10Nm)
- 与 MIT 的区别: 纯力矩控制, 无刚度/阻尼项

---

## 8. 相关文件

| 文件 | 说明 |
|------|------|
| `motor_hal/src/motor_hal.c` | 协议日志桥 + PROTO_SEND/PROTO_RECV 宏 |
| `motor_hal/inc/motor_hal.h` | 回调类型声明 |
| `motor_hal/src/canopen_frames.c` | MIT 帧构造 `canopen_mit_pdo_build_u8()` |
| `stark_periph_node/src/motor/motor_init.cpp` | 回调注册, log_onoff 配置解析 |
| `stark_periph_node/src/test/demo_algo.c` | 调试命令入口 |
| `doc/cansend_reference_v2.md` | CAN FD cansend 命令参考 |
| `doc/canfd_debug.md` | CAN FD 驱动调试 |
