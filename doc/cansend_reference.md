# demo_algo 控制指令 → cansend 参考手册

> 版本: v1.0 | 日期: 2026-07-01 | 基于 motor_hal_types.h / canopen_frames.c / motor_rt_worker.cpp

---

## 1. CAN 帧基础参数

| 参数 | M1 (右髋) | M2 (左髋) |
|------|-----------|-----------|
| node_id | 1 | 2 |
| SDO COB-ID | `0x601` | `0x602` |
| 单轴 PDO COB-ID | `0x101` | `0x102` |
| MIT PDO COB-ID | `0x111` | `0x112` |
| 反馈帧 COB-ID | `0x301` | `0x302` |
| 多轴广播 COB-ID | **`0x200`** | — |
| NMT COB-ID | `0x000` | — |
| CAN 接口 | `can0` | — |

> **重要**: PDO 帧均为 **CANFD** (DLC > 8)，使用前需确保 CAN 接口已配置为 CANFD 模式。

```bash
# 初始化 CANFD
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

---

## 2. SDO 帧格式 (CAN 2.0, COB=0x600+node, DLC=8)

```
Byte0:     SDO 命令码 (0x2F=1B写, 0x2B=2B写, 0x23=4B写, 0x40=读)
Byte1-2:   Object Index 小端 (如 0x6071 → 71 60)
Byte3:     SubIndex
Byte4-7:   数据 小端
```

---

## 3. Byte0 — PDO 控制字节编码

```
Bit7 (0x80): enable      1=使能 PDO 响应
Bit6 (0x40): bus_on      1=母线接通/松抱闸
Bit5 (0x20): clear_err   1=清除错误 (脉冲, 单帧后自动清0)
Bit4-1:      mode << 1   控制模式编码
Bit0 (0x01): 保留
```

### 常用 Byte0 值 (enable=1 + bus_on=1)

| 模式 | 枚举值 | mode<<1 | Byte0 完整值 |
|------|--------|---------|-------------|
| CURRENT (电流) | 5 | `0x0A` | **`0xCA`** |
| CSP (循环位置) | 3 | `0x06` | **`0xC6`** |
| CSV (循环速度) | 4 | `0x08` | **`0xC8`** |
| PP (轮廓位置) | 1 | `0x02` | **`0xC2`** |
| PV (轮廓速度) | 2 | `0x04` | **`0xC4`** |
| MIT (阻抗) | 6 | `0x0C` | **`0xCC`** |

### 特殊 Byte0 值

| 用途 | Byte0 | 含义 |
|------|-------|------|
| 清故障 | `0xE0` | enable=1 + bus_on=1 + clear_err=1 + CURRENT |
| 急停 | `0x00` | enable=0 + bus_on=0 |
| 失能 | `0x40` | enable=0 + bus_on=1 |

---

## 4. 多轴广播帧 (0x200, CANFD DLC=64)

所有 PDO 连续控制 (`torque/speed/pos/pp/pv/multi`) 走同一帧格式。

### 帧布局 (大端)

```
Byte[0-6]:   M1 数据块
Byte[7-13]:  M2 数据块
...
Byte[56-63]: ID 映射表 (每字节存一个电机 node_id)
```

### 单电机数据块 (7 字节)

```
[0]:     Byte0 (flags + mode)
[1-2]:   target1   int16 大端
[3-4]:   target2   uint16 大端
[5-6]:   feedforward  int16 大端
```

---

## 5. 控制命令 → cansend 对照表

---

### 5.1 `torque <mA>` — PDO 电流正弦波

模式: CURRENT, 走 0x200 CANFD。

```bash
# ===== 仅 M1 =====
# M1=500mA (0x01F4)
cansend can0 200##CA01F4000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001

# ===== M1 + M2 (同值) =====
# M1=500mA, M2=500mA
cansend can0 200##CA01F400000000CA01F400000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000102

# ===== M1 + M2 (不同值) =====
# M1=500mA, M2=300mA (0x012C)
cansend can0 200##CA01F400000000CA012C00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000102
```

| 字段 | M1 值 | 说明 |
|------|-------|------|
| Byte0 | `0xCA` | enable + bus_on + CURRENT |
| target1 | `0x01F4` (500) | 电流 mA, int16 |
| target2 | `0x0000` | 未使用 |
| feedforward | `0x0000` | 未使用 |

---

### 5.2 `speed/csv <rpm>` — PDO 速度梯形波

模式: CSV, 走 0x200 CANFD。

```bash
# M1=10RPM (0x000A)
cansend can0 200##C8000A0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001

# M1=-10RPM (0xFFF6)
cansend can0 200##C8FFF60000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001
```

| 字段 | 值 | 说明 |
|------|-----|------|
| Byte0 | `0xC8` | CSV |
| target1 | RPM (int16) | 目标速度 |

---

### 5.3 `pos/csp <deg>` — PDO 位置方波 (CSP)

模式: CSP, 走 0x200 CANFD。

```
counts = deg × 65536 / 360   (int16, 范围 ±32767)
```

```bash
# M1=15° → counts = 15*65536/360 ≈ 2731 = 0x0AAB
cansend can0 200##C60AAB0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001

# M1=-15° → counts = -2731 = 0xF555
cansend can0 200##C6F5550000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001
```

| 字段 | 值 | 说明 |
|------|-----|------|
| Byte0 | `0xC6` | CSP |
| target1 | counts | 角度编码值 int16 |

---

### 5.4 `pp <deg> [acc] [vel]` — PDO 轮廓位置 PP

模式: PP, 走 0x200 CANFD。

> 实际 CAN 帧: target1=counts, target2=acc/100 (uint16), feedforward=vel/100 (int16)

```bash
# M1=15°, acc=500, vel=10
# counts=2731=0x0AAB, target2=5=0x0005, ff=0=0x0000
cansend can0 200##C20AAB0005000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001
```

| 字段 | 值 | 说明 |
|------|-----|------|
| Byte0 | `0xC2` | PP |
| target1 | counts | 目标位置 int16 |
| target2 | accel/100 | uint16 |
| feedforward | vel/100 | int16 |

---

### 5.5 `pv <rpm> [acc]` — PDO 轮廓速度 PV

模式: PV, 走 0x200 CANFD。

```bash
# M1=30RPM, acc=1000 → target2=10=0x000A
cansend can0 200##C4001E000A0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001
```

| 字段 | 值 | 说明 |
|------|-----|------|
| Byte0 | `0xC4` | PV |
| target1 | RPM (int16) | 目标速度 |
| target2 | acc/100 | uint16 |

---

### 5.6 `multi <ma1> <ma2>` — PDO 多轴恒电流

同 torque，但 M1/M2 可以不同值，走 0x200 CANFD。

```bash
# M1=500mA, M2=300mA
cansend can0 200##CA01F400000000CA012C00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000102
```

---

### 5.7 `mit <kp> <kd>` — MIT 阻抗控制

**独立 COB-ID: `0x110+node`**, CANFD DLC=9。

MIT PDO 帧格式 (bit-packed):

```
[0]: Byte0 (0xCC = enable + bus_on + MIT)
[1-2]: position     uint16 大端   (0°→0x8000, 180°→0xFFFF, -180°→0x0000)
[3]:   velocity[11:4]
[4]:   velocity[3:0] << 4 | kp[11:8]
[5]:   kp[7:0]
[6]:   kd[11:4]
[7]:   kd[3:0] << 4 | torque[11:8]
[8]:   torque[7:0]
```

编码规则:
- position = uint16, 0° 映射到 0x8000
- velocity = (实际值 × 16) 的 12bit 有符号
- kp = uint16(kp × 100)
- kd = uint16(kd × 100)
- torque = int16(N·m × 1000)

```bash
# M1 MIT: kp=100, kd=10, pos=0, vel=0, torque=0
# position=0x8000, velocity=0, kp=10000=0x2710, kd=1000=0x03E8
# Byte[0]=0xCC
# Byte[1]=0x80, Byte[2]=0x00
# Byte[3]=0x00 (vel>>4)
# Byte[4]=0x02 ((vel&0xF)<<4 | (kp>>8)&0xF) = 0x02
# Byte[5]=0x10 (kp&0xFF)
# Byte[6]=0x03 (kd>>4)
# Byte[7]=0xE8 ((kd&0xF)<<4 | (torque>>8)&0xF) → 0x80? 
#   kd&0xF=0xE8&0xF=8, 8<<4=0x80, torque>>8=0, → 0x80
# Byte[8]=0x00 (torque&0xFF)
cansend can0 111##CC800000021003E88000
```

---

### 5.8 `sdo cur <id> <mA>` — SDO 单帧电流

写 `0x6071:00` (Target Torque), 2 字节。

```bash
# M1=500mA (0x01F4)
cansend can0 601#2B.71.60.00.F4.01.00.00

# M2=300mA (0x012C)
cansend can0 602#2B.71.60.00.2C.01.00.00

# M1=-500mA (0xFE0C)
cansend can0 601#2B.71.60.00.0C.FE.00.00
```

| 字节 | 值 | 含义 |
|------|-----|------|
| [0] | `0x2B` | SDO download 2 bytes |
| [1-2] | `71 60` | Index 0x6071 小端 |
| [3] | `00` | SubIndex 0 |
| [4-5] | mA 小端 | int16 电流值 |
| [6-7] | `00 00` | 未使用 |

---

### 5.9 `sdo pos <id> <deg>` — SDO 单帧位置 (PP)

Step 1: 写 `0x607A:00` (Target Position), 4 字节
Step 2: 写 `0x6040:00 = 0x004F` 触发绝对运动

```
counts = deg × 65536 / 360
```

```bash
# ===== M1=30° =====
# Step 1: counts = 30*65536/360 = 5461 = 0x1555
cansend can0 601#23.7A.60.00.55.15.00.00

# Step 2: 触发运动 (CW=0x004F: new-set-point + absolute + enable)
cansend can0 601#2B.40.60.00.4F.00.00.00

# ===== M1=-30° =====
# counts = -5461 = 0xEAAB
cansend can0 601#23.7A.60.00.AB.EA.FF.FF
cansend can0 601#2B.40.60.00.4F.00.00.00
```

| 步骤 | Index:Sub | 数据 | 说明 |
|------|-----------|------|------|
| Step 1 | `0x607A:00` | counts (int32 小端) | 目标位置 |
| Step 2 | `0x6040:00` | `0x004F` (2B 小端) | 触发绝对运动 |

> 也可用 `0x603F` + `0x005F` (相对运动, change-set-immediately)

---

### 5.10 `sdo vel <id> <rpm>` — SDO 单帧速度 (PV)

写 `0x60FF:00` (Target Velocity), 4 字节 int32。

```bash
# M1=10RPM
cansend can0 601#23.FF.60.00.0A.00.00.00

# M1=-10RPM
cansend can0 601#23.FF.60.00.F6.FF.FF.FF
```

---

### 5.11 `enable <id>` — 使能

通过 SDO 写 Controlword (`0x6040:00`) 状态机序列:

```bash
# M1 使能
cansend can0 601#2B.40.60.00.06.00.00.00   # CW=0x0006 Shutdown
cansend can0 601#2B.40.60.00.07.00.00.00   # CW=0x0007 Switch On
cansend can0 601#2B.40.60.00.0F.00.00.00   # CW=0x000F Enable Operation

# M2 使能
cansend can0 602#2B.40.60.00.06.00.00.00
cansend can0 602#2B.40.60.00.07.00.00.00
cansend can0 602#2B.40.60.00.0F.00.00.00
```

---

### 5.12 `disable <id>` — 失能

```bash
# M1 失能 (CW=0x0006 Shutdown)
cansend can0 601#2B.40.60.00.06.00.00.00

# 等价: PDO 多轴广播, enable=0 + bus_on=1 (仅关闭使能, 保持母线)
cansend can0 200##4000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001
```

---

### 5.13 `estop <id>` — 急停

通过 PDO 多轴广播: **enable=0 + bus_on=0 + CURRENT + 零目标**。

```bash
# M1 急停
cansend can0 200##0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001

# M1 + M2 急停
cansend can0 200##000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000102
```

| 字段 | 值 | 说明 |
|------|-----|------|
| Byte0 | `0x00` | enable=0, bus=0, mode=CURRENT |
| target1 | `0x0000` | 零电流 |
| target2 | `0x0000` | |
| feedforward | `0x0000` | |

---

### 5.14 `clearf <id>` — 清故障

通过单轴 PDO (`0x100+node`): **enable=1 + bus_on=1 + clear_err=1 + CURRENT + 零目标**。

```bash
# M1 清故障 (Byte0=0xE0)
cansend can0 101##E0000000000000

# M2 清故障
cansend can0 102##E0000000000000
```

| 字段 | 值 | 说明 |
|------|-----|------|
| Byte0 | `0xE0` | enable + bus_on + clear_err + CURRENT |
| target1~3 | `0` | 控制值清零 |

> clear_err (bit5) 是脉冲信号，驱动板下一帧自动清 0。

---

### 5.15 `calib` — 校准

通过 SDO 写 Controlword 校准时序。核心帧:

```bash
# M1 校准入口
cansend can0 601#2B.40.60.00.06.00.00.00   # CW=0x0006 Shutdown
cansend can0 601#2B.40.60.00.07.00.00.00   # CW=0x0007 Switch On
cansend can0 601#2B.40.60.00.0F.00.00.00   # CW=0x000F Enable Operation
```

完整校准流程由 `motor_calib.c` 编排 (包含校准模式切换、超时检测、结果判断等)，不建议手工 `cansend` 模拟。

---

### 5.16 `led <id> <mask> <mode> [r] [g] [b]` — LED 灯控

SDO 写 `0x5503:06`, 4 字节。

```
value = mask | (mode << 8) | (R << 16) | (G << 24)
B 值写入下一个 32bit (暂不暴露给 cansend 单帧)
```

```bash
# M1: 全亮常亮红色 (mask=0xF0, mode=0, R=255, G=0, B=0)
cansend can0 601#23.03.55.00.F0.00.FF.00

# M1: LED1 蓝色闪烁 (mask=0x10, mode=1, R=0, G=0, B=255)
cansend can0 601#23.03.55.00.10.01.00.00

# M1: 全灭
cansend can0 601#23.03.55.00.00.00.00.00
```

| 字节[4] | 含义 |
|---------|------|
| `0x10` | LED1 |
| `0x20` | LED2 |
| `0x40` | LED3 |
| `0x80` | LED4 |
| `0xF0` | 全部 |

| 字节[5] | mode |
|---------|------|
| `0x00` | 常亮 |
| `0x01` | 闪烁 |
| `0x02` | 呼吸 |
| `0x03` | 流水 |

---

### 5.17 `btn` / `stat` / `report` — 只读命令

这三个命令**只从共享内存读取反馈数据，不发送任何 CAN 帧**，无对应 `cansend`。

---

## 6. 快速参考总表

| demo_algo 命令 | CAN 类型 | COB-ID | DLC | Byte0 | 关键数据 |
|---------------|----------|--------|-----|-------|---------|
| `torque <mA>` | CANFD | `0x200` | 64 | `0xCA` | target1=mA |
| `speed <rpm>` | CANFD | `0x200` | 64 | `0xC8` | target1=RPM |
| `pos <deg>` | CANFD | `0x200` | 64 | `0xC6` | target1=counts |
| `pp <deg> [a] [v]` | CANFD | `0x200` | 64 | `0xC2` | t1=counts, t2=acc/100, ff=vel/100 |
| `pv <rpm> [a]` | CANFD | `0x200` | 64 | `0xC4` | t1=RPM, t2=acc/100 |
| `mit <kp> <kd>` | CANFD | `0x110+id` | 9 | `0xCC` | bit-packed 5 段 |
| `multi <ma1> <ma2>` | CANFD | `0x200` | 64 | `0xCA` | M1/M2 各自 target1=mA |
| `sdo cur <id> <mA>` | CAN2.0 | `0x600+id` | 8 | — | 写 `0x6071:00`, 2B |
| `sdo pos <id> <deg>` | CAN2.0 | `0x600+id` | 8 | — | 写 `0x607A:00` + `0x6040:00=0x004F` |
| `sdo vel <id> <rpm>` | CAN2.0 | `0x600+id` | 8 | — | 写 `0x60FF:00`, 4B |
| `enable <id>` | CAN2.0 | `0x600+id` | 8 | — | CW: `0x06`→`0x07`→`0x0F` |
| `disable <id>` | CAN2.0 | `0x600+id` | 8 | — | CW=`0x06` |
| `estop <id>` | CANFD | `0x200` | 64 | `0x00` | enable=0, bus=0 |
| `clearf <id>` | CANFD | `0x100+id` | 7 | `0xE0` | clear_err 脉冲 |
| `calib` | CAN2.0 | `0x600+id` | 8 | — | CW 状态机 |
| `led <id> ...` | CAN2.0 | `0x600+id` | 8 | — | 写 `0x5503:06`, 4B |
| `btn` | — | — | — | — | 只读 SHM |
| `stat` | — | — | — | — | 只读 SHM |
| `report` | — | — | — | — | 只读 SHM |

---

## 7. 常用 OD (Object Dictionary) 速查

| Index:Sub | 名称 | 类型 | 说明 |
|-----------|------|------|------|
| `0x6040:00` | Controlword | uint16 | 状态机控制 |
| `0x6041:00` | Statusword | uint16 | 状态反馈 |
| `0x6060:00` | Modes of Operation | int8 | 控制模式 |
| `0x6061:00` | Modes of Operation Display | int8 | 当前模式 |
| `0x6064:00` | Position Actual Value | int32 | 实际位置 counts |
| `0x6071:00` | Target Torque | int16 | 目标电流 mA |
| `0x607A:00` | Target Position | int32 | 目标位置 counts |
| `0x607D:00` | Software Position Limit | — | 软限位 |
| `0x6081:00` | Profile Velocity | uint32 | 轨迹速度 |
| `0x6083:00` | Profile Acceleration | uint32 | 加速度 |
| `0x6084:00` | Profile Deceleration | uint32 | 减速度 |
| `0x60FF:00` | Target Velocity | int32 | 目标速度 RPM |
| `0x5503:06` | LED Control | uint32 | LED 灯控 |
| `0x603F:00` | Error Code | uint16 | 故障码 |

---

## 8. Controlword 常用值

| 值 | 命令 | 说明 |
|----|------|------|
| `0x0006` | Shutdown | 失能 |
| `0x0007` | Switch On | 上电 |
| `0x000F` | Enable Operation | 使能 |
| `0x000B` | Quick Stop | 快停 |
| `0x004F` | Enable + New Set-Point + Absolute | PP 触发绝对运动 |
| `0x005F` | Enable + New Set-Point + Relative | PP 触发相对运动 |
| `0x010F` | Halt | 暂停当前运动 |

---

## 9. SDO 命令码

| 命令码 | 含义 |
|--------|------|
| `0x23` | Download 4 bytes (写 4 字节) |
| `0x2B` | Download 2 bytes (写 2 字节) |
| `0x2F` | Download 1 byte (写 1 字节) |
| `0x40` | Upload request (读请求) |
| `0x43` | Upload response 4 bytes |
| `0x4B` | Upload response 2 bytes |
| `0x4F` | Upload response 1 byte |
| `0x80` | Abort (错误) |
