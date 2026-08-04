# CAN/CAN FD cansend 指令参考手册

> 版本: V2.1 | 日期: 2026-08-03  
> CAN 接口: can0 | CAN FD: bitrate 1M, dbitrate 5M, BRS on  
> 电机节点: M1=NodeID 1 (0x601/0x581), M2=NodeID 2 (0x602/0x582)

---

## 1. 接口初始化

```bash
# 关闭接口
sudo ip link set can0 down

# CAN FD 1M/5M + BRS, sample-point 0.8/0.75
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 \
    dbitrate 5000000 dsample-point 0.75 fd on

# 启动
sudo ip link set can0 up

# 验证
ip -d link show can0
```

---

## 2. SDO 指令

### 2.1 SDO 帧格式

```
请求: 0x600 + NodeID (M1=0x601, M2=0x602)
响应: 0x580 + NodeID (M1=0x581, M2=0x582)
8 字节

Byte0:    命令码 (0x23=4B写, 0x2B=2B写, 0x2F=1B写, 0x40=读)
Byte1-2:  Index 小端
Byte3:    SubIndex
Byte4-7:  数据 小端
```

### 2.2 读操作

```bash
# 读 0x6041:00 Statusword
cansend can0 601#40.41.60.00.00.00.00.00

# 读 0x6064:00 Position actual value
cansend can0 601#40.64.60.00.00.00.00.00

# 读 0x606C:00 Velocity actual value
cansend can0 601#40.6C.60.00.00.00.00.00

# 读 0x6077:00 Torque actual value
cansend can0 601#40.77.60.00.00.00.00.00

# 读 0x6078:00 Current actual value
cansend can0 601#40.78.60.00.00.00.00.00

# 读 0x6079:00 DC bus voltage
cansend can0 601#40.79.60.00.00.00.00.00

# 读 0x603F:00 Error code
cansend can0 601#40.3F.60.00.00.00.00.00

# 读 0x2661:00 Bus current
cansend can0 601#40.61.26.00.00.00.00.00

# 读 0x2663:00 Motor coil temperature
cansend can0 601#40.63.26.00.00.00.00.00

# 读 0x100A:00 Software version
cansend can0 601#40.0A.10.00.00.00.00.00

# 读 0x3100:05 主线标识
cansend can0 601#40.00.31.05.00.00.00.00

# 读 0x2540:00 CAN FD 数据段波特率
cansend can0 601#40.40.25.00.00.00.00.00
```

### 2.3 CiA 402 控制

```bash
# 使能序列 (0x06→0x07→0x0F)
cansend can0 601#2B.40.60.00.06.00.00.00    # Shutdown
cansend can0 601#2B.40.60.00.07.00.00.00    # Switch On
cansend can0 601#2B.40.60.00.0F.00.00.00    # Enable Operation

# 失能
cansend can0 601#2B.40.60.00.06.00.00.00    # Shutdown

# 清故障 (Fault Reset)
cansend can0 601#2B.40.60.00.80.00.00.00    # CW=0x0080
```

### 2.4 控制命令 (SDO 写)

```bash
# 电流控制: 写 0x6071:00 = 500mA
cansend can0 601#2B.71.60.00.F4.01.00.00    # M1=500mA
cansend can0 601#2B.71.60.00.0C.FE.00.00    # M1=-500mA

# 轮廓位置 PP: 写 0x607A:00 位置 + 0x6040 触发
# M1=30°: counts = 30*65536/360 = 5461 = 0x1555
cansend can0 601#23.7A.60.00.55.15.00.00    # 目标位置
cansend can0 601#2B.40.60.00.4F.00.00.00    # 触发绝对运动

# M1=-30°: counts = -5461 = 0xEAAB
cansend can0 601#23.7A.60.00.AB.EA.FF.FF
cansend can0 601#2B.40.60.00.4F.00.00.00

# 轮廓速度 PV: 写 0x60FF:00
cansend can0 601#23.FF.60.00.0A.00.00.00    # M1=10RPM
cansend can0 601#23.FF.60.00.F6.FF.FF.FF    # M1=-10RPM

# 力矩控制: 写 0x6071:00
cansend can0 601#2B.71.60.00.F4.01.00.00    # M1=500mA 目标电流
```

### 2.5 零位 / 力矩标定

```bash
# 机械零位: 写 0x2531:00 = 1 (opcode=1)
cansend can0 601#23.31.25.00.01.00.00.00    # M1 设零位

# 力矩传感器标定: 写 0x2531:00, opcode=2 | (mNm << 8)
# 理论力矩 = 0 Nm: packed = 0x00000002
cansend can0 601#23.31.25.00.02.00.00.00    # M1 零漂标定
# 理论力矩 = 10 Nm = 10000 mNm: packed = 0x00271002
cansend can0 601#23.31.25.00.02.10.27.00    # M1=10Nm
# 理论力矩 = -10 Nm: packed = 0xFFD8F002
cansend can0 601#23.31.25.00.02.F0.D8.FF    # M1=-10Nm
# 理论力矩 = 50 Nm: packed = 0x00C35002
cansend can0 601#23.31.25.00.02.50.C3.00    # M1=50Nm
```

### 2.6 参数配置

```bash
# 节点 ID: 写 0x2530:00 (修改后需重启)
cansend can0 601#23.30.25.00.01.00.00.00    # 设为 1

# CAN FD 数据段波特率: 写 0x2540:00
# 1=5M, 2=4M, 3=2M, 4=1M
cansend can0 601#23.40.25.00.01.00.00.00    # 5Mbps

# PID 参数
cansend can0 601#23.32.25.00.64.00.00.00    # 电流环 P=100
cansend can0 601#23.33.25.00.32.00.00.00    # 电流环 I=50
cansend can0 601#23.34.25.00.00.01.00.00    # 速度环 P=256
cansend can0 601#23.36.25.00.64.00.00.00    # 位置环 P=100

# 限位
# 正限位 90°: counts = 90*65536/360 = 16384 = 0x4000
cansend can0 601#23.7D.60.02.00.40.00.00    # sub=0x02, 正限位=90°
# 负限位 -90°: counts = -16384 = 0xFFFFC000
cansend can0 601#23.7D.60.01.00.C0.FF.FF    # sub=0x01, 负限位=-90°

# 最大转速: 写 0x6080:00
cansend can0 601#23.80.60.00.D0.07.00.00    # 2000RPM

# 最大电流: 写 0x2538:00
cansend can0 601#23.38.25.00.E8.03.00.00    # 1000mA

# 看门狗/心跳: 写 0x1017:00
cansend can0 601#2B.17.10.00.00.00.00.00    # 关闭

# 保存到 Flash: 写 0x1010:01 = 1
cansend can0 601#23.10.10.01.01.00.00.00

# 保存 Flash: 写 0x2539:00 = 1
cansend can0 601#23.39.25.00.01.00.00.00
```

### 2.7 MIT 缩放参数 (0x2542~0x2546)

```bash
# 读取缩放
cansend can0 601#40.42.25.00.00.00.00.00    # 0x2542 Pmax
cansend can0 601#40.43.25.00.00.00.00.00    # 0x2543 Vmax
cansend can0 601#40.44.25.00.00.00.00.00    # 0x2544 Kpmax
cansend can0 601#40.45.25.00.00.00.00.00    # 0x2545 Kdmax
cansend can0 601#40.46.25.00.00.00.00.00    # 0x2546 Tmax

# 工厂默认响应:
# 0x2542=314 → 43 42 25 00 3A 01 00 00
# 0x2543=314 → 43 43 25 00 3A 01 00 00
# 0x2544=50  → 43 44 25 00 32 00 00 00
# 0x2545=50  → 43 45 25 00 32 00 00 00
# 0x2546=20  → 43 46 25 00 14 00 00 00

# 写入缩放 (失能状态下)
cansend can0 601#23.44.25.00.32.00.00.00    # Kpmax=50
cansend can0 601#23.46.25.00.14.00.00.00    # Tmax=20

# MIT 缩放迁移: 写 Tmax=20 + 保存
cansend can0 601#23.46.25.00.14.00.00.00    # 0x2546=20
cansend can0 601#23.39.25.00.01.00.00.00    # 保存 Flash
```

### 2.8 传感器透传配置

```bash
# OD 0x5503:04, 32bit: period_div | bus_fmt<<16 | mode<<18 | force<<20

# 关闭
cansend can0 601#23.03.55.04.00.00.00.00

# mode=2 (全部), period_div=4 (1ms), FD+BRS, CMD_SPI
cansend can0 601#23.03.55.04.04.28.00.00

# mode=3 (聚合), period_div=4 (1ms), FD+BRS, CMD_SPI
cansend can0 601#23.03.55.04.04.30.10.00

# LED 控制: 0x5503:06
# byte4=enable_mask|mode, byte5=R, byte6=G, byte7=B
cansend can0 601#23.03.55.00.F0.00.FF.00      # 全亮红色常亮
cansend can0 601#23.03.55.00.10.01.00.FF      # LED1 蓝色闪烁
cansend can0 601#23.03.55.00.00.00.00.00      # 全灭
```

---

## 3. PDO 指令

### 3.1 单轴控制 (0x100 + NodeID)

帧格式: CAN FD, DLC=7

```
Byte0:    控制字 (bit7=使能, bit6=抱闸, bit5=清错, bit4:1=mode)
[1-2]:    target1  int16 大端
[3-4]:    target2  uint16 大端
[5-6]:    feedforward int16 大端
```

**控制字:**

| 模式 | mode<<1 | byte0(en+brake+mode) |
|------|---------|---------------------|
| CURRENT | 0x0A | 0xCA |
| CSP | 0x06 | 0xC6 |
| CSV | 0x08 | 0xC8 |
| PP | 0x02 | 0xC2 |
| PV | 0x04 | 0xC4 |
| TORQUE | 0x0C | 0xCC |

```bash
# 电流 500mA
cansend can0 101##CA01F40000000000

# CSP 15°: counts = 15*65536/360 = 2731 = 0x0AAB
cansend can0 101##C60AAB0000000000

# CSV 10RPM
cansend can0 101##C8000A0000000000

# PP 15°, acc=2000, vel=10
cansend can0 101##C20AAB07D0000A00

# 力矩控制 200 × 0.05N.m = 10N.m
cansend can0 101##CC00C80000000000

# 清故障
cansend can0 101##E0000000000000

# 失能
cansend can0 101##40000000000000

# 急停 (enable=0, bus=0)
cansend can0 101##00000000000000
```

### 3.2 MIT 单轴控制 (0x110 + NodeID)

帧格式: CAN FD, DLC=9 → 12 字节

```
Byte0:    控制字 (推荐 0x8C)
[1-8]:    MIT 8 字节 payload
[9-11]:   保留 (00 00 00)
```

MIT payload 8 字节:

```
[1-2]: position_raw  uint16 大端
[3]:   velocity_raw[11:4]
[4]:   velocity_raw[3:0] << 4 | kp_raw[11:8]
[5]:   kp_raw[7:0]
[6]:   kd_raw[11:4]
[7]:   kd_raw[3:0] << 4 | torque_raw[11:8]
[8]:   torque_raw[7:0]
```

编码公式 (默认量程: Pmax=3.14, Vmax=3.14, Kpmax=50, Kdmax=50, Tmax=20):

```
pos_raw  = round((q_des + 3.14) / 6.28 × 65535)
vel_raw  = round((dq_des + 3.14) / 6.28 × 4095)
kp_raw   = round(Kp / 50 × 4095)
kd_raw   = round(Kd / 50 × 4095)
tq_raw   = round((tau_ff + 20) / 40 × 4095)
```

零值 raw: pos=0x8000, vel=0x800, torque=0x800, Kp=0x000, Kd=0x000

```bash
# 零命令 (Kp=0, Kd=0, 位置/速度/力矩=0)
cansend can0 111##8C.8000.8000.0000.0800.000000

# Kp=30, Kd=5, 零目标位置
# pos_raw=0x8000, vel_raw=0x800, kp_raw=30/50×4095=2457=0x0999
# kd_raw=5/50×4095=409=0x199, tq_raw=0x800
cansend can0 111##8C.8000.8009.9901.9908.000000

# 纯力矩 10Nm
# tq_raw = (10+20)/40×4095 = 3071 = 0x0BFF
cansend can0 111##8C.8000.8000.0000.0BFF.000000

# 失能
cansend can0 111##0C.8000.8000.0000.0800.000000
```

### 3.3 多轴控制 (0x200)

帧格式: CAN FD, DLC=64

8 个槽位，每槽位 7 字节 (同单轴 PDO)。ID 映射在 Byte56-63。

```bash
# M1=500mA
cansend can0 200##CA01F4000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001

# M1=500mA, M2=300mA
cansend can0 200##CA01F400000000CA012C00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000102

# 急停双电机
cansend can0 200##000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000102
```

### 3.4 MIT 多轴控制 (0x210)

帧格式: CAN FD, DLC=64

6 个槽位，每槽位 9 字节 (ctrl + 8B MIT payload)。ID 映射在 Byte56-61。未使用槽位填 0。

```bash
# M1=Kp30+ Kd5, M2=Kp30+ Kd5 (两电机零位)
# 槽位0 M1: 8C 80 00 80 09 99 01 99 08 00
# 槽位1 M2: 8C 80 00 80 09 99 01 99 08 00
# ID映射: Byte56=01, Byte57=02
cansend can0 210##8C.8000.8009.9901.9908.008C.8000.8009.9901.9908.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0000.0102.00000000
```

### 3.5 SYNC

```bash
# 触发 0x300 反馈帧
cansend can0 080#
```

---

## 4. NMT

```bash
# NMT 命令: COB-ID 0x000, Byte0=cmd, Byte1=NodeID 或 0x00(广播)

cansend can0 000#01.00                      # Start all nodes
cansend can0 000#02.00                      # Stop all nodes
cansend can0 000#80.00                      # Reset all nodes
cansend can0 000#82.00                      # Reset Communication all nodes
```

---

## 5. 快速速查

```
  初始化:    ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on && ip link set can0 up

  SDO 读:    cansend can0 601#40.[idx_lo].[idx_hi].[sub].00.00.00.00
  SDO 写4B:  cansend can0 601#23.[idx_lo].[idx_hi].[sub].[d0].[d1].[d2].[d3]
  SDO 写2B:  cansend can0 601#2B.[idx_lo].[idx_hi].[sub].[d0].[d1].00.00

  使能:      cansend can0 601#2B.40.60.00.06.00.00.00 (→07→0F)
  失能:      cansend can0 601#2B.40.60.00.06.00.00.00

  PDO 电流:  cansend can0 101##CA[ma_hi][ma_lo]0000000000
  PDO CSP:   cansend can0 101##C6[cnt_hi][cnt_lo]0000000000
  MIT:       cansend can0 111##8C[pos][vel][kp_kd][kp_lo][kd_tq][tq_lo]000000

  透传:      cansend can0 601#23.03.55.04.04.28.00.00  (mode=2)
             cansend can0 601#23.03.55.04.04.30.10.00  (mode=3)
  关闭透传:  cansend can0 601#23.03.55.04.00.00.00.00

  力矩标定:  cansend can0 601#23.31.25.00.02.00.00.00  (零漂)
             cansend can0 601#23.31.25.00.02.10.27.00  (10Nm)
  MIT迁移:   cansend can0 601#23.46.25.00.14.00.00.00  (Tmax=20)
             cansend can0 601#23.39.25.00.01.00.00.00  (存Flash)

  保存Flash: cansend can0 601#23.10.10.01.01.00.00.00

  监控:      candump can0 | grep -E '301|302|6C1|6C2'
```
