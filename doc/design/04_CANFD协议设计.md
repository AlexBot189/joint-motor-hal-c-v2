# 外骨骼 CANFD 协议设计

> 版本: V2.0 (基于 V1.2 透传协议 + V1.1 字典表)  
> 日期: 2026-08-01  
> 作者: zhiqiang.yang  
> 项目: KWS EXOSKELETON V2

---

## 1. 物理层

| 参数 | 值 |
|------|-----|
| 总线类型 | CAN FD (ISO 11898-1:2015) |
| 仲裁段速率 | 1 Mbps |
| 数据段速率 | 5 Mbps (BRS 使能) |
| 帧格式 | 标准帧 (11-bit ID) |
| 接口 | can0 (SocketCAN) |
| 节点数 | 1 主站 (SOC) + 2 从站 (电机驱动板) |
| 终端电阻 | 120Ω (板载) |

---

## 2. CAN ID 分配

### 2.1 自定义 PDO 帧

```
COB-ID         方向          DLC    名称
────────────────────────────────────────────
0x080          主站→        0       SYNC 同步帧 (1KHz)
0x100|Dev_ID   主站→从站     7       单轴控制帧
0x110|Dev_ID   主站→从站     9       MIT 单轴控制帧
0x200          主站→广播     64      多轴控制帧 (最多8电机)
0x210          主站→广播     64      MIT 多轴控制帧 (最多6电机)
0x300|Dev_ID   从站→主站     12/16   执行器反馈帧
0x600|Dev_ID   主站→从站     8       SDO 请求 (Client→Server)
0x580|Dev_ID   从站→主站     8       SDO 响应 (Server→Client)
0x680|Dev_ID   从站→主站     8       传感器透传 (mode=0/2)
0x690|Dev_ID   从站→主站     8       运行反馈帧0: 位置+速度 (mode=1/2)
0x6A0|Dev_ID   从站→主站     8       运行反馈帧1: Iq+母线电流+温度+错误码 (mode=1/2)
0x6B0|Dev_ID   从站→主站     8       SPI力矩帧 (mode=1/2)
0x6C0|Dev_ID   从站→主站     32      聚合透传帧 (mode=3)

0x700|Dev_ID   从站→主站     1       NMT Heartbeat (心跳)
0x701|Dev_ID   从站→主站     1       NMT Bootup (启动)
```

**Dev_ID** = 驱动板 CAN 节点 ID (1=右髋, 2=左髋)

### 2.2 SDO 对象字典 (OD)

标准 CiA 402 对象 + KWS 自定义：

```
索引      子索引   权限    类型      名称
─────────────────────────────────────────────────────
0x1010   0x01     RW      U32       Store parameters (Flash保存)
0x2530   0x00     RW      U32       CAN Node ID (修改后需重启)
0x2540   0x00     RW      U32       CAN 数据段波特率
0x2531   0x00     RW      U32       零位设置 (1=机械零位, 2=扭矩零漂)
0x2532   0x00     RW      U32       电流环 P
0x2533   0x00     RW      U32       电流环 I
0x2534   0x00     RW      U32       速度环 P
0x2535   0x00     RW      U32       速度环 I
0x2536   0x00     RW      U32       位置环 P
0x2537   0x00     RW      U32       位置环 I
0x2538   0x00     RW      U32       电机峰值电流 (mA)
0x2539   0x00     RW      U32       Flash保存参数代码
0x2541   0x00     RW      U32       电机方向 (V2使用0x607E)
0x5503   0x04     RW      U32       传感器透传配置
0x5503   0x06     RW      U32       RGB LED 控制
─────────────────────────────────────────────────────
0x2661   0x00     RO      INT32     母线电流 (mA)
0x2662   0x00     RO      U32       驱动器温度 (0.1°C)
0x2663   0x00     RO      U32       电机线圈温度 (0.1°C)
0x603F   0x00     RO      U16       错误码
0x6040   0x00     RW      U16       CiA 402 控制字
0x6041   0x00     RO      U16       CiA 402 状态字
0x6060   0x00     RW      INT8      运行模式指令
0x6064   0x00     RO      INT32     负载端位置 (cnt)
0x606C   0x00     RO      INT32     电机端速度 (RPM)
0x6071   0x00     RW      INT16     电流目标值 (mA)
0x6077   0x00     RO      INT16     力矩传感器当前值 (0.01N.m)
0x6078   0x00     RO      INT16     电流当前值 (mA)
0x6079   0x00     RO      U32      母线电压 (0.1V)
0x607A   0x00     RW      INT32     负载端位置目标值 (cnt)
0x607D   0x01     RW      INT32     负载位置下限 (cnt)
0x607D   0x02     RW      INT32     负载位置上限 (cnt)
0x6080   0x00     RW      U32      电机最大转速 (RPM, 默认2000)
0x6081   0x00     RW      U32      轮廓速度 (RPM)
0x6083   0x00     RW      U32      轮廓加速度 (RPM/s)
0x6084   0x00     RW      U32      轮廓减速度 (RPM/s)
0x60FF   0x00     RW      INT32     速度目标值 (RPM)
0x60F4   0x00     RO      INT32     位置误差 (cnt)
```

---

## 3. 传感器透传协议

### 3.1 配置字 (0x5503:04)

```
bit0-15:  period_div    发送周期分频, 0=关闭
bit16-17: bus_format    0=Classic CAN, 1/2/3=CAN FD
bit18-19: mode          透传模式
bit20-21: force_module  0=DF181, 1=CMD_SPI
bit22-31: 保留         必须为 0
```

SDO 下载格式：`23 03 55 04 PL PH CFG 00`

其中 `CFG = bus_format | (mode << 2) | (force_module << 4)`

### 3.2 周期基准

**V1.2 新基准：1 count = 0.25ms**

```
发送周期(ms) = period_div × 0.25
发送频率(Hz) = 4000 / period_div
```

| period_div | 周期 | 频率 |
|-----------|------|------|
| 0 | 关闭 | 0 |
| 1 | 0.25ms | 4000Hz |
| 4 | 1ms | 1000Hz |
| 8 | 2ms | 500Hz |
| 16 | 4ms | 250Hz |

### 3.3 mode=0: 兼容模式

仅发送 0x680，8 Byte：

```
Byte 0-1:  hall_adc0        U12, 线性霍尔A, 0~4095
Byte 2 (高4bit) + Byte 3: hall_adc1  U12, 线性霍尔B
Byte 4 (高2bit) + Byte 5: hall_adc2  U12, 线性霍尔C
Byte 6-7 (高14bit): force_raw       U14, DF181力矩, 0~16383
Byte 7 (低2bit):    reserved
Byte 8-9 (高10bit): knee_hall       U12, 膝关节霍尔
Byte 9 (低2bit):    key_landing     1bit, 着地开关
Byte 9 (bit3):      data_valid      1bit, 力矩数据有效
Byte 9 (bit4-7):    reserved
```

### 3.4 mode=1: 运行反馈模式

发送 0x690 + 0x6A0 + 0x6B0，各 8 Byte，不发送 0x680。

### 3.5 mode=2: 全部模式

发送 0x680 + 0x690 + 0x6A0 + 0x6B0，共 4 帧 × 8 Byte (V1.1 行为)。

### 3.6 mode=3: 32 Byte 聚合模式 (V1.2 新增)

仅发送一帧 0x6C0+NodeID, 32 Byte CAN FD：

```
Byte 0-7   ← 0x680: Hall ADC0/1/2, DF181 CH0, knee_hall, key_landing, data_valid
Byte 8-15  ← 0x690: 实际位置 S32 LE + 实际速度 S32 LE
Byte 16-23 ← 0x6A0: Iq电流 S16 + 母线电流 S32 + 电机温度 S16 + 错误码 U16
Byte 24-31 ← 0x6B0: DF181双通道 或 CMD_SPI单通道S24 + 有效位 + 错误码
```

四个区域与 V1.1 逐字节兼容，位定义完全一致。

---

## 4. 执行器反馈帧 (0x300)

### 4.1 带力矩传感器版本 (DLC=16)

```
Byte    位宽    字段              类型    单位
─────────────────────────────────────────────
0-1     16bit   负载端实际位置      int16   cnt [-32768,32767]→[-180°,180°]
2-3     16bit   电机端实际速度      int16   RPM
4-5     16bit   实际电流 Iq         int16   mA
6-7     16bit   错误代码           uint16  -
8-9     16bit   电机线圈温度        int16   0.1°C
10-11   16bit   力矩反馈           int16   0.05N.m
12-14   24bit   预留 (电机端编码器)  -      -
15 bit7 1bit    使能状态           0=失能, 1=使能
15 bit6 1bit    抱闸状态           0=吸合, 1=释放
15 bit5 1bit    报错状态           0=正常, 1=报错
15 bit4 1bit    位置到位           0=运行, 1=到位
15 bit3:0 4bit  控制模式反馈
                 0x1=PP, 0x2=PV, 0x3=CSP, 0x4=CSV
                 0x5=电流, 0x6=MIT, 0x7=力矩
```

### 4.2 不带力矩传感器版本 (DLC=12)

```
Byte    位宽    字段              类型    单位
─────────────────────────────────────────────
0-7     同 16 Byte 版
8-9     16bit   电机线圈温度        int16   0.1°C
10      8bit    控制模式反馈        同 16 Byte 版枚举
11 bit7 1bit    使能状态
11 bit6 1bit    抱闸状态
11 bit5 1bit    报错状态
11 bit4 1bit    到位状态
11 bit3:0       预留
```

---

## 5. 单轴控制帧 (0x100 | Dev_ID)

DLC=7, 不变。

```
Byte 0 bit7     使能控制       0=失能, 1=使能
Byte 0 bit6     抱闸控制       0=吸合, 1=释放
Byte 0 bit5     清除错误       1=复位错误
Byte 0 bit4:1   控制模式
                 0x1=PP(轮廓位置)
                 0x2=PV(轮廓速度)
                 0x3=CSP(周期同步位置)
                 0x4=CSV(周期同步速度)
                 0x5=电流环
                 0x6=力矩环
Byte 0 bit0     预留

Byte 1-2        target1        int16, 含义随模式
Byte 3-4        target2        int16, 含义随模式
Byte 5-6        feedforward    int16, 含义随模式
```

### 各模式参数含义

| 模式 | target1 | target2 | feedforward |
|------|---------|---------|-------------|
| PP (0x1) | 目标角度 cnt | 加减速 RPM/s [0,20000] | 轮廓速度 RPM |
| PV (0x2) | 目标速度 RPM | 加减速 RPM/s [0,20000] | 预留 |
| CSP (0x3) | 目标角度 cnt | 预留 | 预留 |
| CSV (0x4) | 目标速度 RPM | 预留 | 预留 |
| 电流 (0x5) | 目标电流 mA | 预留 | 预留 |
| 力矩 (0x6) | 目标力矩 0.05N.m | 预留 | 预留 |

### 控制示例

```
# 电流模式, 1000mA
TX: 0x101 [A0 03 E8 00 00 00 00]
     ^^  ^^^^^^  ^^^^^^  ^^^^^^
     ctrl 1000mA    -       -

# 力矩模式, 10N.m = 200×0.05
TX: 0x101 [C0 00 C8 00 00 00 00]
     ^^  ^^^^^^  ^^^^^^  ^^^^^^
     ctrl  200      -       -

# PP 模式, 90°=16384cnt, accel=1000RPM/s, vel=10RPM
TX: 0x101 [20 40 00 03 E8 00 0A]
```

---

## 6. MIT 控制帧 (0x110 | Dev_ID)

DLC=9, 独立帧类型。MIT 不再通过 0x100 的 mode 位控制。

```
Byte 0: 控制字 (同单轴控制帧, mode=6)
Byte 1-2: 目标位置      16bit [0-65535]→(-PosMax, PosMax)
Byte 3 + Byte4[7:4]: 目标速度   12bit [0-4095]→(-VelMax, VelMax)
Byte4[3:0] + Byte5:  Kp        12bit [0-4095]→[0-500]
Byte6 + Byte7[7:4]:  Kd        12bit [0-4095]→[0-5]
Byte7[3:0] + Byte8:  目标力矩   12bit [0-4095]→(-Tmax, Tmax)
```

### 12bit 跨字节编码

```
目标速度 = (Byte[1] << 4) | (Byte[4] >> 4)
Kp      = ((Byte[4] & 0x0F) << 8) | Byte[5]
Kd      = (Byte[6] << 4) | (Byte[7] >> 4)
目标力矩 = ((Byte[7] & 0x0F) << 8) | Byte[8]
```

---

## 7. 多轴控制帧 (0x200 / 0x210)

### 7.1 普通多轴 (0x200, DLC=64)

8 个 slot, 每 slot 7 字节 (同单轴控制帧)，ID 在 Byte[56-63]:

```
Byte 0-6:    slot 1 控制指令
Byte 7-13:   slot 2
...
Byte 49-55:  slot 8
Byte 56:     slot 1 执行器 ID
Byte 57:     slot 2 执行器 ID
...
Byte 63:     slot 8 执行器 ID
```

未使用的 slot 填 0。

### 7.2 MIT 多轴 (0x210, DLC=64)

最多 6 个 slot, 每 slot 9 字节 (同 MIT 单轴帧)：

```
Byte 0-8:    slot 1 MIT 指令
Byte 9-17:   slot 2
...
Byte 45-53:  slot 6
Byte 56-61:  slot 1-6 对应 ID
```

---

## 8. 波特率配置 (0x2540)

**V2 新映射**：

| 值 | 仲裁段 | 数据段 | 说明 |
|----|--------|--------|------|
| 0 | 500K | - | Classic CAN |
| 1 | 1M | - | Classic CAN |
| 2 | 1M | 1M | CAN FD (无加速) |
| 3 | 1M | 2M | CAN FD |
| 4 | 1M | 5M | CAN FD (外骨骼默认) |

**注意**：V1 映射是 1=5M, 2=4M, 3=2M, 4=1M。**不兼容**。

---

## 9. NMT 状态切换

### 9.1 CiA 402 状态机

```
上电 → NOT_READY_TO_SWITCH_ON → SWITCH_ON_DISABLED
                                        ↓ (Shutdown 0x06)
                                  READY_TO_SWITCH_ON
                                        ↓ (Switch On 0x07)
                                  SWITCHED_ON
                                        ↓ (Enable Operation 0x0F)
                                  OPERATION_ENABLED
```

### 9.2 控制字 (0x6040)

```
0x06  Shutdown
0x07  Switch On
0x0F  Enable Operation
0x00  Disable Voltage (急停)
0x80  Fault Reset
```

### 9.3 状态字 (0x6041)

```
bit0: Ready to Switch On
bit1: Switched On
bit2: Operation Enabled
bit3: Fault
bit4: Voltage Enabled
bit5: Quick Stop
bit6: Switch On Disabled
bit7: Warning
bit8: Manufacturer Specific
bit9: Remote
bit10: Target Reached
bit11: Internal Limit Active
```

---

## 10. 心跳协议 (NMT Heartbeat)

```
COB-ID: 0x700 + Node_ID
DLC: 1
Byte 0:
  0x00: Boot-up
  0x04: Stopped
  0x05: Operational
  0x7F: Pre-operational
```

外骨骼配置 `heartbeat_ms: 0` 关闭心跳监控，仅保留 Bootup 检测。

---

## 11. SYNC 同步帧

```
COB-ID: 0x080
DLC: 0
周期: 1ms (1KHz)
```

由 stark_node 控制线程发送，用于触发从站的同步 TPDO (0x300 反馈帧)。

---

## 12. LED 控制 (0x5503:06)

```
SDO 写 U32:
  byte0: enable_mask | mode
    bit7:4   LED1~4 使能掩码
    bit3:0   led_mode_t (0=常亮, 1=闪烁, 2=呼吸, 3=流水)
  byte1: R (0-255)
  byte2: G (0-255)
  byte3: B (0-255)
```

---

## 13. 传感器透传 0x6A0 帧详细定义

```
Byte 0-1:  Iq 电流         int16  mA   (与 0x6078 一致)
Byte 2-3:  母线电流        int16  mA   (超范围时饱和)
Byte 4-5:  电机线圈温度    int16  0.1°C
Byte 6-7:  错误码          uint16 -   (与 0x603F 一致)
```

注意：V1.2 聚合帧中字节顺序与 V1.1 独立帧完全一致，所有多字节字段为小端。

---

## 14. 透传帧 0x6B0 (CMD_SPI 模式)

```
Byte 0-2:  CH0 力矩值     S24, 小端 (带符号扩展)
Byte 3:    保留
Byte 4 bit0: 数据有效位
Byte 5:    错误码         (10=传感器校验失败)
Byte 6-7:  预留
```

CMD_SPI 力矩值转换：`float torque = (int32_t)(raw_s24 << 8 >> 8) / 100.0f` (单位: N.m)

---

## 15. SDO 通信约束

- SDO 请求和响应成对出现
- 超时默认 200ms
- SDO 操作在非 RT 线程执行（状态机/校准/主循环）
- RT 线程禁止 SDO（会阻塞 CAN 总线并破坏实时性）
- 电机使能状态下禁止某些 SDO 写（如零位设置，V2 放宽）

### SDO 帧格式

```
请求 (主站→从站):
  Byte 0: 命令码 (0x23=快速写4B, 0x40=读请求, 0x2B=快速写2B)
  Byte 1-2: 索引 (小端)
  Byte 3: 子索引
  Byte 4-7: 数据 (小端, 快速传输)

响应 (从站→主站):
  Byte 0: 命令码 (0x60=写成功, 0x4F/0x4B/0x47/0x43=读响应)
  Byte 1-2: 索引
  Byte 3: 子索引
  Byte 4-7: 数据 (读响应时)

  ABORT (0x80):
  Byte 0: 0x80
  Byte 1-2: 索引
  Byte 3: 子索引
  Byte 4-7: Abort Code (U32 LE)
```
