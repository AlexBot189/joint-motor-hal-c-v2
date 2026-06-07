# motor_hal_c API 使用文档

> 适配: 无锡巨蟹智能驱动一体化通讯模组  
> 协议: CAN/CANFD + CANopen CiA 402  
> 物理: CANFD 标准帧 11bit, 仲裁段 1Mbps, 数据段 5Mbps

---

## 目录

1. [架构概览](#1-架构概览)
2. [初始化与生命周期](#2-初始化与生命周期)
3. [SDO 控制（串行命令）](#3-sdo-控制串行命令)
4. [SDO 读取](#4-sdo-读取)
5. [SDO 参数配置](#5-sdo-参数配置)
6. [PDO 控制 — 巨蟹自定义（实时）](#6-pdo-控制--巨蟹自定义实时)
7. [PDO 控制 — 标准 CANopen](#7-pdo-控制--标准-canopen)
8. [反馈缓存读取](#8-反馈缓存读取)
9. [传感器透传](#9-传感器透传)
10. [电机校准](#10-电机校准)
11. [多轴广播控制](#11-多轴广播控制)
12. [错误码与故障排查](#12-错误码与故障排查)
13. [完整使用示例](#13-完整使用示例)

---

## 1. 架构概览

```
┌───────────────────────────────────────────────────┐
│ 上层 / motor_tool CLI                             │
│   SDO: 使能/模式/参数/读取 (低速, 50~200ms)        │
│   PDO: 位置/速度/电流/MIT (实时, <100μs)          │
│   PDO: 标准 RPDO 发送 + 自定义 TPDO 接收           │
├───────────────────────────────────────────────────┤
│ motor_hal 库 (libmotor_hal.so)                    │
│   ┌──────────┐ ┌──────────┐ ┌────────────────┐   │
│   │ SDO 客户端│ │PDO 发送  │ │ 接收线程        │   │
│   │ (队列+    │ │(自定义+  │ │ (SDO入队+      │   │
│   │  condvar) │ │  MIT+多轴)│ │  反馈缓存+     │   │
│   └──────────┘ └──────────┘ │  TPDO/EMCY分发) │   │
│                             └────────────────┘   │
├───────────────────────────────────────────────────┤
│ SocketCAN (can_driver)                            │
│   CAN RAW socket + select + CANFD BRS             │
└───────────────────────────────────────────────────┘
```

**关键原则**:
- **SDO**: 同步阻塞，50~200ms 延迟，用于低频操作（配置、启动、查询）
- **自定义 PDO**: 非阻塞，<100μs 延迟，直接写 SocketCAN，用于实时控制
- **标准 PDO**: 需要先 SDO 配置映射，再发送/接收
- **接收线程**: 后台自动接收反馈帧，更新缓存，控制线程只读缓存不触网卡

---

## 2. 初始化与生命周期

### 2.1 硬件准备

```bash
# 配置 CANFD 接口
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

# 仅 CAN (不启用 FD):
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 2.2 C API

```c
#include "motor_hal.h"

// 1. 创建 HAL
motor_hal_t *hal = motor_hal_create();
if (!hal) { perror("create"); return -1; }

// 2. 打开 CAN/CANFD 接口
//    FD 模式: arb=1M, data=5M
//    CAN 模式: arb=1M, data=0 (忽略)
int ret = motor_hal_init(hal, "can0", 1000000, 5000000);
if (ret < 0) { fprintf(stderr, "CAN init failed: %d\n", ret); return -1; }

// 3. 注册电机
motor_config_t cfg = {0};
cfg.node_id           = 1;       // CAN 节点 ID
cfg.heartbeat_ms      = 2000;    // 心跳周期
cfg.profile_accel     = 5000;    // 加速度 RPM/s
cfg.profile_decel     = 5000;    // 减速度 RPM/s
cfg.profile_velocity  = 20;      // 最大轨迹速度 RPM
cfg.disable_watchdog  = true;    // 关闭看门狗 (推荐)
cfg.auto_enable       = true;    // ★ 控制 startup 内部是否自动走 DS402 使能
                                  //    true: startup 一步到位 (Bootup+配置+使能)
                                  //    false: startup 只配置不使能, 后面手动 motor_hal_enable()
cfg.bootup_timeout_ms = 5000;    // 启动超时
motor_hal_add_motor(hal, &cfg);

// 4. 启动接收线程 ★ 必须在 startup 之前
motor_hal_recv_start(hal);

// 5. 启动电机 (Bootup → SDO 配置 → DS402 使能)
ret = motor_hal_startup(hal, 1, 5000);
if (ret != 0) { fprintf(stderr, "startup failed\n"); }

// ... 控制循环 ...

// 6. 销毁 (自动脱使能 + 停线程 + 关 CAN)
motor_hal_destroy(hal);
```

### 2.3 CLI

```bash
# 启动守护进程 (后台管理 CAN + 电机)
motor_tool daemon can0 &

# 上电启动电机
motor_tool startup 1       # 电机 1
motor_tool startup 0       # 广播: 1~4
motor_tool enable 1        # 使能
motor_tool disable 1       # 脱使能
motor_tool reset 1         # 故障复位

# 停止守护进程
motor_tool stop
```

---

## 3. SDO 控制（串行命令）

SDO 通过队列+条件变量实现同步阻塞调用，每次 50~200ms。

### 3.1 电流（力矩）控制 — 带时序

> **重要**: motor_tool (CLI) 层自动完成了使能+切模式+写目标的完整时序。
> 如果直接用 HAL C API 做 SDO 电流控制, 需要**自己调用使能**:
> ```c
> motor_hal_enable(hal, 1);                               // 步骤1: 使能
> motor_hal_set_mode(hal, 1, MOTOR_MODE_CURRENT);         // 步骤2: 切模式
> motor_hal_sdo_write(hal, 1, 0x6071, 0x00, 1000, 2);    // 步骤3: 写目标
> ```

**时序**: 使能(SDO 0x6040) → 切电流模式(0x6060=0x0A) → 写目标电流(0x6071)

```c
// C API
motor_hal_set_mode(hal, 1, MOTOR_MODE_CURRENT);          // 切电流模式
motor_hal_sdo_write(hal, 1, 0x6071, 0x00, 1000, 2);     // 写目标电流 1000mA
```

```bash
# CLI — 自动完成完整时序 (使能→模式→写目标)
motor_tool torque 1 1000        # 电机1: 1000mA 电流
motor_tool torque 2 -500        # 电机2: -500mA 反向电流
```

### 3.2 速度控制 — 带时序

**时序**: 使能 → 切 PV 模式(0x6060=0x03) → 设加减速(0x6083/0x6084) → 写目标速度(0x60FF)

```c
// C API
motor_hal_set_mode(hal, 1, MOTOR_MODE_PROFILE_VEL);
motor_hal_set_accel_decel(hal, 1, 1000, 1000);
motor_hal_set_speed_target(hal, 1, 500);  // RPM
```

```bash
# CLI — 自动完成完整时序
motor_tool speed 1 5000 100000   # 50RPM, 加速1000RPM/s
motor_tool speed 1 -3000         # -30RPM 反转
```

### 3.3 位置控制 — 带时序

**时序**: 使能 → 切 PP 模式(0x6060=0x01) → 设加减速 → 设轨迹速度 → 写目标位置(0x607A) → 启动(0x6040=0x4F)

```c
// C API
motor_hal_set_mode(hal, 1, MOTOR_MODE_PROFILE_POS);
motor_hal_set_accel_decel(hal, 1, 2000, 2000);
motor_hal_set_profile_velocity(hal, 1, 10);
motor_hal_set_pos_target(hal, 1, 16384);  // 90° = 16384 cnt
motor_hal_set_pos_ctrl(hal, 1, true);     // 启动运动
```

```bash
# CLI — 自动完成完整时序
motor_tool abs 1 4500          # 电机1: 转动 45°
motor_tool abs 1 -9000        # 电机1: 转动 -90°
motor_tool abs_stop 1         # 停止位置运动
motor_tool abs_accel 200000   # 加减速 2000RPM/s
motor_tool abs_speed 1000     # 轨迹速度 10RPM (输出端)
```

### 3.4 通用 SDO 读写

```c
// C API
motor_hal_sdo_write(hal, 1, 0x6071, 0x00, 2000, 2);   // 写 0x6071 = 2000 (2字节)
uint32_t val;
motor_hal_sdo_read_u32(hal, 1, 0x6064, 0x00, &val);    // 读 0x6064
```

```bash
# CLI
motor_tool sdoread 1 0x6064           # 读位置
motor_tool sdoread 1 0x100A           # 读固件版本
motor_tool sdowrite 1 0x6071 0 2000 2 # 写目标电流 2000mA
```

### 3.5 SDO 控制模式对照

| 模式 | 枚举值 | SDO 0x6060 值 | 说明 |
|------|--------|---------------|------|
| PP 轮廓位置 | `MOTOR_MODE_PROFILE_POS` | 0x01 | 梯形速度曲线位置控制 |
| PV 轮廓速度 | `MOTOR_MODE_PROFILE_VEL` | 0x02 | 梯形加减速速度控制 |
| CSP 同步位置 | `MOTOR_MODE_CSP` | 0x03 | 每周期给目标位置 |
| CSV 同步速度 | `MOTOR_MODE_CSV` | 0x04 | 每周期给目标速度 |
| 电流环 | `MOTOR_MODE_CURRENT` | 0x0A | 直接控制 Iq 电流 |
| MIT 阻抗 | `MOTOR_MODE_MIT` | 0x06 | 力位混合阻抗控制 |

---

## 4. SDO 读取

### 4.1 C API — 状态查询

```c
// DS402 状态
motor_state_t st = motor_hal_get_state(hal, 1);
printf("State: %s\n", motor_state_str(st));

// Statusword 原始值
uint16_t sw = motor_hal_get_statusword(hal, 1);

// 位置/速度/电流 (通过 SDO, 50~200ms)
int32_t pos = motor_hal_get_position(hal, 1);
int32_t vel = motor_hal_get_velocity(hal, 1);
int32_t cur = motor_hal_get_current(hal, 1);

// PID 参数
motor_pid_t pid;
motor_hal_read_pid(hal, 1, &pid);

// 故障码 / 温度 / 最大电流
uint16_t err;
motor_hal_get_fault_code(hal, 1, &err);
int32_t mos_temp, motor_temp;
motor_hal_get_mos_temp(hal, 1, &mos_temp);
motor_hal_get_motor_temp(hal, 1, &motor_temp);
uint32_t max_ma;
motor_hal_get_max_current(hal, 1, &max_ma);
```

### 4.2 CLI — 读取命令

```bash
motor_tool read angle 1       # 角度 (°)
motor_tool read speed 1       # 速度 (RPM)
motor_tool read current 1     # 电流 (mA)
motor_tool read temp 1        # 温度 (0.1°C)
motor_tool read state 1       # DS402 状态
motor_tool read error 1       # 故障码
motor_tool read version 1     # 固件版本
motor_tool read mode 1        # 运行模式
motor_tool read pid 1         # PID 参数
motor_tool read all 1         # 全部信息
motor_tool read all 0         # 广播: 读所有已注册电机

# 限位
motor_tool read_limit_pos 1   # 正限位
motor_tool read_limit_neg 1   # 负限位
```

---

## 5. SDO 参数配置

### 5.1 C API

```c
// PID
motor_pid_t pid = { .current_p=100, .current_i=50, .velocity_p=200,
                    .velocity_i=30, .position_p=80, .position_i=10 };
motor_hal_set_pid(hal, 1, &pid);

// 限位
motor_hal_set_limits(hal, 1, 90.0f, -90.0f);  // ±90°

// 零位
motor_hal_set_zero(hal, 1);

// 保存 Flash
motor_hal_save_flash(hal, 1);

// 其他
motor_hal_set_max_current(hal, 1, 5000);       // 最大电流 5A
motor_hal_set_heartbeat(hal, 1, 1000);         // 心跳 1s
motor_hal_set_node_id(hal, 1, 5);              // 改节点 ID (重启生效)
motor_hal_set_canfd_baud(hal, 1, 1);           // 1=5M, 2=4M, 3=2M, 4=1M
```

### 5.2 CLI

```bash
motor_tool setzero 1                         # 零位标定 (自动失能)
motor_tool limit_pos 1 9000                  # 正限位 90°
motor_tool limit_neg 1 -9000                 # 负限位 -90°

# PID 设置 — 写入 0x2532~0x2537 (只改 RAM, 断电丢失!)
motor_tool pid 1 100 50 200 30 80 10        # CP CI VP VI PP PI
#                │  │   │   │  │  └─ PP  位置环 P 增益
#                │  │   │   │  └─── PI  位置环 I 增益
#                │  │   │   └────── VI  速度环 I 增益
#                │  │   └────────── VP  速度环 P 增益
#                │  └────────────── CI  电流环 I 增益
#                └──────────────── CP  电流环 P 增益

# 读 PID
motor_tool read pid 1                        # 读 PID 参数

# ★ 保存到 Flash (PID/限位等参数需手动保存, 否则断电丢失)
motor_tool save 1                            # 保存到 Flash

---

## 6. PDO 控制 — 巨蟹自定义（实时）

**不需要 SDO 映射，直接发送。延迟 <100μs，RT 安全。**

### 6.1 单轴控制 PDO (0x100+ID, 7字节)

```
Byte[0]   = flags: bit7使能 | bit6抱闸 | bit5清错 | bit4:1模式 | bit0预留
Byte[1-2] = target1  (int16, 大端)
Byte[3-4] = target2  (uint16, 大端)
Byte[5-6] = feedforward (int16, 大端)
```

#### C API

```c
// 位置控制 (PP 模式) — 最常用
motor_hal_set_position(hal, 1, 45.0f);    // 转到 45°
motor_hal_set_position(hal, 1, -30.0f);   // 转到 -30°

// 速度控制 (PV 模式)
motor_hal_set_velocity(hal, 1, 500.0f);   // 500 RPM 正转
motor_hal_set_velocity(hal, 2, -300.0f);  // 300 RPM 反转

// 电流/力矩控制 (电流环)
motor_hal_set_torque(hal, 1, 2000);       // 2000 mA
motor_hal_set_torque(hal, 1, -500);       // -500 mA 反向

// 急停
motor_hal_quick_stop(hal, 1);

// 抱闸
motor_hal_set_brake(hal, 1, true);   // 释放
motor_hal_set_brake(hal, 1, false);  // 吸合

// 停止 (目标位置=0)
motor_hal_stop(hal, 1);

// 通用 raw 接口
motor_hal_ctrl_raw(hal, 1, MOTOR_MODE_CSP, 0, 0, 0);
```

#### CLI — 单轴 PDO 控制 (实时)

```bash
# PDO 单轴 (不经过 SDO, <100μs)
motor_tool pdo 1 pos 4500          # 电机1: 位置 45°
motor_tool pdo 1 pos -9000        # 电机1: 位置 -90°
motor_tool pdo 1 vel 5000         # 电机1: 速度 50 RPM
motor_tool pdo 1 cur 1000         # 电机1: 电流 1000 mA
motor_tool pdo 1 csp 16384        # 电机1: CSP 16384cnt

# PDO 多轴广播 (一帧控制多个电机, 时间同步)
motor_tool multi pos 1:4500 2:-4500          # 双关节位置
motor_tool multi vel 1:5000 2:-3000          # 双关节速度
motor_tool multi cur 1:1000 2:500            # 双关节电流
motor_tool multi csp 1:16384 2:-16384       # 双关节 CSP (每帧回显反馈)

# MIT 阻抗控制
motor_tool mit 1 0 0 30 5 0                 # 柔顺模式 (低刚度, 可被人推动)
motor_tool mit 1 3000 0 200 30 0            # 刚性位置 30°
```

> **PDO vs SDO 对比**:
> - `motor_tool pdo` = PDO 自定义帧 (0x100+ID, 7B), <100μs, 适合实时循环
> - `motor_tool torque/speed/abs` = SDO 串行命令, 300~500ms, 适合手动调试

### 6.2 MIT 阻抗控制 PDO (0x110+ID, 9字节)

```
Byte[0]      = flags (同单轴)
Byte[1-2]    = 目标位置 (uint16, 大端)
Byte[3][7:4] = 目标速度低4bit + Kp高4bit
Byte[5]      = Kp低8bit
Byte[6]      = Kd高8bit (bits 11:4)
Byte[7][3:0] = 力矩高4bit
Byte[8]      = 力矩低8bit
```

#### C API

```c
// 柔顺模式 (低刚度, 可被人推动)
motor_hal_mit_control(hal, 1, 0, 0, 0.3f, 0.05f, 0);

// 刚性位置控制 (高刚度)
motor_hal_mit_control(hal, 1, 30.0f, 0, 2.0f, 0.3f, 0);

// 带前馈力矩
motor_hal_mit_control(hal, 1, 0, 0, 1.0f, 0.2f, 500.0f);
```

### 6.3 SYNC 帧 (0x080, 0字节)

触发从站同步上报反馈。配合 TPDO 映射使用。

```c
// 手动发 SYNC
motor_hal_sync(hal);

// 自动定时 SYNC (5ms = 200Hz)
motor_hal_sync_start(hal, 5000);
motor_hal_sync_stop(hal);
```

---

## 7. PDO 控制 — 标准 CANopen

**需要先通过 SDO 配置映射表，驱动板才知道每字节对应哪个 OD。**

### 7.1 映射时序

根据巨蟹文档，时序为：

```
1. 关闭其他 PDO 通道 (best-effort)
2. 设置传输类型 (comm sub0x02)
3. 清空映射 (map sub0x00 = 0)
4. 写入映射条目 (map sub0x01, sub0x02, ...)
5. 保存映射数量 (map sub0x00 = N)
6. 启用 PDO (comm sub0x01 = COB-ID)
```

### 7.2 TPDO 映射 (从站→主站, 反馈上报)

```c
// C API — 自定义映射
pdo_map_entry_cfg_t tpdo[] = {
    {0x6041, 0x00, 16},  // Statusword
    {0x6064, 0x00, 32},  // Position Actual
    {0x606C, 0x00, 32},  // Velocity Actual
    {0x6078, 0x00, 16},  // Current Actual
};
motor_hal_pdo_map(hal, 1, tpdo, 4, 0, PDO_TYPE_TPDO, 0x181, 1);

// 便捷封装 (固定映射 Statusword+Position+Velocity+Current)
motor_hal_tpdo_config(hal, 1, 1);  // 每个 SYNC 上报一次

// 注册自定义 TPDO 解析回调
void my_tpdo_cb(uint8_t id, const canfd_frame_t *f, void *ctx) {
    int16_t cur  = f->data[0] | (f->data[1] << 8);
    int32_t pos  = f->data[2] | (f->data[3] << 8) | ...;
    printf("id=%d cur=%d pos=%d\n", id, cur, pos);
}
motor_hal_set_tpdo_cb(hal, 1, my_tpdo_cb, NULL);
```

```bash
# CLI — TPDO 映射
motor_tool tpdo_map 1 0x181 1 0x6041 0 16 0x6064 0 32 0x606C 0 32 0x6078 0 16
```

### 7.3 RPDO 映射 (主站→从站, 控制命令)

```c
// C API — 映射 RPDO1
pdo_map_entry_cfg_t rpdo[] = {
    {0x6040, 0x00, 16},  // Controlword
    {0x607A, 0x00, 32},  // Target Position
    {0x6071, 0x00, 16},  // Target Current
};
motor_hal_pdo_map(hal, 1, rpdo, 3, 0, PDO_TYPE_RPDO, 0x201, 255);

// 发送 RPDO 帧 (数据小端, 按映射顺序)
uint8_t data[8] = {
    0x0F, 0x00,              // Controlword = 0x000F (EnableOp)
    0x00, 0x40, 0x00, 0x00, // TargetPos = 16384 cnt (90°)
    0xE8, 0x03,              // TargetCur = 1000 mA
};
motor_hal_rpdo_send(hal, 1, data, 8);
```

```bash
# CLI — RPDO 映射
motor_tool rpdo_map 1 0x201 255 0x6040 0 16 0x607A 0 32 0x6071 0 16

# CLI — 发送 RPDO 帧
motor_tool rpdo_send 1 0F00 00004000 03E8
#                       CW   Pos=90°   Cur=1000mA
```

### 7.4 PDO 传输类型

| 值 | 含义 |
|----|------|
| 0 | 同步非周期 (收到SYNC且数据变时发) |
| 1~240 | 同步周期 (每 N 个 SYNC 发一次) |
| 254 | 异步, 厂商事件触发 |
| 255 | 异步, 设备子协议触发 |

---

## 8. 反馈缓存读取

> **watch / report / sensor 三者区别**:

| 命令 | 数据来源 | 需要配置？ | 原理 |
|------|---------|:---:|------|
| `watch 200` | 反馈帧 0x300 缓存 | ❌ 零配置 | motor_tool 轮询 `motor_hal_get_feedback()` 缓存 |
| `report 5` | 反馈缓存 + 传感器缓存 | 需先 `sensor config` | 类似 GD32 的 @CA, 独立线程周期输出两者 |
| `sensor watch 1` | 透传帧 0x680 缓存 | 需先 `sensor config` | motor_tool 轮询 `motor_hal_get_sensor()` 缓存 |

- **watch**: 读 PDO 反馈帧 (0x300), 驱动板收到任何 PDO/SYNC 后自动上报, 不需配置
- **report**: 读 feedback+sensor 两者, 用于类似 GD32 的完整数据上报场景
- **sensor watch**: 读透传帧 (0x680), 必须先 SDO 配 0x5503 开启透传, 驱动板按配置周期上报

反馈数据来自驱动板周期性上报的反馈帧 (0x300+ID, 12字节)，由接收线程自动更新缓存。

**读取缓存不触发 SDO，延迟 ~100ns，RT 安全。**

### 8.1 C API

```c
motor_feedback_t fb;
int ret = motor_hal_get_feedback(hal, 1, &fb);
if (ret == 0) {
    float angle = motor_counts_to_deg(fb.position);  // 角度(°)
    int   rpm   = fb.velocity;                        // 速度(RPM)
    int   ma    = fb.current_iq;                      // 电流(mA)
    float temp  = motor_temp_to_c(fb.temperature);   // 温度(°C)

    // 状态位
    bool enabled       = fb.status_byte & 0x80;  // bit7
    bool brake_release = fb.status_byte & 0x40;  // bit6
    bool has_error     = fb.status_byte & 0x20;  // bit5
    bool target_hit    = fb.status_byte & 0x10;  // bit4
}

// 便捷位字段提取
bool fbk_enabled       = feedback_is_enabled(&fb);
bool fbk_brake         = feedback_brake_released(&fb);
bool fbk_error         = feedback_has_error(&fb);
bool fbk_target_reached = feedback_target_reached(&fb);

// 错误码字符串
const char *err_str = feedback_error_string(fb.error_code);
```

### 8.2 反馈帧格式 (0x300+ID, 12字节)

```
Byte[0-1]   = 负载端实际位置 (int16, 大端, [-32768~32767] → [-180°~180°])
Byte[2-3]   = 电机端速度 (int16, 大端, RPM)
Byte[4-5]   = Iq 实际电流 (int16, 大端, mA)
Byte[6-7]   = 错误码 (uint16, 大端)
Byte[8-9]   = 线圈温度 (int16, 大端, 0.1°C)
Byte[10]    = 控制模式反馈 (0x01=PP, 0x02=PV, 0x03=CSP, 0x04=CSV, 0x05=Current)
Byte[11]    = 状态字节: bit7使能|bit6抱闸|bit5错误|bit4到位
```

### 8.3 反馈回调

```c
void my_fb_cb(uint8_t node_id, const motor_feedback_t *fb, void *ctx) {
    printf("Motor %d: pos=%d speed=%d cur=%d\n",
           node_id, fb->position, fb->velocity, fb->current_iq);
}
motor_hal_set_feedback_cb(hal, 1, my_fb_cb, NULL);
```

---

## 9. 传感器透传

透传帧 (0x680+ID, 8字节, 小端 bit-packed):

```
bits[11:0]  = Hall ADC0 (U12)
bits[23:12] = Hall ADC1 (U12)
bits[35:24] = Hall ADC2 (U12)
bits[49:36] = DF181 Force (U14)
bits[61:50] = Knee ADC (U12)
bit[62]     = HW SW PC9
bit[63]     = Data Valid
```

### 9.1 C API

```c
// 启动传感器透传 (period_div * 250μs)
motor_hal_sensor_config(hal, 1, 4, 3);  // 4*250μs=1ms, CANFD BRS

// 读取传感器缓存
motor_sensor_t s;
motor_hal_get_sensor(hal, 1, &s);
printf("Hall: %d %d %d Force: %d Knee: %d Sw: %d Valid: %d\n",
       s.hall_adc0, s.hall_adc1, s.hall_adc2,
       s.force_raw, s.knee_adc, s.hw_sw_pc9, s.data_valid);

// 传感器回调
void my_sensor_cb(uint8_t id, const motor_sensor_t *s, void *ctx) { ... }
motor_hal_set_sensor_cb(hal, 1, my_sensor_cb, NULL);

// 停止透传
motor_hal_sensor_stop(hal, 1);
```

### 9.2 CLI

```bash
motor_tool sensor config 1 1        # 1ms/1KHz 透传
motor_tool sensor config 1 10       # 10ms/100Hz 透传
motor_tool sensor read 1            # 读一次
motor_tool sensor watch 1           # 持续显示 (Ctrl+C 退出)
motor_tool sensor stop 1            # 停止透传
```

---

## 10. 电机校准

设零位 → 检测位置 ±1° → 使能 + 电流模式 + 开透传。

### 10.1 C API

```c
motor_calib_t *cal = motor_calib_create(hal);

motor_calib_config_t cal_cfg = {
    .motor_id_r          = 1,
    .motor_id_l          = 2,
    .timeout_ms          = 10000,
    .angle_threshold_deg = 1.0f,
    .ctrl_mode           = MOTOR_MODE_CURRENT,
};

motor_calib_start(cal, &cal_cfg);

// 轮询 (每 20ms)
while (1) {
    motor_calib_state_t st = motor_calib_poll(cal);
    if (st == MOTOR_CALIB_DONE) break;
    if (st == MOTOR_CALIB_TIMEOUT) { /* 超时 */ break; }
    usleep(20000);
}

motor_calib_exit(cal);
motor_calib_destroy(cal);
```

### 10.2 CLI

```bash
motor_tool calib start 1 2          # 校准电机关节 R=1 L=2
motor_tool calib status             # 查看校准状态
motor_tool calib exit               # 退出校准
```

---

## 11. 多轴广播控制

**不需要 SDO 映射。** 一帧 64 字节 CANFD 同时控制最多 8 个电机。

```
Byte[0-6]    = 电机1 命令 (同单轴 PDO 7字节)
Byte[7-13]   = 电机2 命令
...
Byte[49-55]  = 电机8 命令
Byte[56-63]  = ID 映射表 (8×1字节)
```

**反馈**: 每个电机会各自回复一帧 `0x300+ID` (12字节)，由接收线程自动捕获。
`motor_tool multi` 命令在发送后自动等待 20ms 并回显所有电机反馈。

### 11.1 C API

```c
multi_axis_cmd_t cmds[2] = {
    {
        .node_id       = 1,
        .mode          = MOTOR_MODE_CSP,
        .enable        = true,
        .release_brake = true,
        .target1       = 16384,   // 电机1: 90°
        .target2       = 0,
        .feedforward   = 0,
    },
    {
        .node_id       = 2,
        .mode          = MOTOR_MODE_CSP,
        .enable        = true,
        .release_brake = true,
        .target1       = -16384,  // 电机2: -90°
        .target2       = 0,
        .feedforward   = 0,
    },
};
motor_hal_multi_ctrl(hal, cmds, 2);
```

#### CLI

```bash
# 多轴广播 — 一帧同时控制, 自动回显反馈
motor_tool multi pos 1:4500 2:-4500          # 双关节位置 45°/-45°
motor_tool multi vel 1:5000 2:-3000          # 双关节速度 50/-30 RPM
motor_tool multi cur 1:1000 2:500            # 双关节电流 1000/500mA
motor_tool multi csp 1:16384 2:-16384       # 双关节 CSP
```

---

## 12. 错误码与故障排查

### 12.1 错误码表

| 错误码 | 含义 | 清除方式 |
|--------|------|----------|
| 0x0001 | 过压 (母线>72V) | 重启 |
| 0x0002 | 欠压 (母线<40V) | 重启 |
| 0x0004 | 过温 (>95°C 持续100ms) | 降温到85°C以下指令清除 |
| 0x0008 | 堵转 (Iq>最大电流持续2s) | 重启 |
| 0x0010 | 过载 (Iq积分>阈值) | 重启 |
| 0x0020 | 速度跟踪误差过大 (>320RPM) | 重启 |
| 0x0040 | 正/负限位 | 向限位反方向运行 |
| 0x0100 | 输出端编码器异常 | 重启 |
| 0x0200 | 位置环过速 (CSP Δ>100cnt/ms) | 指令清除 |
| 0x0400 | 电流采样错误 | 重启 |
| 0x0800 | 电流跟踪误差超差 | 重启 |
| 0x1000 | 位置跟踪误差过大 (>2000cnt) | 重启 |

### 12.2 EMCY 紧急报文 (0x080+ID, 8字节)

接收线程自动处理，触发 `motor_error_cb_t` 回调。

```c
void my_err_cb(uint8_t id, uint16_t err_code, void *ctx) {
    const char *s = motor_utils_emcy_str(err_code);
    fprintf(stderr, "Motor %d EMCY: 0x%04X (%s)\n", id, err_code, s ? s : "?");
}
motor_hal_set_error_cb(hal, 1, my_err_cb, NULL);
```

### 12.3 CLI

```bash
motor_tool read error 1       # 读故障码
motor_tool reset 1            # 故障复位
motor_tool reboot 1           # 电机重启 (NMT Reset Node)
```

---

## 13. 完整使用示例

### 13.1 C 上层最小示例 — SDO 控制

```c
#include "motor_hal.h"
#include <unistd.h>

int main() {
    motor_hal_t *hal = motor_hal_create();
    motor_hal_init(hal, "can0", 1000000, 5000000);

    motor_config_t cfg = { .node_id=1, .auto_enable=true,
                           .disable_watchdog=true, .bootup_timeout_ms=5000 };
    motor_hal_add_motor(hal, &cfg);

    motor_hal_recv_start(hal);              // ★ 先启动接收
    motor_hal_startup(hal, 1, 5000);        // 上电使能

    // SDO: 切电流模式, 写目标电流
    motor_hal_set_mode(hal, 1, MOTOR_MODE_CURRENT);
    motor_hal_sdo_write(hal, 1, 0x6071, 0, 2000, 2);

    sleep(2);

    motor_hal_destroy(hal);
    return 0;
}
```

### 13.2 C 上层示例 — PDO 实时控制循环

```c
#include "motor_hal.h"
#include <unistd.h>
#include <signal.h>

static volatile int running = 1;
static void sig_handler(int s) { (void)s; running = 0; }

int main() {
    signal(SIGINT, sig_handler);

    motor_hal_t *hal = motor_hal_create();
    motor_hal_init(hal, "can0", 1000000, 5000000);

    motor_config_t cfg1 = { .node_id=1, .auto_enable=true,
                            .disable_watchdog=true, .bootup_timeout_ms=5000 };
    motor_config_t cfg2 = { .node_id=2, .auto_enable=true,
                            .disable_watchdog=true, .bootup_timeout_ms=5000 };
    motor_hal_add_motor(hal, &cfg1);
    motor_hal_add_motor(hal, &cfg2);

    motor_hal_recv_start(hal);
    motor_hal_startup(hal, 1, 5000);
    motor_hal_startup(hal, 2, 5000);

    // 200Hz 控制循环 (~5ms)
    while (running) {
        // 读反馈缓存 (不触 SDO, RT 安全)
        motor_feedback_t fb1, fb2;
        motor_hal_get_feedback(hal, 1, &fb1);
        motor_hal_get_feedback(hal, 2, &fb2);

        // PDO 位置控制 (不触 SDO, RT 安全)
        motor_hal_set_position(hal, 1, 30.0f);
        motor_hal_set_position(hal, 2, -30.0f);

        usleep(5000);
    }

    motor_hal_destroy(hal);
    return 0;
}
```

### 13.3 C 上层示例 — 完整的 CAN/CANFD 标准 PDO 流程

```c
// 步骤1: 映射 TPDO (从站→主站, 反馈上报)
pdo_map_entry_cfg_t tpdo[] = {
    {0x6041, 0x00, 16},  // Statusword
    {0x6064, 0x00, 32},  // Position
    {0x6078, 0x00, 16},  // Current
};
motor_hal_pdo_map(hal, 1, tpdo, 3, 0, PDO_TYPE_TPDO, 0x181, 1);

// 步骤2: 映射 RPDO (主站→从站, 控制)
pdo_map_entry_cfg_t rpdo[] = {
    {0x6040, 0x00, 16},  // Controlword
    {0x607A, 0x00, 32},  // Target Position
};
motor_hal_pdo_map(hal, 1, rpdo, 2, 0, PDO_TYPE_RPDO, 0x201, 255);

// 步骤3: 启动 SYNC (触发 TPDO 上报)
motor_hal_sync_start(hal, 5000);  // 200Hz

// 步骤4: 控制循环 — 发 RPDO
while (running) {
    uint8_t data[6] = {0x0F,0x00, 0x00,0x40,0x00,0x00};  // CW=EnableOp, Pos=16384
    motor_hal_rpdo_send(hal, 1, data, 6);
    usleep(5000);
}
```

### 13.4 CLI 工作流示例

```bash
# 1. 启动守护进程
motor_tool daemon can0 &

# 2. 上电电机
motor_tool startup 1
motor_tool startup 2

# 3. SDO 控制 (低频命令)
motor_tool torque 1 500          # 电机1 500mA
motor_tool speed 1 5000          # 电机1 50RPM
motor_tool abs 1 4500            # 电机1 45°
motor_tool read all 0            # 读双电机全部信息

# 4. 持续监控反馈
motor_tool watch 200             # 200ms 周期显示

# 5. 数据上报 (每5ms, 类似 GD32 的 @CA)
motor_tool report 5              # 启动上报
motor_tool report 0              # 停止上报

# 6. 传感器
motor_tool sensor config 1 1     # 1KHz 透传
motor_tool sensor watch 1        # 持续显示传感器

# 7. 校准
motor_tool calib start 1 2       # 校准双关节

# 8. 停止
motor_tool stop                  # 关闭守护进程
```
