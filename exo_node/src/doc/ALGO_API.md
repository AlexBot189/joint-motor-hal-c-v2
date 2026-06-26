# 外骨骼电机控制协议 — 上层算法调用接口

## 1. 数据通道

```
算法进程 ──(SHM mailbox)──→ motor_node (RT线程) ──(CANFD)──→ 驱动板
         ←──(SHM fb_buffer)──────────────────────────────────── 反馈
```

- **发出**: 写 `exo_shm_t.mailbox.cmd[0]` / `cmd[1]`
- **读回**: 读 `exo_shm_t.fb_buffer[active].motor[0]` / `motor[1]`
- **握手**: 写 `seq_begin++` → 写 `seq_end = seq_begin`（motor_node 通过 seq 变化感知新命令）

## 2. 控制流程

```
SDO startup (motor_node自动完成)
    │
    ▼
算法发 ENABLE        → PDO Byte0 bit7=1  电机开始响应PDO
算法发 SET_MODE      → 选择控制模式
    │
    ▼
┌─────────────────────────────┐
│  控制循环 (1KHz)            │
│  发 TORQUE/POS/SPEED/MIT/PP │  ← 只传 target 值
└─────────────────────────────┘
    │
    ▼
急停:  ESTOP  → PDO Byte0 bit7=0 + bit6=0
恢复:  RECOVER → PDO Byte0 bit7=1 + bit6=1
```

## 3. motor_command_t 结构

```c
typedef struct {
    uint8_t  motor_id;     // 1=右髋  2=左髋
    uint8_t  cmd;          // 命令类型 (见下表)
    int32_t  value;        // target1: 主目标值
    int32_t  value2;       // target2: 加减速 (PP/PV模式)
    int32_t  feedforward;  // 前馈 (PP模式轮廓速度)
    uint16_t mit_pos;      // MIT: 目标位置 [0-65535]
    uint16_t mit_vel;      // MIT: 目标速度 [0-4095]
    uint16_t mit_kp;       // MIT: 位置刚度 [0-4095]
    uint16_t mit_kd;       // MIT: 速度阻尼 [0-4095]
    uint16_t mit_torque;   // MIT: 前馈力矩 [0-4095]
    uint8_t  _pad[5];
    uint64_t timestamp_us; // 算法下发时刻
} motor_command_t;
```

## 4. 命令一览

### 4.1 控制命令（控制循环每帧发送）

| 命令 | 值 | value | value2 | feedforward | MIT字段 | 说明 |
|------|---|-------|--------|-------------|---------|------|
| `EXO_CMD_TORQUE` | 1 | mA | — | — | — | 电流/力矩模式 |
| `EXO_CMD_SPEED` | 2 | RPM×100 | — | — | — | 轮廓速度 PV |
| `EXO_CMD_POS` | 3 | °×100 | — | — | — | 循环同步位置 CSP |
| `EXO_CMD_MIT` | 4 | — | — | — | ✅ | MIT 阻抗控制 |
| `EXO_CMD_PP` | 5 | °×100 | RPM/s | RPM | — | 轮廓位置 PP |
| `EXO_CMD_CSV` | 6 | RPM×100 | — | — | — | 循环同步速度 CSV |
| `EXO_CMD_MULTI` | 7 | 取决于mode | RPM/s | RPM | — | 多轴广播帧 (一帧发完双电机) |

### 4.2 Byte0 控制命令（改 PDO 状态，不发 target）

| 命令 | 值 | value | 说明 |
|------|---|-------|------|
| `EXO_CMD_ENABLE` | 10 | — | PDO使能 (Byte0 bit7=1) |
| `EXO_CMD_DISABLE` | 11 | — | PDO失能 (Byte0 bit7=0) |
| `EXO_CMD_ESTOP` | 12 | — | 急停: enable=0 + bus=OFF |
| `EXO_CMD_RECOVER` | 13 | — | 恢复: bus=ON + enable=1 |
| `EXO_CMD_SET_MODE` | 14 | mode值 | 切换PDO控制模式 (1=PP 2=PV 3=CSP 4=CSV 5=电流 6=MIT) |
| `EXO_CMD_CLEAR_FAULT` | 15 | — | 清错脉冲 (Byte0 bit5, 下一帧自动清除) |

## 5. 使用示例

### 5.1 初始化

```c
#include "exo_shm.h"

// 等待 motor_node 完成 SDO startup (DS402 → Operation Enabled)
while (shm->node_state < STATE_ENABLED) { usleep(10000); }

// PDO 使能 + 设模式 (发一次即可)
cmd_set(shm, 0, 1, EXO_CMD_ENABLE,   0, 0, 0, ts);
cmd_set(shm, 1, 2, EXO_CMD_ENABLE,   0, 0, 0, ts);
publish(shm);  // seq_begin++ → seq_end
usleep(5000);

cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, 5, 0, 0, ts);  // 电流模式
cmd_set(shm, 1, 2, EXO_CMD_SET_MODE, 5, 0, 0, ts);
publish(shm);
```

### 5.2 控制循环 — 力矩模式

```c
while (running) {
    read_feedback(shm);  // 读 fb_buffer

    // 只传 target, Byte0 由 motor_node 内部维护
    cmd_set(shm, 0, 1, EXO_CMD_TORQUE, 500, 0, 0, ts);  // 电机1: 500mA
    cmd_set(shm, 1, 2, EXO_CMD_TORQUE, 300, 0, 0, ts);  // 电机2: 300mA
    publish(shm);

    usleep(1000);  // 1KHz
}
```

### 5.3 位置模式 CSP — 正弦摆动

```c
/* 先切模式 (只发一次) */
cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, 3, 0, 0, ts);  // CSP mode=3
cmd_set(shm, 1, 2, EXO_CMD_SET_MODE, 3, 0, 0, ts);
publish(shm);
usleep(5000);

/* 控制循环 */
while (1) {
    float angle = 15.0f * sinf(t * 2.0f * M_PI);
    cmd_set(shm, 0, 1, EXO_CMD_POS, (int32_t)(angle * 100), 0, 0, ts);
    cmd_set(shm, 1, 2, EXO_CMD_POS, (int32_t)(-angle * 100), 0, 0, ts);
    publish(shm);
    usleep(1000);
}
```

### 5.4 MIT 阻抗控制 — 柔顺模式

```c
cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, 6, 0, 0, ts);  // MIT mode=6
cmd_set(shm, 1, 2, EXO_CMD_SET_MODE, 6, 0, 0, ts);
publish(shm);
usleep(5000);

while (1) {
    shm->mailbox.cmd[0].cmd      = EXO_CMD_MIT;
    shm->mailbox.cmd[0].mit_pos  = 32768;   // 中间位置
    shm->mailbox.cmd[0].mit_vel  = 0;
    shm->mailbox.cmd[0].mit_kp   = 50;      // 低刚度 (可被人推动)
    shm->mailbox.cmd[0].mit_kd   = 5;
    shm->mailbox.cmd[0].mit_torque = 0;
    // cmd[1] 同理
    publish(shm);
    usleep(1000);
}
```

### 5.5 PP 轮廓位置 — 带加减速

```c
cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, 1, 0, 0, ts);  // PP mode=1
publish(shm);
usleep(5000);

// value=45°, value2=2000RPM/s(加减速), feedforward=500RPM(轮廓速度)
cmd_set(shm, 0, 1, EXO_CMD_PP, 4500, 2000, 500, ts);
publish(shm);
```

### 5.6 多轴广播

```c
/* cmd[0]和cmd[1]都设 MULTI → RT线程自动打包一帧64B CANFD */
cmd_set(shm, 0, 1, EXO_CMD_MULTI, 500, 0, 0, ts);  // 电机1: 500mA
cmd_set(shm, 1, 2, EXO_CMD_MULTI, 300, 0, 0, ts);  // 电机2: 300mA
publish(shm);
```

### 5.7 急停/恢复

```c
/* 检测到异常 → 急停 */
cmd_set(shm, 0, 1, EXO_CMD_ESTOP, 0, 0, 0, ts);
cmd_set(shm, 1, 2, EXO_CMD_ESTOP, 0, 0, 0, ts);
publish(shm);

/* 恢复 */
cmd_set(shm, 0, 1, EXO_CMD_RECOVER, 0, 0, 0, ts);
cmd_set(shm, 1, 2, EXO_CMD_RECOVER, 0, 0, 0, ts);
publish(shm);
```

### 5.8 bit5 清错

```c
/* 读错误码 */
read_feedback(shm);
uint8_t err = shm->fb_buffer[active].motor[0].error_code;

/* 判断错误类型, 决定是否清除 */
if (err == 0x04) {  // 过温
    // 等降温, 不清除
} else if (err == 0x08) {  // 堵转
    // 先失能, 再清除
    cmd_set(shm, 0, 1, EXO_CMD_DISABLE, 0, 0, 0, ts);
    publish(shm);
    usleep(10000);
    cmd_set(shm, 0, 1, EXO_CMD_CLEAR_FAULT, 0, 0, 0, ts);
    publish(shm);
} else if (err != 0) {  // 其他错误
    cmd_set(shm, 0, 1, EXO_CMD_CLEAR_FAULT, 0, 0, 0, ts);
    publish(shm);
}

## 6. 辅助宏

```c
/* 快捷填充 cmd */
#define CMD_SET(shm, idx, id, cmd, val, val2, ff, ts)  do { \
    shm->mailbox.cmd[idx].motor_id     = id;  \
    shm->mailbox.cmd[idx].cmd          = (uint8_t)(cmd); \
    shm->mailbox.cmd[idx].value        = val; \
    shm->mailbox.cmd[idx].value2       = val2; \
    shm->mailbox.cmd[idx].feedforward  = ff;  \
    shm->mailbox.cmd[idx].mit_pos      = 0;   \
    shm->mailbox.cmd[idx].mit_vel      = 0;   \
    shm->mailbox.cmd[idx].mit_kp       = 0;   \
    shm->mailbox.cmd[idx].mit_kd       = 0;   \
    shm->mailbox.cmd[idx].mit_torque   = 0;   \
    shm->mailbox.cmd[idx].timestamp_us = ts;  \
} while(0)

/* 发布命令 (seq握手) */
#define PUBLISH(shm)  do { \
    (shm)->mailbox.seq_begin++; \
    (shm)->mailbox.seq_end = (shm)->mailbox.seq_begin; \
} while(0)
```

## 7. 反馈读取

```c
/* 读双Buffer反馈 */
uint32_t active = shm->active_idx;  // atomic acquire
feedback_frame_t* fb = &shm->fb_buffer[active];

// 电机数据
fb->motor[0].position    // encoder counts [-32768,32767]
fb->motor[0].velocity    // RPM
fb->motor[0].current_iq  // mA
fb->motor[0].temperature // 0.1°C
fb->motor[0].error_code  // 故障码
fb->motor[0].status_byte // bit7=使能 bit6=抱闸 bit5=错误

// SHM 状态
shm->motor_online   // bit0=电机1在线 bit1=电机2在线
shm->node_state     // exo_state_t
shm->motor_severity // 0=OK 1=WARN 2=FAULT
```
