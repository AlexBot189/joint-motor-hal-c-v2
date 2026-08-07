# demo_algo 模式原理详解

调试日志: 2025-07-02

## 通用流程 (所有模式共用)

```
stark_open()         连接 SHM "/stark_shm"
stark_ready()        等校准完成 (calib_state==2)
stark_enable(1,2)    使能双电机, 写 PDO Byte0 bit7=1 (管理命令)
usleep(5000)         等 RT 线程处理
stark_set_mode(id,m) 通过 SDO 写电机 0x6060 寄存器切换控制模式 (管理命令)
usleep(5000)
[控制循环]           每周期写 SHM mailbox, RT 线程 1KHz 读取后发 PDO
stark_estop(1,2)     急停 (enable=0 + bus=OFF)
stark_close()         断开 SHM
```

分层示意:

```
算法进程 (demo_algo)           SHM                     RT 线程 1KHz              CANFD 总线
─────────────────────────────────────────────────────────────────────────────────────
stark_multi() 写 mailbox ──> seq_begin++ ──> ProcessMailbox 读 cmd[0],cmd[1]
                                              │
                                              ├─ MULTI/MIT/torque/speed/pos/pp/pv
                                              │    ┌─ 查 pdo_byte0 取 enable/mode 位
                                              │    ├─ 填 target1/target2/feedforward
                                              │    └─ pdo_ctrl_send / pdo_mit_send / pdo_multi_send
                                              │
                                              └─ ENABLE/DISABLE/SET_MODE/CLEAR_FAULT
                                                   └─ 修改 pdo_byte0 对应 bit
```

关键点: 管理命令修改 pdo_byte0, 控制命令使用 pdo_byte0 发帧。pdo_byte0 在控制命令里原地更新 mode 字段, 不需要每次都设模式。

---

## 1. torque 电流环

```
./demo_algo torque 200
```

### 流程

main() 设置:
```
stark_set_mode(1, 5)    // SDO 写 0x6060 = 0x05 (巨蟹电流环)
stark_set_mode(2, 5)
```

run_torque() 循环:
```
每个周期 (1ms):
  ma = 200 * sin(t / 2000ms * 2π)    // ±200mA 正弦波, 周期 2s
  stark_multi(c, ma, 0, 0, ma, 0, 0) // 双电机同相同幅电流
```

### CAN 帧

stark_multi 走多轴广播路径:

```
CAN ID: 0x200 (多轴广播 PDO)
帧类型: CANFD 64B
Byte0: Enable=1  Bus=1  Mode=Current(0x05)
Byte1-2: target1 = ma (int16, mA)       // Q轴电流目标
Byte3-4: target2 = 0
Byte5-6: feedforward = 0
...
电机1 和电机2 的 8 字节命令打包在同一帧
```

### 驱动板行为

驱动板收到 PDO 0x200, 解析出电流目标值, 直接写 target_iq (0x6071), 电流环 PID 驱动 PWM。

### 特点

- 最底层控制, SOC 直接控制 Q 轴电流
- 没有位置/速度闭环在驱动板, 位置环完全由上层算法负责
- 力矩响应最快, 适合步态控制 (算法算出力矩目标, 驱动板只负责跟踪电流)

---

## 2. multi 多轴广播恒电流

```
./demo_algo multi 200 200
```

### 流程

main() 设置:
```
stark_set_mode(1, 5)     // 和 torque 一样, 电流模式
stark_set_mode(2, 5)
```

run_multi() 循环:
```
每个周期 (1ms):
  stark_multi(c, 200, 0, 0, 200, 0, 0)   // 两电机恒定 200mA
```

### CAN 帧

```
和 torque 完全相同的 PDO 0x200 多轴广播帧
区别: target1 恒为 200, 不做正弦变化
```

### 与 torque 的区别

| 维度 | torque | multi |
|------|--------|-------|
| 目标值变化 | 正弦波 ±200mA | 恒定 200mA |
| mode 值 | 5 (Current) | 5 (Current) |
| CAN 路径 | PDO 0x200 | PDO 0x200 |
| 用途 | 测试电流跟踪 | 恒定力矩保持/省力模式 |

底层走完全相同的代码路径, 只是算法层发的值不同。

---

## 3. CSP 循环同步位置

```
./demo_algo csp 15
```

### 流程

main() 设置:
```
stark_set_mode(1, 3)     // SDO 写 0x6060 = 0x03 (CSP)
stark_set_mode(2, 3)
```

run_position() 循环:
```
每个周期 (1ms):
  motor1 目标: ±15° 方波, 每 2s 翻转一次
  motor2 目标: -motor1 目标 (对称运动)
  stark_position(c, 1, target)    // 内部填 cmd=POW, value=deg*100
  stark_position(c, 2, -target)

RT 线程处理:
  cmd == STARK_CMD_POS
  motor_hal_set_position(hal, mid, value/100.0f)
```

### motor_hal_set_position 内部

```
1. 角度转编码器 counts: counts = deg * 65536 / 360
2. 从 pdo_byte0 取 enable/brake 位, 置换 mode 为 CSP
3. pdo_ctrl_send_raw(drv, node, byte0, counts, accel, 0)
```

### CAN 帧

```
CAN ID:  0x101 (电机1), 0x102 (电机2)      // 单轴 PDO = 0x100 + ID
帧类型:  CANFD 7B
Byte0:   Enable=1  Bus=1  Mode=CSP(0x03)
Byte1-2: target1 = position (int16, counts)
Byte3-4: target2 = profile_accel (uint16, RPM/s)
Byte5-6: feedforward = 0
```

### 驱动板行为

- 收到 PDO 帧存储 target_position
- 收到 SYNC 帧时, 立即用当前 target_position 执行位置闭环
- RT 线程 1KHz 发 SYNC, 驱动板 1KHz 执行位置跟踪

### 特点

- 位置闭环在驱动板内部完成 (位置 PID + 速度 PID + 电流 PID 三级串级)
- SOC 只发目标位置, 不关心中间过程
- 刚性位置控制, 适合需要精确角度的场景
- SYNC 同步触发保证所有电机同一时刻执行

---

## 4. CSV 循环同步速度

```
./demo_algo csv 10
```

### 流程

main() 设置:
```
stark_set_mode(1, 4)     // SDO 写 0x6060 = 0x04 (CSV)
stark_set_mode(2, 4)
```

run_speed() 循环 (每 5ms):
```
梯形波: 0→+10RPM (1s加速) → +10RPM (1s恒速) → 0 (1s减速) → -10RPM (1s恒速)
stark_speed(c, 1, rpm)     // 内部填 cmd=SPEED, value=rpm*100
stark_speed(c, 2, rpm)

RT 线程处理:
  cmd == STARK_CMD_SPEED
  motor_hal_set_velocity(hal, mid, value/100.0f)
```

### motor_hal_set_velocity 内部

```
1. 从 pdo_byte0 取 enable/brake 位, 置换 mode 为 CSV
2. pdo_ctrl_send_raw(drv, node, byte0, (int16_t)rpm, accel, 0)
   // target1 = 速度 RPM, target2 = 加速度配置值
```

### CAN 帧

```
CAN ID:  0x101 / 0x102 (单轴 PDO)
Byte0:   Enable=1  Bus=1  Mode=CSV(0x04)
Byte1-2: target1 = velocity (int16, RPM)
Byte3-4: target2 = profile_accel (uint16)
Byte5-6: feedforward = 0
```

### 驱动板行为

- 收到 PDO 帧存储 target_velocity
- 收到 SYNC 帧时执行速度闭环
- 速度闭环在驱动板内部 (速度 PID + 电流 PID)

### 特点

- 和 CSP 对称, 只是闭环的层级不同 (速度环 vs 位置环)
- 同步触发, 多电机速度同步好
- **注意: 驱动板 CSV 不做加减速平滑, 速度指令直接生效**

---

## 5. PP 轮廓位置

```
./demo_algo pp 15 2000 10
```

### 流程

main() 设置:
```
stark_set_mode(1, 1)     // SDO 写 0x6060 = 0x01 (PP)
stark_set_mode(2, 1)
```

run_pp() 循环 (每 1ms):
```
target = ±15° 方波, 2s/拍
motor1: stark_pp(c, 1, target, 2000, 10)
  // deg=±15°, accel=2000RPM/s, vel=10RPM
motor2: stark_pp(c, 2, -target, 2000, 10)

RT 线程处理:
  cmd == STARK_CMD_PP
  motor_hal_ctrl_raw(hal, mid, MOTOR_MODE_PROFILE_POS,
                     (int16_t)(deg*100), (uint16_t)(accel*100), (int16_t)(vel*100))
```

### motor_hal_ctrl_raw 内部

```
1. 从 pdo_byte0 取 enable/brake, 置换 mode 为 PP
2. pdo_ctrl_send_raw(drv, node, byte0, target1, target2, feedforward)
   // target1 = 位置(0.01°), target2 = 加速度(0.01 RPM/s), feedforward = 轮廓速度(0.01 RPM)
```

### CAN 帧

```
和 CSP 相同的 0x100+ID 单轴 PDO 帧格式
区别: Byte0 mode=PP(0x01), 且 Byte3-4 和 5-6 有意义
```

### 驱动板行为

- 收到 PDO 帧, 解析 target_position、profile_accel、profile_velocity
- 驱动板**自己**做梯形速度规划:
  1. 从当前位置出发
  2. 以 profile_accel 加速到 profile_velocity
  3. 匀速运动
  4. 以 profile_accel 减速到目标位置
- SOC 不需要逐周期更新位置, 发一次目标就行

### 与 CSP 的区别

| 维度 | CSP | PP |
|------|-----|-----|
| 轨迹规划 | SOC 上层做 | 驱动板做 |
| 加减速控制 | SOC 逐周期算中间位置 | 驱动板自动梯形规划 |
| SOC 负载 | 高 (每周期算位置) | 低 (只需发终点) |
| 适用场景 | 动态轨迹 (步态) | 点到点运动 (摆臂/归位) |

### 注意

demo_algo 里 PP 每 1ms 都在发同样的目标位置 (同一个方波平台期持续 2s), 这是因为 demo 写得简单, 实际用 PP 时只在目标切换时发一次就行。

---

## 6. PV 轮廓速度

```
./demo_algo pv 30 1000
```

### 流程

main() 设置:
```
stark_set_mode(1, 2)     // SDO 写 0x6060 = 0x02 (PV)
stark_set_mode(2, 2)
```

run_pv() 循环 (每 5ms):
```
梯形波: 0→30RPM→30→0→-30 RPM, 每段 1s
motor1: stark_pv(c, 1, rpm, 1000)
  // rpm=当前速度, accel=1000RPM/s
motor2: stark_pv(c, 2, -rpm, 1000)   // 反向对称

RT 线程处理:
  cmd == STARK_CMD_PV
  motor_hal_ctrl_raw(hal, mid, MOTOR_MODE_PROFILE_VEL,
                     (int16_t)(rpm), (uint16_t)(accel), 0)
```

### CAN 帧

```
CAN ID:  0x101 / 0x102 (单轴 PDO)
Byte0:   Enable=1  Bus=1  Mode=PV(0x02)
Byte1-2: target1 = velocity (int16, RPM)
Byte3-4: target2 = accel (uint16, RPM/s)
Byte5-6: feedforward = 0
```

### 驱动板行为

- 收到 PDO 帧, 解析 target_velocity 和 profile_accel
- 驱动板自己做加减速斜坡: 从当前速度以 accel 匀变到目标速度
- **不需要 SYNC 触发 (PP/PV 是非同步模式)**

### 与 CSV 的区别

| 维度 | CSV | PV |
|------|-----|-----|
| 加减速 | 无 (速度指令直接生效) | 有 (驱动板做斜坡) |
| 同步触发 | 需要 SYNC | 不需要 |
| 多电机同步 | 好 (SYNC 保证) | 各自独立 |
| 速度突变 | 可能抖 | 平滑过渡 |
| 适用场景 | 需要精确速度同步 | 需要平滑变速 |

---

## 7. MIT 阻抗控制

```
./demo_algo mit 100 50
```

### 流程

main() 设置:
```
stark_set_mode(1, 6)     // SDO 写 0x6060 = 0x06 (MIT)
stark_set_mode(2, 6)
```

run_mit() 循环 (每 1ms):
```
目标位置=0° (当前位置, 即零力点), kp=100, kd=50, 前馈力矩=0

stark_mit(c, 1, 0.0f, 0.0f, 100.0f, 50.0f, 0.0f)
stark_mit(c, 2, 0.0f, 0.0f, 100.0f, 50.0f, 0.0f)

RT 线程处理:
  cmd == STARK_CMD_MIT
  motor_hal_mit_control(hal, mid, pos_deg, vel_rpm, kp, kd, torque_ma)
```

### motor_hal_mit_control 内部

```
位置: pos = (deg + 180) / 360 * 65535        // uint16 编码
速度: vel = (uint16)((int16_t)vel_rpm)
kp:   kp_v = kp * 100                         // 无量纲 ×100
kd:   kd_v = kd * 100
力矩: torq = torque_ma

pdo_mit_send_raw(drv, node_id, byte0, pos, vel, kp_v, kd_v, torq)
```

### CAN 帧 (MIT 专用 PDO)

```
CAN ID:  0x111 (电机1), 0x112 (电机2)     // MIT PDO = 0x110 + ID
帧类型:  CANFD 12B
Byte0:   Enable=1  Bus=1  Mode=MIT(0x06)
Byte1-2: position (uint16, [0..65535])
Byte3-4: velocity (uint16)
Byte5-6: kp (uint16, ×100)
Byte7-8: kd (uint16, ×100)
Byte9-10: torque (int16)
Byte11:  reserved
```

### 驱动板行为

驱动板内部执行阻抗控制律:

```
tau = kp * (target_position - actual_position) / 100
    + kd * (target_velocity - actual_velocity) / 100
    + feedforward_torque
```

驱动板用算出的 tau 作为电流环目标, 不需要外部算力矩。

### 特点

- 驱动板内置阻抗解算, SOC 发位置+刚度阻尼参数
- kp 越大越"硬", kp 越小越"软"
- kd 提供阻尼, 防止振荡
- 零力模式: kp=50, target_pos=当前角度, 用手可以推动电机
- 高刚性模式: kp=500, target_pos=目标角度, 电机"锁死"在目标位置

### 参数调节经验

| kp | kd | 效果 |
|-----|-----|------|
| 0 | 0 | 无力输出 (等同 torque=0) |
| 50 | 20 | 很软, 容易推动 |
| 100 | 50 | 中等, 适合起步测试 |
| 300 | 100 | 较硬, 有弹力感 |
| 500 | 200 | 很硬, 接近纯位置控制 |

---

## 模式总览对照表

| 模式 | mode值 | stark API | CAN帧 | COB-ID | 数据字段 | 闭环位置 | 同步方式 |
|------|--------|-----------|-------|--------|----------|----------|----------|
| torque | 5 | stark_multi | 多轴广播 | 0x200 | t1=电流 | 驱动板电流环 | 无 |
| multi | 5 | stark_multi | 多轴广播 | 0x200 | t1=电流 | 驱动板电流环 | 无 |
| CSP | 3 | stark_position | 单轴PDO | 0x100+ID | t1=位置 | 驱动板三级串级 | SYNC触发 |
| CSV | 4 | stark_speed/csv | 单轴PDO | 0x100+ID | t1=速度 | 驱动板速度串级 | SYNC触发 |
| PP | 1 | stark_pp | 单轴PDO | 0x100+ID | t1=位置,t2=accel,ff=vel | 驱动板规划 | 非同步 |
| PV | 2 | stark_pv | 单轴PDO | 0x100+ID | t1=速度,t2=accel | 驱动板规划 | 非同步 |
| MIT | 6 | stark_mit | MIT PDO | 0x110+ID | pos/vel/kp/kd/tor | 驱动板阻抗解算 | 无 |

## 双电机命令方式对比

| 方式 | 帧数 | 同步性 | 推荐场景 |
|------|------|--------|----------|
| stark_multi (多轴广播) | 1帧 64B | 最好 (同一帧) | 实时步态控制 |
| stark_xxx 逐电机发 | 2帧 7B/12B | 差 1 个 CAN 周期 | 单电机调试 |
