# motor_hal_c + motor_tool 通信闭环验收计划

**环境**: RV1126B + 巨蟹驱动板 ×1 (CANFD) | **测试阶段**: 先无电机，后接电机

---

## 目录

- [Part A: 没接电机 —— CANFD 协议层验收](#part-a)
  - [A1: motor_tool CLI 单步测试 (53项)](#a1)
  - [A2: HAL 并发压力测试 demo (12项)](#a2)
- [Part B: 接电机后 —— 物理层验收](#part-b)
  - [B1: 单电机基础验证 (10项)](#b1)
  - [B2: 双电机同步验证 (8项)](#b2)
  - [B3: 全控制模式验证 (14项)](#b3)
  - [B4: 传感器透传数据验证 (5项)](#b4)
  - [B5: 校准流程验证 (5项)](#b5)
  - [B6: 故障/边界验证 (6项)](#b6)
- [Part C: 上层完整控制项映射表](#part-c)
- [验收通过标准](#验收通过标准)
- [附录: 编译与运行](#附录)

---

## <a name="part-a"></a>Part A: 没接电机 —— CANFD 协议层验收

目标: 只接驱动板 (ID=1)，验证 HAL 所有协议路径无逻辑 bug。

---

### <a name="a1"></a>A1: motor_tool CLI 单步测试 (53项)

```bash
# 环境准备
pkill motor_tool 2>/dev/null
rm -f /dev/shm/exo_shm
motor_tool daemon can0 &
sleep 2
```

**验收关键**: 每步对比 `candump can0 -x` 输出，验证帧格式。

#### A1.0 环境检查 (2项)

| # | 命令 | 验证点 | 预期 |
|---|------|--------|------|
| A1.0.1 | `motor_tool daemon can0 &` | daemon 启动 | 无报错，打印 `✓ CANFD can0 opened` |
| A1.0.2 | `candump can0 -x` | CAN 总线有帧 | 能看到 Heartbeat / Bootup 帧 |

#### A1.1 SDO 读 (7项)

| # | 命令 | 验证点 | 预期 |
|---|------|--------|------|
| A1.1.1 | `motor_tool sdoread 1 0x6041` | Statusword | 返回合法值 (如 0x0437) |
| A1.1.2 | `motor_tool sdoread 1 0x100A` | 固件版本 | 返回 32bit 版本号 |
| A1.1.3 | `motor_tool sdoread 1 0x6061` | 当前运行模式 | 返回 0x00~0x0A |
| A1.1.4 | `motor_tool sdoread 1 0x1017` | 心跳周期 | daemon 配置的值 |
| A1.1.5 | `motor_tool sdoread 1 0x6060` | 控制模式 | 返回合法值 |
| A1.1.6 | `motor_tool sdoread 1 0x6064` | 位置原始值 | 返回 int32 |
| A1.1.7 | `motor_tool sdoread 1 0x606C` | 速度原始值 | 返回 int32 (应为 0) |

#### A1.2 SDO 写+回读 (4项)

| # | 命令 | 验证 |
|---|------|------|
| A1.2.1 | `motor_tool sdowrite 1 0x1017 0 2000 2` | 写心跳周期 |
| A1.2.2 | `motor_tool sdoread 1 0x1017` | 回读=0x07D0 (2000) |
| A1.2.3 | `motor_tool sdowrite 1 0x2650 0 1 4` | 关看门狗 |
| A1.2.4 | `motor_tool sdoread 1 0x2650` | 回读=1 |

#### A1.3 DS402 状态机 (5项)

| # | 命令 | 验证 |
|---|------|------|
| A1.3.1 | `motor_tool read state 1` | 查询当前状态 |
| A1.3.2 | `motor_tool enable 1` | Shutdown→SwitchOn→EnableOp |
| A1.3.3 | `motor_tool read state 1` | 显示 `OPERATION_ENABLED` |
| A1.3.4 | `motor_tool disable 1 && motor_tool enable 1` | disable/enable 循环 ×3 |
| A1.3.5 | `motor_tool fault_reset 1` | SDO 0x6040=0x80 |

#### A1.4 SDO 全量读取 (9项)

| # | 命令 | 验证 |
|---|------|------|
| A1.4.1 | `motor_tool read all 1` | 一次输出全部信息 |
| A1.4.2 | `motor_tool read angle 1` | 编码器角度 |
| A1.4.3 | `motor_tool read speed 1` | 速度 RPM |
| A1.4.4 | `motor_tool read current 1` | 电流 mA |
| A1.4.5 | `motor_tool read temp 1` | 温度 °C |
| A1.4.6 | `motor_tool read error 1` | 故障码 (应为 0x0000) |
| A1.4.7 | `motor_tool read version 1` | 固件版本 |
| A1.4.8 | `motor_tool read mode 1` | 运行模式 |
| A1.4.9 | `motor_tool read voltage 1` | 母线电压 (若已实现) |

#### A1.5 SDO 控制命令 (安全值) (6项)

| # | 命令 | 验证 |
|---|------|------|
| A1.5.1 | `motor_tool torque 1 0` | 自动切电流模式, 0mA |
| A1.5.2 | `motor_tool torque 1 100` | 100mA — 无负载不转 |
| A1.5.3 | `motor_tool speed 1 0` | 自动切 PV 模式, 0 RPM |
| A1.5.4 | `motor_tool speed 1 50` | 50 RPM (默认 accel=1000) |
| A1.5.5 | `motor_tool abs 1 0` | 自动切 PP 模式, 目标 0° |
| A1.5.6 | `motor_tool abs 1 45` | 目标 45° |

> 注: `mode`/`accel` 不是独立命令。模式由 torque/speed/abs 隐式切换, 加减速默认 1000 RPM/s 可通过 `abs_accel` 全局设置。

#### A1.6 PDO Byte0 控制 (7项)

| # | 命令 | 验证 |
|---|------|------|
| A1.6.1 | `motor_tool pdo_enable 1` | bit7=1 |
| A1.6.2 | `motor_tool pdo_disable 1` | bit7=0 |
| A1.6.3 | `motor_tool byte0 1` | 显示 byte0 值 |
| A1.6.4 | `motor_tool estop 1` | Byte0→0x00 |
| A1.6.5 | `motor_tool recover 1` | Byte0→0xC0 |
| A1.6.6 | `motor_tool setmode 1 5` | PDO mode=电流 (5) |
| A1.6.7 | `motor_tool setmode 1 3` | PDO mode=CSP (3) |

> 注: setmode 接受数字 1~6 (1=PP, 2=PV, 3=CSP, 4=CSV, 5=CURRENT, 6=MIT)。Byte0 命令只改内存不触发 CAN 帧，帧在下一次控制命令时发出。

#### A1.7 PDO 实时控制帧 (6项)

| # | 命令 | 验证 (candump 对照) |
|---|------|---------------------|
| A1.7.1 | `motor_tool pdo 1 pos 0` | COB 0x101 7B 帧 |
| A1.7.2 | `motor_tool pdo 1 vel 0` | COB 0x101 7B 帧 |
| A1.7.3 | `motor_tool pdo 1 cur 0` | COB 0x101 7B 帧 |
| A1.7.4 | `motor_tool pdo 1 csp 500` | COB 0x101, target=500 |
| A1.7.5 | `motor_tool mit 1 32768 0 100 10 0` | COB 0x111 9B 帧 |
| A1.7.6 | `motor_tool stop 1` | DS402 停位 |

#### A1.8 多轴广播 (3项)

| # | 命令 | 验证 (candump) |
|---|------|----------------|
| A1.8.1 | `motor_tool multi cur 1:0 2:0` | COB 0x200 64B |
| A1.8.2 | `motor_tool multi pos 1:0 2:0` | COB 0x200 |
| A1.8.3 | `motor_tool multi vel 1:0 2:0` | COB 0x200 |

#### A1.9 反馈接收 (2项)

| # | 命令 | 验证 |
|---|------|------|
| A1.9.1 | `motor_tool watch 100` | 持续输出反馈 (Ctrl+C退出) |
| A1.9.2 | `motor_tool report 5` | 5ms 周期上报 (Ctrl+C退出) |

#### A1.10 传感器透传 (5项)

| # | 命令 | 验证 |
|---|------|------|
| A1.10.1 | `motor_tool sensor config 1 4000` | 配置 250Hz |
| A1.10.2 | `motor_tool sensor read 1` | 读缓存数据 |
| A1.10.3 | `motor_tool sensor watch 1` | 持续显示 (Ctrl+C退出) |
| A1.10.4 | `motor_tool sensor stop 1` | 停止透传 |
| A1.10.5 | `motor_tool sensor config 1 1000` | 重新配置 1000Hz |

#### A1.11 持久化 & 配置 (5项)

| # | 命令 | 验证 |
|---|------|------|
| A1.11.1 | `motor_tool pid 1 100 10 200 20 50 5` | 设置 PID |
| A1.11.2 | `motor_tool read pid 1` | 回读验证 |
| A1.11.3 | `motor_tool save 1` | Flash 保存 |
| A1.11.4 | `motor_tool setzero 1` | 设零点 |
| A1.11.5 | `motor_tool limit_pos 1 4500` | 正限位 +45° |

#### A1.12 异常处理 (3项)

| # | 命令 | 验证 |
|---|------|------|
| A1.12.1 | `motor_tool sdoread 2 0x6041` | 超时，不卡死 |
| A1.12.2 | `motor_tool sdoread 1 0xFFFF` | SDO Abort，有错误码 |
| A1.12.3 | `motor_tool reboot 1` | NMT Reset → 等 3s 重新上线 |

---

**A1 验收标准**: 53项全部执行通过，无超时无崩溃，candump 帧格式正确。
**预计耗时**: 15~20分钟

---

### <a name="a2"></a>A2: HAL 并发压力测试 (hal_stress_test)

**位置**: `test/hal_stress_test.c`
**用途**: 覆盖 motor_tool CLI 无法测试的并发/线程安全路径

```bash
# 先停掉 motor_tool daemon (避免 CAN 总线冲突)
pkill motor_tool

# 编译
cd build && cmake .. && make hal_stress_test

# 运行 (需要 root / CAP_NET_ADMIN)
sudo ./hal_stress_test
```

| # | 测试 | 覆盖路径 | 验收标准 |
|---|------|---------|---------|
| T0 | CANFD 初始化+注册 | can_driver / motor_hal_init | CANFD 打开 OK |
| T1 | SDO 基础读写 | sdo_client 同步读写+回读验证 | Statusword/固件版本可读，心跳可写可回读 |
| T2 | SDO 并发压力 | **SDO 队列+condvar 多线程安全** | 4线程×100次=400次，0 超时 0 失败 |
| T3 | DS402 状态机 | 完整启动/使能/脱使能 | enable 成功，disable/enable×3 循环 OK |
| T4 | PDO Byte0 控制 | pdo_handler 所有 Byte0 API | enable/disable/bus/estop/recover/setmode OK |
| T5 | PDO 控制帧压力 | **PDO 帧快速切换 500 次** | 0 失败 |
| T6 | 多轴广播 | multi_ctrl 帧构造 | 电流/CSP/速度 三种广播 OK |
| T7 | MIT 阻抗控制 | MIT PDO 帧构造 | 帧发送 OK |
| T8 | TPDO 同步上报 | **SYNC→TPDO 链路** | 收到 TPDO 帧 |
| T9 | 传感器透传 | sensor_config + 回调 | 配置 OK，回调触发 |
| T10 | SDO 超时重试 | **离线电机 timeout→在线电机恢复** | 超时不卡死，后续 SDO 正常 |
| T11 | 反馈持续接收 | **recv 线程稳定性** | 3 秒持续收到反馈帧，不崩溃不丢帧 |

**A2 验收标准**: 输出 `RESULT: 11/11 passed`，无 timeout 无 deadlock。

---

## <a name="part-b"></a>Part B: 接电机后 —— 物理层验收

**前提**: A1+A2 全部通过 → CANFD 协议层无 bug

**环境**: RV1126B + 巨蟹驱动板 ×2 (ID=1,2) + **左右实体电机**

---

### <a name="b1"></a>B1: 单电机基础验证 (10项)

| # | 测试步骤 | 验证点 | 验收标准 |
|---|---------|--------|---------|
| B1.1 | 上电等 bootup → `motor_tool read state 1` | DS402 自动使能 | OPERATION_ENABLED |
| B1.2 | `motor_tool read angle 1` | 编码器读数 | 合理角度值，非 0x7FFF |
| B1.3 | `motor_tool torque 1 500` | 小电流控制 | 电机有微弱力矩输出 (用手感知) |
| B1.4 | `motor_tool torque 1 0` | 零力矩 | 电机无力 |
| B1.5 | `motor_tool speed 1 500` (50RPM) | 速度控制 | 电机匀速旋转 |
| B1.6 | `motor_tool read speed 1` | 速度反馈 | 接近 50RPM |
| B1.7 | `motor_tool speed 1 0 && motor_tool stop 1` | 停止 | 电机停转 |
| B1.8 | `motor_tool abs 1 4500` (45°) | 位置控制 | 电机转到 ~45° |
| B1.9 | `motor_tool read angle 1` | 角度回读 | 接近设的目标 |
| B1.10 | `motor_tool abs 1 -4500` (-45°) | 反向位置 | 电机反向转到 ~-45° |

---

### <a name="b2"></a>B2: 双电机同步验证 (8项)

| # | 测试 | 验收标准 |
|---|------|---------|
| B2.1 | `motor_tool torque 1 200 && motor_tool torque 2 200` | 双电机同时输出力矩 |
| B2.2 | `motor_tool torque 1 0 && motor_tool torque 2 0` | 双停 |
| B2.3 | `motor_tool speed 1 1000 && motor_tool speed 2 -1000` | 反向旋转 100RPM |
| B2.4 | `motor_tool stop 1 && motor_tool stop 2` | 双停 |
| B2.5 | `motor_tool abs 1 4500 && motor_tool abs 2 -4500` | 对称位置 |
| B2.6 | `motor_tool multi cur 1:500 2:500` | **多轴广播电流** |
| B2.7 | `motor_tool multi pos 1:3000 2:-3000` | **多轴广播位置** |
| B2.8 | `motor_tool multi vel 1:1000 2:-1000` | **多轴广播速度** |

---

### <a name="b3"></a>B3: 全控制模式验证 (14项)

| # | 模式 | 命令 | 验证点 |
|---|------|------|--------|
| B3.1 | CURRENT | `motor_tool torque 1 200` | 力矩输出 |
| B3.2 | PV | `motor_tool speed 1 1000` | 速度 100RPM |
| B3.3 | PP | `motor_tool abs 1 9000` | 位置 90° |
| B3.4 | CSP | `motor_tool csp 1 6000` | 同步位置 60° |
| B3.5 | CSV | `motor_tool mode 1 csv && motor_tool speed 1 2000` | 同步速度 200RPM |
| B3.6 | MIT | `motor_tool mit 1 32768 0 100 10 0` | 阻抗模式, 中间位 |
| B3.7 | SDO→PDO 切换 | `motor_tool torque 1 500` → 等 2s → `motor_tool pdo 1 cur 500` | 无缝过渡 |
| B3.8 | PDO→SDO 切换 | `motor_tool pdo 1 cur 500` → 等 2s → `motor_tool torque 1 500` | 无缝过渡 |
| B3.9 | 模式切换 | CUR→PV→PP→CSP→CUR | 5 次切换全部成功 |
| B3.10 | PDO disable | `motor_tool pdo disable 1` | 电机脱力 |
| B3.11 | PDO enable | `motor_tool pdo enable 1 && motor_tool pdo 1 cur 500` | 恢复出力 |
| B3.12 | ESTOP | `motor_tool estop 1` | 立即停 |
| B3.13 | RECOVER | `motor_tool recover 1 && motor_tool pdo 1 cur 500` | 恢复出力 |
| B3.14 | PID 调参 | `motor_tool pid 1 200 20 400 40 100 10 && motor_tool speed 1 1000` | 新 PID 生效, 速度响应变化 |

---

### <a name="b4"></a>B4: 传感器透传数据验证 (5项)

| # | 命令 | 验证 |
|---|------|------|
| B4.1 | `motor_tool sensor config 1 4000` | 开透传 250Hz → OK |
| B4.2 | `motor_tool sensor watch 1` | 持续观察: Hall ADC 有变化, Force 有值, Knee 有值 |
| B4.3 | 手动转动电机轴 → 观察 Hall ADC 变化 | Hall 值跟随旋转变化 |
| B4.4 | `motor_tool sensor read 1` | data_valid=1 |
| B4.5 | `motor_tool sensor stop 1` | 停透传 → OK |

---

### <a name="b5"></a>B5: 校准流程验证 (5项)

| # | 命令 | 验证 |
|---|------|------|
| B5.1 | `motor_tool calib start 1 2` | 校准流程启动，电机自动移动→停→验证 |
| B5.2 | `motor_tool calib status` | 显示进度/状态 |
| B5.3 | `motor_tool calib exit` | 退出校准模式 |
| B5.4 | `motor_tool setzero 1` | 手动设零 → OK |
| B5.5 | `motor_tool read angle 1` | 当前=0° (或接近) |

---

### <a name="b6"></a>B6: 故障/边界验证 (6项)

| # | 测试 | 验证 |
|---|------|------|
| B6.1 | `motor_tool read error 1` | 正常运行时应为 0x0000 |
| B6.2 | 急停 → 再使能: `motor_tool disable 1 && motor_tool enable 1` | 无错误残留 |
| B6.3 | 反复 enable/disable ×5 | 不丢帧不卡死 |
| B6.4 | `motor_tool limit_pos 1 1000 && motor_tool abs 1 2000` | 限位生效，不超过 10° |
| B6.5 | `motor_tool save 1` → 断电重启 → `motor_tool read pid 1` | PID/限位/零点 持久化 |
| B6.6 | `motor_tool stop` → 重启 daemon → 电机重新上线 | daemon 恢复 OK |

---

## <a name="part-c"></a>Part C: 上层完整控制项映射表

对照 GD32 ODS 协议，全部上层控制命令的 HAL API 和 motor_tool CLI 映射。

### C1. 控制类 (SOC→MCU→CAN→电机)

| GD32 | 功能 | HAL API | motor_tool CLI | A1测试 | B测试 |
|------|------|---------|---------------|:---:|:---:|
| V-G | Q轴电流 | `motor_hal_set_torque(id, mA)` | `torque <id> <mA>` | ✅ | B1.3 |
| V-I | 目标速度 | `motor_hal_set_velocity(id, rpm)` | `speed <id> <rpm*100>` | ✅ | B1.5 |
| V-J | 加速度 | `motor_hal_set_accel_decel(id, a, d)` | `accel <id> <acc*100>` | ✅ | — |
| V-K | 绝对位置 | `motor_hal_set_position(id, deg)` | `abs <id> <deg*100>` | ✅ | B1.8 |
| V-L | 相对位置 | **❌ 未实现** | **❌ 未实现** | ❌ | ❌ |
| V-U | 位置模式最大速度 | `motor_hal_set_profile_velocity(id, rpm)` | `maxv <id> <rpm*100>` | ✅ | — |
| V-H | 电流斜率 | **❌ 未实现** | **❌ 未实现** | ❌ | ❌ |
| V-A | 清零故障 | `motor_hal_fault_reset(id)` | `fault_reset <id>` | ✅ | B6.2 |
| V-F | 失能 | `motor_hal_disable(id)` | `disable <id>` | ✅ | B1.10 |
| V-E | 系统重启 | `motor_hal_nmt_send(id, RESET)` | `reboot <id>` | ✅ | — |
| C-I | 速度控制(简化) | `motor_hal_set_velocity(id, rpm)` | `speed <id> <rpm*100>` | ✅ | ✅ |
| C-J | 加速度(简化) | `motor_hal_set_accel_decel(id, a, d)` | `accel <id> <acc*100>` | ✅ | — |
| C-K | 绝对位置(简化) | `motor_hal_set_position(id, deg)` | `abs <id> <deg*100>` | ✅ | ✅ |
| C-L | 相对位置(简化) | **❌ 未实现** | **❌ 未实现** | ❌ | ❌ |

### C2. 系统/校准类

| GD32 | 功能 | HAL API | motor_tool CLI | 状态 |
|------|------|---------|---------------|:---:|
| V-B | 返回原点 | **❌** | **❌** | 未实现 |
| V-C | 设原点 | `motor_hal_set_zero(id)` | `setzero <id>` | ✅ |
| V-D | 编码器校准 | `motor_calib` 模块 | `calib start <r> <l>` | ✅ |
| V-M | 传感器校准 | 独立驱动 | — | 未集成 |

### C3. 读取类 (SOC→MCU→CAN→MCU→SOC)

| GD32 | 读什么 | HAL API | motor_tool CLI | 状态 |
|------|--------|---------|---------------|:---:|
| V-a | 绝对值角度 | `motor_hal_get_position(id)` | `read angle <id>` | ✅ |
| V-b | 旋转速度 | `motor_hal_get_velocity(id)` | `read speed <id>` | ✅ |
| V-c | Q轴电流 | `motor_hal_get_current(id)` | `read current <id>` | ✅ |
| V-f | 电机温度 | `motor_hal_get_motor_temp(id)` | `read temp <id>` | ✅ |
| V-h | 电机状态 | `motor_hal_get_statusword(id)` | `read state <id>` | ✅ |
| V-I | 故障码 | `motor_hal_get_fault_code(id)` | `read error <id>` | ✅ |
| V-j | 固件版本 | `motor_hal_sdo_read_u32(id, 0x100A)` | `read version <id>` | ✅ |
| V-d | 母线电压 | `motor_hal_sdo_read_u32(id, 0x????)` | **⚠ 需确认 OD** | 待确认 |
| V-e | 母线电流 | `motor_hal_sdo_read_u32(id, 0x????)` | **⚠ 需确认 OD** | 待确认 |
| V-k | 陀螺仪 | ICM45608 SPI 直连 | `read imu` / `sensor read` | 独立驱动 |
| V-n | 霍尔位置 | `motor_hal_get_sensor(id)` | `sensor read <id>` | ✅ |

### C4. 实时控制 (PDO 路径)

| 功能 | HAL API | motor_tool CLI | 状态 |
|------|---------|---------------|:---:|
| PDO 电流控制 | `motor_hal_set_torque(id, mA)` | `pdo <id> cur <mA>` | ✅ |
| PDO 速度控制 | `motor_hal_set_velocity(id, rpm)` | `pdo <id> vel <rpm*100>` | ✅ |
| PDO 位置控制(CSP) | `motor_hal_ctrl_raw(id, CSP, ...)` | `pdo <id> csp <cnt>` | ✅ |
| MIT 阻抗控制 | `motor_hal_mit_control(id, ...)` | `mit <id> <p> <v> <kp> <kd> <t>` | ✅ |
| 多轴广播 | `motor_hal_multi_ctrl(hal, cmds, n)` | `multi cur/pos/vel 1:V 2:V` | ✅ |
| PDO Byte0 控制 | `motor_hal_pdo_enable/disable/estop/...` | `pdo enable/estop/recover/...` | ✅ |

### C5. 数据上报

| 功能 | HAL API / motor_tool | 状态 |
|------|---------------------|:---:|
| 定时反馈上报 | `motor_tool watch <ms>` | ✅ |
| CA 数据上报 | `motor_tool report <ms>` | ✅ |
| 传感器看板 | `motor_tool sensor watch <id>` | ✅ |
| TPDO 同步上报 | `motor_hal_tpdo_config` + `motor_hal_sync_start` | ✅ |

### C6. 缺失项清单

| # | 缺失功能 | 优先级 | 备注 |
|---|---------|:---:|------|
| 1 | V-L / C-L 相对位置控制 | 中 | `motor_hal_set_position` 可间接实现 |
| 2 | V-B 返回原点 | 低 | 校准流程的一部分 |
| 3 | V-H 电流斜率控制 | 低 | 驱动板是否支持待确认 |
| 4 | V-d 母线电压读取 | 中 | 需要确认巨蟹 OD 表索引 |
| 5 | V-e 母线电流读取 | 中 | 需要确认巨蟹 OD 表索引 |

---

## <a name="验收通过标准"></a>验收通过标准

| 层级 | 测试范围 | 项数 | 通过条件 |
|------|---------|:---:|---------|
| A1 | motor_tool CLI 单步 | 53 | 全部返回 OK，candump 帧格式正确 |
| A2 | HAL 并发压力 | 12 | `11/11 passed` 或 `12/12 passed` |
| B1 | 单电机基础 | 10 | 全部通过，位置/速度反馈合理 |
| B2 | 双电机同步 | 8 | 双轴协同正常，多轴广播 OK |
| B3 | 全控制模式 | 14 | 所有模式切换正常 |
| B4 | 传感器透传 | 5 | 数据有效，物理转动有变化 |
| B5 | 校准流程 | 5 | 校准完成，零点正确 |
| B6 | 故障/边界 | 6 | 异常处理无 crash |

**总计**: ~113 项测试
**最终标准**: Part A 全部通过 → CANFD 协议层验收完成。Part A+B 全部通过 → HAL+协议+物理层全链验收完成。

---

## <a name="附录"></a>附录: 编译与运行

### 编译 HAL 库
```bash
cd motor_hal_c/build
cmake .. && make -j$(nproc)
```

### 编译 motor_tool
```bash
cd motor_hal_c/tools/build
cmake .. && make -j$(nproc)
```

### 运行 A1 测试
```bash
# 终端 1: 监控总线
candump can0 -x

# 终端 2: 运行 motor_tool 命令
motor_tool daemon can0 &
sleep 2
# 然后逐条执行 A1 命令
```

### 运行 A2 压力测试
```bash
pkill motor_tool
cd motor_hal_c/build
sudo ./hal_stress_test
```

### 运行 B 测试
```bash
# A1+A2 通过后再做
motor_tool daemon can0 &
sleep 2
# 然后逐条执行 B1~B6 命令
```
