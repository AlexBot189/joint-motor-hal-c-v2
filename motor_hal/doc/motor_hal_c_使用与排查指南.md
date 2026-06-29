# motor_hal_c 完整使用与排查指南

> 版本: v2 | 日期: 2026-06-06  
> 硬件: RV1126B + CANFD | 驱动板: 无锡巨蟹智能 | 协议: CANopen CiA 402  
> 作者: 张君宝

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────────────────┐
│                        motor_tool (CLI 调试工具)                      │
│  motor_tool daemon can0 &    → 启动守护进程                           │
│  motor_tool speed 1 5000     → 控制电机1: 50.00 RPM                  │
│  motor_tool read all 0       → 读取双电机全部状态                      │
│  motor_tool watch 200        → 200ms 持续监控                         │
│  motor_tool startup 1        → 上电启动电机1                          │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ Unix Socket
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        motor_hal (C 库, libmotor_hal.a)               │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                    对外公共 API (motor_hal.h)                  │    │
│  │                                                               │    │
│  │  生命周期: create / init / destroy                            │    │
│  │  电机管理: add_motor / remove_motor                           │    │
│  │  启动停用: startup / enable / disable / fault_reset           │    │
│  │  实时控制: set_position / set_velocity / set_torque /         │    │
│  │            mit_control / ctrl_raw / stop / set_brake          │    │
│  │  参数配置: set_mode / set_pid / set_zero / save_flash         │    │
│  │  状态查询: get_feedback / get_state / get_position            │    │
│  │  全局控制: nmt_broadcast / sync / multi_ctrl                  │    │
│  │  工具函数: counts_to_deg / deg_to_counts                      │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
│  │can_driver│ │sdo_client│ │pdo_handler│ │motor_calib│ │feedback_ │   │
│  │SocketCAN │ │SDO同步读写│ │PDO/MIT/  │ │校准状态机 │ │parser    │   │
│  │封装      │ │队列+CV   │ │多轴构造  │ │		│ │解析      │   │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘   │
│       │            │            │            │            │           │
│       ▼            ▼            ▼            ▼            ▼           │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                   接收线程 (_recv_thread_fn)                    │   │
│  │  唯一 recv 入口: while(running) { can_driver_recv → dispatch } │   │
│  │                                                                 │   │
│  │  dispatch 分发规则:                                             │   │
│  │    0x580 → SDO 响应 → 入队 → condvar → sdo_client 消费         │   │
│  │    0x300 → 反馈帧 → PI mutex 写缓存 + 回调                      │   │
│  │    0x180 → 标准 TPDO1 → 写缓存 + 回调                          │   │
│  │    0x700 → Bootup/Heartbeat → 标志位                            │   │
│  │    0x080 → EMCY 紧急报文 → 错误回调                             │   │
│  │    0x680 → 传感器透传 → 写缓存 + 回调                           │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    控制线程 (你的代码)                           │   │
│  │  只读缓存 (motor_hal_get_feedback) + 只发 PDO (motor_hal_set_*) │   │
│  │  ★ 不调 recv/poll/select, RT 安全                              │   │
│  └──────────────────────────────────────────────────────────────┘   │
└──────────────────────┬──────────────────────────────────────────────┘
                       │ 系统调用 write()/read()
                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   Linux SocketCAN (can0)                              │
│                   CANFD: 仲裁 1Mbps / 数据 5Mbps                      │
└──────────────────────┬──────────────────────────────────────────────┘
                       │ MCP2518FD (SPI → CANFD PHY)
                       ▼
            ┌───────────┴───────────┐
            │      物理总线          │
            ├───────────────────────┤
            │  右电机 ID=1 (CAN Node) │
            │  左电机 ID=2 (CAN Node) │
            │  可扩展至 ID=3,4...     │
            └───────────────────────┘
```

---

## 2. 数据控制与读取流程（完整链路）

### 2.1 控制通路（发命令 → 电机执行）

```
应用调用 motor_hal_set_position(hal, 1, 45.0f)
  │
  ├─ motor_deg_to_counts(45.0) → int16 编码器值
  ├─ _find_motor(hal, 1)       → 查电机节点
  ├─ 检查 enable 状态            → 未使能返回 -EAGAIN
  │
  └─ pdo_ctrl_send(drv, node_id, PP, enable, brake, target1, target2, ff)
       ├─ canopen_custom_pdo_build() → 构造 7 字节 PDO 帧
       │   Byte[0]: Enable|Brake|ClearErr|Mode
       │   Byte[1-2]: target1 (大端)
       │   Byte[3-4]: target2 (大端)
       │   Byte[5-6]: feedforward (大端)
       │
       └─ can_driver_send(drv, &frame)
            └─ write(sock_fd, &cfd) → SocketCAN → 驱动板 → 电机

延迟: 一次 write() 系统调用 + CANFD 发送时间 (< 50μs)
```

**所有控制 API 都走 PDO 通道（非阻塞，RT 安全）**:
| API | PDO帧 | 字节 | 延迟 |
|-----|-------|------|------|
| `set_position` | 0x100+ID, 自定义PDO | 7B | <50μs |
| `set_velocity` | 0x100+ID, 自定义PDO | 7B | <50μs |
| `set_torque` | 0x100+ID, 自定义PDO | 7B | <50μs |
| `mit_control` | 0x110+ID, MIT PDO | 9B | <50μs |
| `ctrl_raw` | 0x100+ID, 自定义PDO | 7B | <50μs |
| `multi_ctrl` | 0x200, 多轴广播CANFD | 64B | <50μs |

### 2.2 反馈通路（电机上报 → 应用读取）

```
硬件: 驱动板周期性发送 0x300+ID 反馈帧 (12字节, 大端)
  │
CAN 总线
  │
接收线程 _recv_thread_fn
  │
  └─ can_driver_recv(drv, &f, 100ms timeout)
       └─ select() → read(sock_fd, &cfd)
            │
            ▼
       _dispatch_frame(hal, &f)
            │
            ├─ case 0x300: (私有反馈帧)
            │    pdo_feedback_parse(&f, &fb)
            │    ├─ position     = 大端 int16  (Byte0-1)
            │    ├─ velocity     = 大端 int16  (Byte2-3)
            │    ├─ current_iq   = 大端 int16  (Byte4-5)
            │    ├─ error_code   = 大端 uint16 (Byte6-7)
            │    ├─ temperature  = 大端 int16  (Byte8-9)
            │    ├─ mode         = Byte10
            │    └─ status_byte  = Byte11
            │    fb.timestamp_us = now_us()
            │
            │    → PI mutex 写缓存 (cached_fb)
            │    → 触发 feedback_cb (在接收线程上下文中)
            │
            ├─ case 0x180: (标准 TPDO1, SYNC同步上报, 12字节小端)
            │    解析: Statusword(16b)+Position(32b)+Velocity(32b)+Current(16b)
            │    → PI mutex 写缓存
            │    → 触发 feedback_cb
            │
            └─ case 0x680: (传感器透传, 8字节小端 bit-packed)
                解析: hall_adc0~2 + force_raw + knee_adc + hw_sw + data_valid
                → PI mutex 写缓存
                → 触发 sensor_cb
```

**应用侧读取（非阻塞）**:
```c
motor_feedback_t fb;
motor_hal_get_feedback(hal, 1, &fb);  // PI mutex + memcpy, ~几十ns
float angle = motor_counts_to_deg(fb.position);
if (fb.status_byte & 0x20) { /* 有错误 */ }
```

### 2.3 SDO 通路（参数读写，阻塞 50~200ms）

```
应用调用 motor_hal_set_mode(hal, 1, MOTOR_MODE_CURRENT)
    │
    └─ sdo_write_simple(drv, node, 0x6060, 0x00, 0x0A, 1)
         ├─ canopen_sdo_write_build() → SDO 写帧 (0x601, 8字节)
         ├─ can_driver_send(drv, &f)  → 发 CAN
         │
         └─ _sdo_wait_response(drv, node, index, &val, &abort, 200ms)
              └─ pthread_cond_timedwait(&g_sdo_queue.cond, ...)
                   │
                   │  ← 接收线程收到 0x581 → sdo_push_response → cond_broadcast
                   │
                   └─ 匹配 index → 解析 → 返回

延迟: SDO 发送 + 驱动板处理 + 响应 = 50~200ms (同步阻塞)
重试: 默认 2 次重试, 200ms 超时
```

---

## 3. 完整 API 分类手册

### 3.1 生命周期

| API | 作用 | 何时调用 |
|-----|------|---------|
| `motor_hal_create()` | 分配 HAL, 初始化内部结构 | 程序启动, **第一个调用** |
| `motor_hal_init(hal, "can0", 1M, 5M)` | 打开 SocketCAN, 启用 CANFD | create 之后, 任何其他 API 之前 |
| `motor_hal_destroy(hal)` | 自动停止线程、脱使能、关CAN、释放内存 | 程序结束 |

### 3.2 电机管理

| API | 作用 | 注意事项 |
|-----|------|---------|
| `motor_hal_add_motor(hal, &cfg)` | 注册电机, cfg 内部拷贝 | 最多 16 个, 同 ID 重复注册返回 -EEXIST |
| `motor_hal_remove_motor(hal, id)` | 移除电机, 紧凑数组 | 不自动脱使能, 先调 disable |

### 3.3 启动与停用（SDO，阻塞）

| API | 作用 | 内部步骤 |
|-----|------|---------|
| `motor_hal_startup(hal, id, timeout)` | **完整启动** | ①快速试探Bootup(500ms) ②SDO配心跳(验证在线) ③关看门狗 ④读固件版本 ⑤DS402使能 ⑥等120ms抱闸 |
| `motor_hal_enable(hal, id)` | 手动使能 | Shutdown→SwitchOn→EnableOp, 每步20ms |
| `motor_hal_disable(hal, id)` | 脱使能 | SDO写CW=0x06 (Shutdown) |
| `motor_hal_fault_reset(hal, id)` | 故障复位 | SDO写CW=0x80, 回 SWITCH_ON_DISABLED |

**启动时序要求**:
```
motor_hal_create → init → add_motor → recv_start(★) → startup
                                       ↑ 必须在 startup 之前!
```

### 3.4 实时控制（PDO，非阻塞，RT 安全）

| API | 模式 | 参数说明 |
|-----|------|---------|
| `set_position(hal, id, angle_deg)` | PP | 目标角度 °, ±180° |
| `set_velocity(hal, id, rpm)` | PV | 目标转速 RPM, 正=正转 |
| `set_torque(hal, id, ma)` | 电流环 | 目标 Iq 电流 mA |
| `mit_control(hal, id, pos, vel, kp, kd, t)` | MIT | 阻抗控制五参数 |
| `ctrl_raw(hal, id, mode, t1, t2, ff)` | 任意 | 裸 PDO 三个参数 |
| `stop(hal, id)` | PP | 目标位置=0 |
| `set_brake(hal, id, release)` | — | 抱闸松/紧 |

### 3.5 参数配置（SDO，阻塞 50~200ms）

| API | 对应的 OD | 说明 |
|-----|-----------|------|
| `set_mode(hal, id, mode)` | 0x6060 | 切换控制模式 |
| `set_accel_decel(hal, id, a, d)` | 0x6083/0x6084 | 加减速 RPM/s |
| `set_profile_velocity(hal, id, rpm)` | 0x6081 | 位置模式最大轨迹速度 |
| `set_pid(hal, id, &pid)` | 0x2532~0x2537 | 6个 PID 一次性写入 |
| `read_pid(hal, id, &pid)` | 0x2532~0x2537 | 读取全部 PID |
| `save_flash(hal, id)` | 0x2539 | 参数保存到 Flash |
| `set_zero(hal, id)` | 0x2531 | 当前位置记为零点 |
| `set_limits(hal, id, +°, -°)` | 0x607D | 软限位 |
| `sdo_read_u32(hal, id, idx, sub, &v)` | 任意 | 通用 SDO 读 |
| `sdo_write(hal, id, idx, sub, v, size)` | 任意 | 通用 SDO 写 |

### 3.6 反馈读取（非阻塞缓存读取）

| API | 来源 | 延迟 |
|-----|------|------|
| `get_feedback(hal, id, &fb)` | 0x300 私有反馈帧缓存 | PI mutex+memcpy, ~几十ns |
| `get_sensor(hal, id, &s)` | 0x680 传感器透传缓存 | 同上 |
| `get_state(hal, id)` | SDO 读 0x6041 | 50~200ms (**阻塞!**) |
| `get_position(hal, id)` | SDO 读 0x6064 | 50~200ms (**阻塞!**) |
| `get_velocity(hal, id)` | SDO 读 0x606C | 50~200ms (**阻塞!**) |
| `get_current(hal, id)` | SDO 读 0x6078 | 50~200ms (**阻塞!**) |

> **重要**: 运行时用 `get_feedback` 读缓存, 不要用 SDO 查询 (SDO 会阻塞 RT 控制循环!)

### 3.7 接收线程与回调

| API | 作用 |
|-----|------|
| `recv_start(hal)` | 启动接收线程, **在 startup 之前** |
| `recv_stop(hal)` | 停止 (一般用不到, destroy 自动调) |
| `recv_is_running(hal)` | 查询状态 |
| `set_feedback_cb(hal, id, cb, ctx)` | 每收到反馈帧触发, **接收线程上下文** |
| `set_error_cb(hal, id, cb, ctx)` | EMCY/反馈错误触发 |
| `set_state_cb(hal, id, cb, ctx)` | 状态变更触发 |
| `set_sensor_cb(hal, id, cb, ctx)` | 传感器透传触发 |

### 3.8 全局控制

| API | 作用 |
|-----|------|
| `nmt_broadcast(hal, cmd)` | 总线 NMT 广播 (START/STOP/RESET...) |
| `sync(hal)` | 发 SYNC 帧 (0字节, 极低开销) |
| `sync_start(hal, period_us)` | 启动 SYNC 定时器线程 |
| `sync_stop(hal)` | 停止 SYNC |
| `tpdo_config(hal, id, sync_count)` | 配置从站同步上报 |
| `multi_ctrl(hal, cmds, count)` | 一帧 CANFD 控制最多 8 个电机 |

### 3.9 工具函数

| 函数 | 换算 | 实现 |
|------|------|------|
| `motor_counts_to_deg(counts)` | count×(360/65536) → ° | inline |
| `motor_deg_to_counts(deg)` | deg×(65536/360) → count | inline, 钳位 ±180° |
| `motor_temp_to_c(raw)` | raw×0.1 → °C | inline |
| `motor_ma_to_a(ma)` | ma÷1000 → A | inline |

### 3.10 传感器透传

| API | 作用 |
|-----|------|
| `sensor_config(hal, id, period_div, format)` | 配置透传周期: 250μs×N, format=3(CANFD) |
| `sensor_stop(hal, id)` | 停止透传 |
| `get_sensor(hal, id, &s)` | 读取缓存 |

---

## 4. motor_tool 调试命令速查表

### 4.1 系统命令

```bash
motor_tool daemon can0 &        # 启动守护进程 (注册 ID 1~4, 自动 recv_start)
motor_tool startup 1             # 上电启动电机1 (等 Bootup + SDO 使能)
motor_tool startup 0             # 上电启动所有电机
motor_tool enable 1              # 手动使能电机1
motor_tool disable 1             # 脱使能
motor_tool reset 1               # 故障复位
motor_tool stop                  # 停止 daemon
```

### 4.2 控制命令（×100 精度）

```bash
motor_tool speed 1 5000          # 电机1 速度 50.00 RPM
motor_tool speed 0 10000         # 双电机 速度 100.00 RPM  (id=0=广播)
motor_tool abs 2 4500            # 电机2 绝对位置 45.00°
motor_tool rel 1 1000            # 电机1 相对位置 +10.00° (先读再设)
motor_tool accel 1 3000          # 电机1 加速度 30.00 RPM/s
motor_tool maxv 1 2000           # 电机1 位置模式最大速度 20.00 RPM
motor_tool mode 1 cur            # 电机1 切换电流模式 (pp/pv/csp/csv/cur)
motor_tool torque 1 2000         # 电机1 力矩 2000mA
motor_tool csp 1 3000            # 电机1 CSP 位置 30.00°
motor_tool mit 1 30 0 2.0 0.3 0  # 电机1 MIT: pos=30° vel=0 kp=2.0 kd=0.3 torque=0
motor_tool brake 1 release       # 电机1 松抱闸
motor_tool quickstop 1           # 电机1 急停
```

### 4.3 配置命令

```bash
motor_tool save 1                # 保存参数到 Flash
motor_tool setzero 1             # 当前位置设为零点
motor_tool pid 1 100 50 200 100 150 80  # PID: cp ci vp vi pp pi
```

### 4.4 读取命令

```bash
motor_tool read angle 1          # 读角度 (编码器 count)
motor_tool read speed 1          # 读速度 (RPM)
motor_tool read current 1        # 读电流 (mA)
motor_tool read temp 1           # 读温度 (raw, ×0.1 = °C)
motor_tool read state 1          # 读 DS402 状态
motor_tool read error 1          # 读故障码
motor_tool read version 1        # 读固件版本
motor_tool read all 0            # 读双电机全部信息
```

### 4.5 调试命令

```bash
motor_tool sdoread 1 0x100A      # SDO 读固件版本 (对象字典 0x100A)
motor_tool sdoread 1 0x6041      # SDO 读状态字
motor_tool sdoread 1 0x6064      # SDO 读当前位置
motor_tool sdowrite 1 0x6040 0 0x0F 3  # SDO 写使能命令
motor_tool watch 200             # 200ms 持续监控, Ctrl+C 退出
```

---

## 5. 问题排查指南（调试配方）

### 5.1 电机不上电 / 收不到数据

**症状**: `motor_tool read all 1` 返回全是 0

**排查步骤**:
```bash
# Step 1: 检查 CAN 接口是否 up
ip -details link show can0
# 确认: "UP", bitrate 1000000, data bitrate 5000000, "fd on"

# Step 2: 用 candump 看总线上有没有数据
candump can0
# 如果什么都看不到 → 硬件问题 (接线/供电/驱动板状态)

# Step 3: 如果能看到 CAN 帧, 检查是不是 motor_hal 没收到
# 先用 motor_tool 启动 daemon
motor_tool daemon can0 &
# 切到 daemon 的 stderr 看日志:
#   → Bootup node=1  (说明电机上电了)
#   没看到 Bootup → 电机没上电

# Step 4: Bootup 一直没收到
motor_tool sdoread 1 0x100A     # 直接 SDO 读固件版本验证通信
# 返回 0 → 通信正常
# 超时 → 检查电机供电, 检查 CAN 接线
```

### 5.2 数据格式不对

**症状**: 算法层反馈的位置/速度/电流数值明显不对

**排查步骤**:
```bash
# Step 1: 确认字节序
# 反馈帧 (0x300) 是大端, PDO 控制帧也是大端
# 传感器透传 (0x680) 是小端 bit-packed

# Step 2: 用 candump 抓原始帧验证
candump can0 | grep "301\|302"  # 看反馈帧

# 例如: can0  301  [12]  DE 0A FF FF 0A 00 00 00 18 00 01 80
# Byte[0-1] = 0xDE 0x0A → 大端 int16 = 0xDE0A = -8694 → 用 motor_counts_to_deg(-8694) 验证
# Byte[2-3] = 0xFF 0xFF → 大端 int16 = -1 RPM
# Byte[4-5] = 0x0A 0x00 → 大端 int16 = 2560 mA
# Byte[10]  = 0x01 → 模式 = PP
# Byte[11]  = 0x80 → bit7=1 (使能), bit6=0 (抱闸吸合), bit5=0 (无错误)

# Step 3: 常见字节序错配症状:
#   大端当小端读 → 数值跳变, 正负号不对
#   小端当大端读 → 同上
#   偏移错位 → 所有字段都乱

# Step 4: 用 motor_tool read all 对比
motor_tool read all 1
# 此时输出的就是 motor_hal 已经解析好的值,
# 如果 motor_tool 输出正确但算法层读到的不对 → 问题在算法侧
```

### 5.3 SDO 超时 / SDO 通信失败

**症状**: `motor_tool startup 1` 超时, 返回 -ETIMEDOUT

**排查步骤**:
```bash
# Step 1: 确认接收线程在跑
# daemon 启动时 stderr 会输出, 或代码里加:
printf("recv running: %d\n", motor_hal_recv_is_running(hal));
# 如果 false → 检查 recv_start 是否在 startup 之前调用了

# Step 2: cansend 手动测试通信
cansend can0 601#2F17011000E80300    # SDO 写电机1心跳 2000ms
# 看是否有 581 响应
candump can0 | grep "581\|601"        # 确认双向通信

# Step 3: 如果 candump 能看到 601 发出但没有 581 响应
# → 电机不在线 / CAN ID 不匹配
# → 检查电机 node_id (默认1, 对应 COB SDO_TX=0x601)

# Step 4: 确认电机 node_id
motor_tool sdoread 1 0x2530     # 读电机 ID (OD 0x2530)
# 返回 1 → 正确

# Step 5: SDO 队列满 (极少见)
# 看 stderr 有没有 "[SDO] queue: dropping" 日志
```

### 5.4 PDO 控制发了但电机不动

**排查步骤**:
```bash
# Step 1: 检查使能状态
motor_tool read state 1
# 必须输出 "OPERATION_ENABLED" 才能响应 PDO

# Step 2: 如果没使能
motor_tool enable 1

# Step 3: 检查控制模式
motor_tool read mode 1
# CSP 模式下 set_position 实际走 PP 模式, 确认模式匹配

# Step 4: candump 确认 PDO 帧发出
candump can0 | grep "101\|111"  # 0x101=电机1自定义PDO, 0x111=电机1 MIT PDO
# 如果看不到 PDO 帧 → 检查 motor_hal_set_position 返回值 (0=成功)

# Step 5: 检查抱闸
# feedback.status_byte bit6=1 → 抱闸已释放, 电机可以转
# bit6=0 → 抱闸吸合, 电机锁死
# 手动松抱闸: motor_tool brake 1 release
```

### 5.5 反馈数据一直不变 (卡住)

**排查步骤**:
```bash
# Step 1: candump 确认总线有反馈帧
candump can0 | grep "301"  # 电机1反馈帧
# 如果持续看到 → 接收线程问题

# Step 2: 检查接收线程是否在运行
# 在代码中加监控:
motor_feedback_t fb1, fb2;
motor_hal_get_feedback(hal, 1, &fb1);
usleep(100000);
motor_hal_get_feedback(hal, 1, &fb2);
if (fb1.timestamp_us == fb2.timestamp_us) {
    printf("WARN: feedback not updating for 100ms!\n");
}

# Step 3: 如果 candump 看不到反馈帧
# → 驱动板没有周期上报, 检查:
#   - 驱动板是否正常工作
#   - 某些模式下驱动板上报频率较低
#   - 可以配置 TPDO: motor_hal_tpdo_config(hal, 1, 1)
```

### 5.6 传感器透传收不到数据

```bash
# 确认透传是否已启动
motortool sensor 1 4 3       # 1KHz 透传, CANFD 模式

# 用 candump 验证
candump can0 | grep "681"    # 传感器帧 COB=0x681

# 如果 candump 有但 motor_hal_get_sensor 读到空
# → 检查 sensor_config 返回值, 检查 period_div 是否 > 0
```

### 5.7 常见错误码速查

| error_code | 含义 | 典型处理 |
|------------|------|---------|
| 0x0000 | 正常 | — |
| 0x0001 | 过压 | 检查 24V 电源 |
| 0x0002 | 欠压 | 检查供电 |
| 0x0004 | 过温 | 降温后自动恢复 |
| 0x0008 | 堵转 | 检查机械卡死, fault_reset |
| 0x0010 | 过载 | 降低力矩, fault_reset |
| 0x0100 | 编码器超时 | 检查编码器线, 断电重启 |
| 0x1000 | 位置误差过大 | 降低 KP, 检查跟随 |
| 0x2000 | 编码器故障 | 硬件检查, 更换驱动板 |

**快速检测**: `feedback.status_byte & 0x20` → 有错误时读 `feedback.error_code`

### 5.8 算法对接时的常见坑

| 问题 | 原因 | 解决 |
|------|------|------|
| 位置值跳变 ±32768 | 编码器溢出, int16 跨边界 | 算法层做 unwrap 处理 |
| 速度值忽大忽小 | 大端/小端字节序混淆 | 确认 0x300 为大端, PDO 控制也为大端 |
| 力矩发了没反应 | 模式没切换到电流环 | `set_mode(MOTOR_MODE_CURRENT)` 后再 set_torque |
| feedback 读到旧数据 | 接收线程未启动 | 在 startup 之前调 recv_start |
| SDO 调用后卡死 | 接收线程没在跑 | SDO 的 condvar 等不到 notify |
| algo_heartbeat/POSIX SHM 相关 | 不在 motor_hal 层 | 看 `exo_periph_design.md` |

---

## 6. 快速上手指南

### 最小可运行代码（复制即用）

```c
#include "motor_hal.h"
#include <unistd.h>
#include <stdio.h>

int main() {
    // 1. 创建 HAL → 初始化 CANFD
    motor_hal_t *hal = motor_hal_create();
    if (motor_hal_init(hal, "can0", 1000000, 5000000) < 0) {
        perror("CAN init failed"); return 1;
    }

    // 2. 注册电机
    motor_config_t cfg = {0};
    cfg.node_id           = 1;
    cfg.heartbeat_ms      = 2000;
    cfg.profile_accel     = 5000;
    cfg.profile_decel     = 5000;
    cfg.disable_watchdog  = true;      // ★ 必开
    cfg.auto_enable       = true;
    cfg.bootup_timeout_ms = 5000;
    motor_hal_add_motor(hal, &cfg);

    // 3. ★ 先启动接收线程
    motor_hal_recv_start(hal);

    // 4. 再启动电机
    if (motor_hal_startup(hal, 1, 5000) < 0) {
        fprintf(stderr, "Startup failed\n"); return 1;
    }

    // 5. 控制循环
    for (int i = 0; i < 100; i++) {
        motor_feedback_t fb;
        motor_hal_get_feedback(hal, 1, &fb);
        printf("pos=%7d vel=%5d cur=%5dmA temp=%d err=0x%04X st=0x%02X\n",
               fb.position, fb.velocity, fb.current_iq,
               fb.temperature, fb.error_code, fb.status_byte);
        motor_hal_set_position(hal, 1, 30.0f);
        usleep(5000);  // 200Hz
    }

    // 6. 清理 (自动脱使能 + 关 CAN)
    motor_hal_destroy(hal);
    return 0;
}
```

### DEBUG 模式编译

```bash
# 编译时加 MOTOR_DEBUG_HEX 宏, 所有 CAN 帧会自动 hex dump
gcc -DMOTOR_DEBUG_HEX -Iinc my_app.c src/*.c -lpthread -lm -o my_app

# 运行时 stderr 输出:
# [TX] id=0x601 dlc=8 : 2F 17 10 10 D0 07 00 00  ← SDO 写心跳 2000ms
# [RX] id=0x581 dlc=8 : 60 17 10 10 00 00 00 00  ← SDO 确认
# [recv] id=0x301 dlc=12 : 0A 00 00 10 64 00 00 00 20 00 01 80 ← 反馈帧
```

---

## 7. 架构评审要点

### 7.1 设计原则

1. **传输层隔离**: 上层只调 API, 不感知 CANopen/SDO/PDO 协议细节
2. **控制/读取分离**: 控制走 PDO (非阻塞, RT 安全), 配置走 SDO (同步阻塞)
3. **接收线程唯一化**: 只有一个线程阻塞 recv, 其余线程只读缓存/发 PDO
4. **PI mutex**: 所有反馈缓存用 PTHREAD_PRIO_INHERIT, 防止优先级倒置
5. **零外部依赖**: 仅 Linux SocketCAN + pthread + math

### 7.2 关键指标

| 指标 | 值 | 说明 |
|------|-----|------|
| PDO 控制延迟 | < 50μs | write() + CANFD 发送 |
| 反馈读取延迟 | ~几十 ns | PI mutex + memcpy |
| SDO 参数读写 | 50~200ms | 同步阻塞, 有重试 |
| 支持电机数 | 最多 16 | 可扩展 |
| 控制模式 | 6 种 | PP/PV/CSP/CSV/Current/MIT |
| 编译产物 | libmotor_hal.a | 静态库, ~50KB |
| CANFD 物理 | 仲裁 1Mbps / 数据 5Mbps | 标准帧 11bit |

| 同步机制 | tx_re_flag 信号量 | condvar + PI mutex |
| 调试工具 | 串口助手 | motor_tool CLI |
| RT 能力 | 裸机 | PREEMPT_RT 内核 |

### 7.4 风险与限制

| 风险 | 影响 | 缓解 |
|------|------|------|
| CAN 断线 | SDO/PDO 全不可用 | 反馈超时检测 + 自动重连 |
| SDO 超时 | 启动/配置失败 | 重试机制 + 诊断日志 |
| 线程优先级不当 | RT 控制抖动 | cyclictest 验证 + RT 内核 |
| 编码器溢出 | int16 跨 ±32768 | 算法层 unwrap 处理 |
| Flash 写入次数 | 寿命限制 | 调试不调 save_flash, 量产一次写入 |

---

## 附录A: 文件清单

```
motor_hal_c/
├── CMakeLists.txt
├── README.md                    # 使用文档
├── inc/
│   ├── motor_hal.h              # ★ 公共 API (50+ 函数)
│   ├── motor_hal_types.h        # ★ 类型/常量/COB-ID/对象字典
│   ├── canopen_frames.h         # 帧构造/解析 (纯函数)
│   └── pdo_mapper.h             # PDO 映射工具
├── src/
│   ├── motor_hal.c              # ★ 核心实现 (API + 帧分发 + 接收线程)
│   ├── motor_hal_startup.c      # 启动流程 (Bootup + DS402)
│   ├── can_driver.c             # SocketCAN 驱动封装
│   ├── can_driver_internal.h    # 内部接口
│   ├── sdo_client.c             # SDO 客户端 (队列+condvar)
│   ├── sdo_client_internal.h    # SDO 内部接口
│   ├── pdo_handler.c            # PDO/MIT/多轴发送
│   ├── pdo_mapper.c             # PDO 映射实现
│   ├── canopen_frames.c         # 帧引擎
│   ├── feedback_parser.c        # 反馈解析 + 便捷位字段函数
│   ├── heartbeat.c              # 心跳/看门狗
│   ├── nmt_master.c             # NMT 网络管理
│   └── utils.c                  # 时间/睡眠/诊断查表
├── tools/
│   ├── main.c                   # motor_tool 入口
│   ├── daemon.c/.h              # 守护进程 (Unix socket)
│   ├── tool_hal.c/.h            # HAL 薄封装 (×100 + 广播)
│   ├── command_registry.c/.h    # 命令注册表
│   ├── cmd_control.c            # speed/abs/rel/mode/torque...
│   ├── cmd_read.c               # read angle/speed/current...
│   ├── cmd_watch.c              # watch 持续监控
│   ├── cmd_calib.c              # 校准命令
│   ├── cmd_sensor.c             # 传感器命令
│   ├── cmd_system.c             # startup/enable/disable/reset
│   ├── cmd_help.c               # 帮助
│   ├── cmd_report.c             # 数据上报
│   └── CMakeLists.txt
├── examples/
│   ├── single_motor.c
│   ├── dual_motor.c
│   ├── dual_motor_full.c
│   ├── mit_control.c
│   └── feedback_callback.c
└── doc/
    ├── exo_periph_design.md     # 外骨骼外设节点框架设计 v1.3
    └── known_issues.md          # 已知问题清单
```
