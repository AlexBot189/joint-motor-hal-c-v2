# 外骨骼机器人 CANFD 控制系统 — 汇报材料

> 项目: 外骨骼机器人 RV1126B 关节电机控制系统  
> 日期: 2026-06-06  
> 拟汇报: 技术负责人 / 研发团队 / 评审工程师

---

## 一、系统架构全貌

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        上位机 Web / CLI 工具                              │
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  页面工具 (Web UI)                      CLI 工具 (motor_tool)      │    │
│  │                                                                    │    │
│  │  [启动] [使能] [停止] [急停]          motor_tool daemon can0 &     │    │
│  │  [位置 45°] [速度 50RPM] [力矩 2A]    motor_tool abs 1 4500       │    │
│  │  [读反馈] [传感器] [持续监控]          motor_tool watch 200        │    │
│  │  [PID 调参] [Flash 保存] [校准]       motor_tool sensor watch 1   │    │
│  └────────────────────┬───────────────────────────────────────────────┘    │
│                       │ Unix Socket (/tmp/motor_tool.sock)                │
└───────────────────────┼───────────────────────────────────────────────────┘
                        │
┌───────────────────────┼───────────────────────────────────────────────────┐
│               exo_node (基于 petrobot_periph_manager 量产框架)              │
│                                                                           │
│   6 线程模型 (SCHED_FIFO RT):                                             │
│   ┌───────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐  │
│   │ 控制线程 1KHz  │ │ 接收线程      │ │ 上报线程 200Hz│ │ 安全监控 100Hz│  │
│   │ prio=90       │ │ prio=85      │ │ prio=80      │ │ prio=75      │  │
│   │ PDO → CANFD   │ │ CAN帧 分发   │ │ 反馈→SHM      │ │ 心跳→紧急停机 │  │
│   └───────┬───────┘ └──────┬───────┘ └──────┬───────┘ └──────┬───────┘  │
│           │                │                │                │           │
│   ┌───────────────┐ ┌──────────────────────────────────────────────┐    │
│   │ 状态机线程     │ │ ROS 调试线程 (ROS Service / Topic)           │    │
│   │ prio=0        │ │ prio=0                                       │    │
│   │ startup/calib │ │ /motor/startup /motor/config /motor/feedback │    │
│   └───────────────┘ └──────────────────────────────────────────────┘    │
│                                                                           │
│  接口: 共享内存 /dev/shm/exo_shm (算法进程读反馈 + 写控制命令)             │
└───────────────────────────┬───────────────────────────────────────────────┘
                            │
┌───────────────────────────┼───────────────────────────────────────────────┐
│                   motor_hal_c (libmotor_hal.a 静态库)                       │
│                                                                           │
│  纯 C 库, 零外部依赖 (仅 Linux SocketCAN + pthread + math)                 │
│                                                                           │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │
│  │ can_driver│ │sdo_client│ │pdo_handler│ │motor_calib│ │feedback_ │      │
│  │SocketCAN │ │ SDO 队列  │ │ PDO/MIT/ │ │校准状态机 │ │parser    │      │
│  │ 封装      │ │ +condvar │ │ 多轴构造  │ │ │ │解析      │      │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘      │
│       │            │            │            │            │              │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                  接收线程 (唯一 recv 入口)                          │   │
│  │                                                                   │   │
│  │  CAN 帧自动分发:                                                   │   │
│  │    0x580 → SDO 响应入队      0x300 → 反馈帧写缓存                  │   │
│  │    0x180 → TPDO1 写缓存      0x680 → 传感器透传写缓存              │   │
│  │    0x700 → Bootup/心跳       0x080 → EMCY 紧急报文                 │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                           │
│  公共 API (50+ 函数):                                                     │
│    生命周期: create / init / destroy    控制: set_position / torque / mit │
│    启动停用: startup / enable / disable 配置: set_pid / set_zero / flash  │
│    反馈缓存: get_feedback / get_sensor  回调: on_feedback / on_error      │
└───────────────────────────┬───────────────────────────────────────────────┘
                            │
┌───────────────────────────┼───────────────────────────────────────────────┐
│                   Linux SocketCAN (can0)                                   │
│                   CANFD: 仲裁 1Mbps / 数据 5Mbps                            │
└───────────────────────────┬───────────────────────────────────────────────┘
                            │
┌───────────────────────────┼───────────────────────────────────────────────┐
│                   RV1126B MCP2518FD (SPI → CANFD PHY)                      │
└───────────────────────────┬───────────────────────────────────────────────┘
                            │
        ┌───────────────────┴───────────────────┐
        │           物理 CANFD 总线              │
        ├───────────────────────────────────────┤
        │  右电机 ID=1 (巨蟹关节电机关节)        │
        │  左电机 ID=2 (巨蟹关节电机关节)        │
        │  可扩展 ID=3,4...                      │
        └───────────────────────────────────────┘
```

### 层级说明

| 层级 | 组件 | 职责 | 对上层接口 |
|------|------|------|-----------|
| **硬件** | RV1126B + MCP2518FD + 巨蟹驱动板 | CANFD 物理通信 | — |
| **驱动** | Linux SocketCAN | CANFD 数据收发 | `write()` / `read()` |
| **HAL** | `motor_hal_c` (libmotor_hal.a) | CANopen CiA 402 协议栈, SDO/PDO/反馈/透传 | C API: `set_position()` / `get_feedback()` 等 |
| **节点** | `exo_node` (petrobot 框架) | 线程调度、共享内存、安全监控、校准 | `/dev/shm/exo_shm` 共享内存 |
| **工具** | `motor_tool` CLI | 调试/控制/监控 | Unix Socket JSON |
| **应用** | 上位机 Web UI | 可视化控制面板 | HTTP/WebSocket → Unix Socket |

---

## 二、上位机工具接口汇总

### 2.1 系统控制类

| 指令 | 参数 | 功能 | 等价 HAL API |
|------|------|------|-------------|
| `daemon can0` | can接口名 | 启动守护进程 (初始化 CAN + 注册电机 ID 1~4) | `motor_hal_init` + `motor_hal_add_motor` + `motor_hal_recv_start` |
| `stop` | — | 停止守护进程 (NMT广播Stop + 脱使能 + 清理) | `motor_hal_destroy` |
| `startup <id>` | id: 1/2/0(全部) | 完整上电启动 (Bootup→心跳→看门狗→固件→DS402使能) | `motor_hal_startup` |
| `enable <id>` | id: 1/2/0 | DS402 使能 (Shutdown→SwitchOn→EnableOp) | `motor_hal_enable` |
| `disable <id>` | id: 1/2/0 | 脱使能 (Shutdown) | `motor_hal_disable` |
| `reset <id>` | id: 1/2/0 | 故障复位 (CW=0x80) | `motor_hal_fault_reset` |
| `fault_reset <id>` | id: 1/2/0 | 清零故障 (0=全部) | `motor_hal_fault_reset` |
| `reboot <id>` | id: 1/2 | 电机系统重启 (NMT Reset) | NMT 协议 |

### 2.2 运动控制类

| 指令 | 参数 | 功能 | 模式 |
|------|------|------|------|
| `speed <id> <rpm×100>` | rpm×100: 5050=50.50RPM | 速度控制 | PV |
| `abs <id> <deg×100>` | deg×100: 4500=45.00° | 绝对位置控制 | PP |
| `rel <id> <delta×100>` | delta×100: 1000=10.00° | 相对位置 (先读当前位置再加偏移) | PP |
| `torque <id> <mA>` | mA: 2000=2A | 电流/力矩控制 | Current |
| `csp <id> <deg×100>` | deg×100 | 循环同步位置 (每周期上位机给目标) | CSP |
| `mit <id> <pos> <vel> <kp> <kd> <t>` | pos/vel/kp/kd/torque | MIT 阻抗控制 (力位混合, 柔顺) | MIT |
| `stop [id]` | id: 默认0=全部 | 停止运动 (目标位置=当前位置) | PP |
| `quickstop <id>` | id: 1/2 | 急停 (DS402 Quick Stop, CW=0x02) | — |
| `brake <id> <release\|lock>` | release=松开, lock=吸合 | 抱闸控制 | — |

### 2.3 模式与参数配置类

| 指令 | 参数 | 功能 |
|------|------|------|
| `mode <id> <pp\|pv\|csp\|csv\|cur>` | 模式名 | 切换控制模式 (PP/PV/CSP/CSV/Current) |
| `accel <id> <acc×100>` | acc×100: 3000=30.00RPM/s | 设置加减速度 |
| `maxv <id> <rpm×100>` | rpm×100 | 位置模式最大轨迹速度 |
| `pid <id> <cp> <ci> <vp> <vi> <pp> <pi>` | 6个PID值 | 设置 PID 参数 (电流环P/I, 速度环P/I, 位置环P/I) |
| `save <id>` | id | 保存参数到 Flash (掉电不丢失) |
| `setzero <id>` | id | 当前位置标记为编码器零点 |
| `calib <start\|status\|exit> <id_r> <id_l> [timeout]` | 左右ID + 超时 | 电机零位校准 (左右腿对齐) |

### 2.4 数据读取类

| 指令 | 功能 | 数据来源 |
|------|------|---------|
| `read angle <id>` | 读编码器位置 (counts) | SDO 0x6064 |
| `read speed <id>` | 读当前速度 (RPM) | SDO 0x606C |
| `read current <id>` | 读当前电流 (mA) | SDO 0x6078 |
| `read temp <id>` | 读线圈温度 (raw, ×0.1=°C) | SDO 或反馈帧 |
| `read state <id>` | 读 DS402 状态 (NOT_READY / OP_ENABLED / FAULT …) | SDO 0x6041 |
| `read error <id>` | 读故障码 (0x0001过压, 0x0008堵转 …) | SDO 或反馈帧 |
| `read version <id>` | 读固件版本 | SDO 0x100A |
| `read all <id>` | 读全部信息 (位置/速度/电流/温度/错误码/状态) | 聚合 |
| `watch <period_ms>` | 持续轮询显示 (200ms刷新) | 反馈帧缓存 |

### 2.5 传感器透传类

| 指令 | 参数 | 功能 |
|------|------|------|
| `sensor config <id> <period_ms> [bus_fmt]` | period_ms: 1=1KHz, 5=200Hz, 10=100Hz | 配置驱动板传感器透传周期 |
| `sensor watch <id>` | id | 持续打印传感器数据 (Hall/力矩/膝关节/着地开关) |
| `sensor read <id>` | id | 读一帧传感器数据 |
| `sensor stop <id>` | id | 停止传感器透传 + 停止看板 |

### 2.6 底层调试类

| 指令 | 参数 | 功能 |
|------|------|------|
| `sdoread <id> <0xIndex> [subidx]` | 0x100A, 0x6041… | 通用 SDO 读 (读驱动板任意对象字典) |
| `sdowrite <id> <0xIndex> <subidx> <value> <size>` | 完整参数 | 通用 SDO 写 (写驱动板任意对象字典) |

### 2.7 控制模式一览

| 模式 | ID | CLI | 适用场景 |
|------|-----|-----|---------|
| Profile Position (PP) | 1 | `mode 1 pp` | 点对点角度运动, 梯形速度曲线 |
| Profile Velocity (PV) | 2 | `mode 1 pv` | 速度控制, 梯形加速 |
| Cyclic Sync Position (CSP) | 3 | `mode 1 csp` | 上位机插补, 每周期给目标位置 |
| Cyclic Sync Velocity (CSV) | 4 | `mode 1 csv` | 上位机插补, 每周期给目标速度 |
| Current (Iq) | 5 | `mode 1 cur` | 力矩控制 / 零力示教 |
| MIT Impedance | 6 | `mit 1 …` | 阻抗控制 (柔顺交互, 步行) |

---

## 三、exo_node 接口（算法进程 → 系统侧）

```
┌──────────────────────────────────────────────────────────────┐
│   算法进程 (算法同事负责)                                      │
│                                                              │
│   只需要一个头文件: exo_shm.h                                  │
│   不需要知道: CANopen / SDO / PDO / CANFD / 透传              │
│                                                              │
│   读数据:                                                     │
│     fb = shm->fb_buffer[shm->active_idx]                     │
│     fb.motor[0]  → 右腿反馈 (位置/速度/电流/温度/状态)        │
│     fb.motor[1]  → 左腿反馈                                   │
│     fb.imu       → 姿态数据                                   │
│                                                              │
│   写命令:                                                     │
│     shm->mailbox.cmd.value_right = torque_right;             │
│     shm->mailbox.cmd.value_left  = torque_left;              │
│     shm->algo_heartbeat++;           // 100Hz 心跳            │
│                                                              │
│   更新率: 200Hz (反馈) / 1KHz (控制)                          │
│   延迟:   < 5ms 端到端                                        │
└──────────────────────────────────────────────────────────────┘
                         ↕ /dev/shm/exo_shm (POSIX SHM)
┌──────────────────────────────────────────────────────────────┐
│   exo_node (系统侧负责)                                       │
│                                                              │
│   控制线程: mailbox.cmd → motor_hal_set_torque() → PDO→CANFD │
│   上报线程: feedback缓存 → fb_buffer双Buffer                  │
│   安全监控: algo_heartbeat 200ms无变化 → 力矩清零             │
│                      500ms无变化 → DS402 Shutdown             │
└──────────────────────────────────────────────────────────────┘
```

---

## 四、技术指标

| 指标 | 数值 | 说明 |
|------|------|------|
| 物理层 | CANFD 仲裁1Mbps / 数据5Mbps | 标准帧 11bit |
| 控制周期 | 1ms (1KHz) | RT 线程 SCHED_FIFO prio=90 |
| 数据上报周期 | 5ms (200Hz) | 双 Buffer 共享内存 |
| 控制延迟 | < 50μs | PDO write() → CANFD 发送 |
| 反馈读取延迟 | ~几十 ns | PI mutex 缓存拷贝 |
| SDO 参数读写 | 50~200ms | 同步阻塞, 2次重试 |
| 支持电机数 | 最多 16 | CAN 节点 ID 可配 |
| 控制模式 | 6 种 | PP/PV/CSP/CSV/Current/MIT |
| 编译产物 | libmotor_hal.a (~50KB) | 纯 C11, 零外部依赖 |
| 实时内核 | PREEMPT_RT | 控制抖动 < 100μs |

---

| 协议栈 | 手写 CANopen (417行) | motor_hal_c 完整实现 (16文件) |
| 参数轮询 | 200μs TIMER 12步 SDO | TPDO 同步上报 (零轮询) |
| 调试工具 | 串口助手 | motor_tool (30+ CLI命令) |
| 上位机 | 无 | Web UI + motor_tool 双通道 |
| 实时性 | 裸机中断 | PREEMPT_RT 内核 |
| 扩展性 | 改代码 | 加模块 = 加文件, 不改旧代码 |

---

## 六、后续开发计划

| 阶段 | 内容 | 优先级 |
|------|------|--------|
| P0 | NMT Start 缺失修复 (startup 后自动切 Operational) | 高 |
| P0 | 上位机 Web UI 开发 (控制面板 + 实时监控) | 高 |
| P1 | IMU SPI 驱动集成 (ICM45608) | 中 |
| P1 | 气压计 I2C 驱动集成 (QMP6990) | 中 |
| P1 | 校准流程联调 (左右腿对齐) | 中 |
| P2 | 算法进程联调 (步态控制) | 低 |
| P2 | 整机测试 (外骨骼穿戴) | 低 |
