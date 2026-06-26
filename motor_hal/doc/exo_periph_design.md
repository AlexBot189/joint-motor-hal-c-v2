# 外骨骼外设节点框架设计文档 v2.0

> 项目: 外骨骼机器人 RV1126B CANFD 控制  
> 核心技术: RT-Linux + POSIX SHM + Double Buffer + Mailbox + 状态机 + 安全监控  
> 日期: 2026-06-07

---

## 1. 总体架构

```
┌──────────────────────────────────────────────────────────────┐
│                     算法进程 (算法同事负责)                     │
│                                                              │
│  算法主循环 (1KHz, SCHED_FIFO 90):                            │
│    读 shm→fb_buffer[active]→motor[0]  右腿 (ID=1)            │
│    读 shm→fb_buffer[active]→motor[1]  左腿 (ID=2)            │
│    步态计算 → 力矩                                              │
│    写 shm→mailbox.cmd = {TORQUE, right=2000, left=-1500}     │
│    写 shm→algo_heartbeat++               ★ 100Hz心跳          │
│                                                              │
│  只需要: #include "exo_shm.h"                                 │
│  不需要: motor_hal / CANopen / SDO / PDO                      │
└──────────────────────────┬───────────────────────────────────┘
                           │ /dev/shm/exo_shm  (唯一共享内存)
                           │ 算法读: fb_active + 状态
                           │ 算法写: mailbox.cmd + algo_heartbeat
═══════════════════════════╪══════════════════════════════════════
                           │
┌──────────────────────────┼───────────────────────────────────┐
│                      motor_node (系统侧)                      │
│                                                              │
│  RT 路径 (实时, SCHED_FIFO)                                    │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ RT控制线程 (prio=90, 1KHz)                               │ │
│  │   shm→mailbox.cmd → motor_hal_set_torque() → PDO → CANFD │ │
│  │                                                          │ │
│  │ CAN接收线程 (prio=85, 事件驱动, HAL内部)                   │ │
│  │   select()→read()→dispatch→fb_cache/sensor_cache          │ │
│  │                                                          │ │
│  │ RT上报线程 (prio=80, 200Hz)                               │ │
│  │   fb/sensor缓存 → FeedbackFrame → 写双Buffer SHM          │ │
│  │                                                          │ │
│  │ 安全监控线程 (prio=75, 100Hz)                              │ │
│  │   algo_heartbeat 200ms无变化 → torque=0                   │ │
│  │   algo_heartbeat 500ms无变化 → DS402 Shutdown             │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                              │
│  非 RT 路径 (SCHED_OTHER)                                      │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ 状态机线程                                                │ │
│  │   INIT→探测→READY→校准→ENABLED→握手→RUNNING              │ │
│  │                                                          │ │
│  │ ROS Service (编译可选)                                     │ │
│  │   /motor/startup /enable /calib /config                  │ │
│  │                                                          │ │
│  │ ROS Topic / WebServer (编译可选, 从 SHM pull)              │ │
│  │   /motor/feedback  ← RosListener 独立线程读 SHM           │ │
│  │   WebSocket /ws    ← WsListener 独立线程读 SHM            │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                              │
│  依赖:                                                        │
│    libmotor_hal.a  — CANFD HAL (PDO + SDO + 反馈 + 透传 + 校准) │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. 数据分层 (三层)

```
┌─────────────────────────────────────────────────────────┐
│  SHM (实时核心数据, 必须)                                 │
│  反馈帧、mailbox、心跳、motor_online/motor_fault/        │
│  calib_state/node_state                                  │
│  更新频率: 200Hz (上报) / 1KHz (mailbox)                │
│  消费者: 算法进程、RosListener(pull)、WsListener(pull)   │
├─────────────────────────────────────────────────────────┤
│  ROS Topic (调试流, 编译可选)                             │
│  /motor/feedback  ← RosListener 独立线程从 SHM pull      │
│  /motor/state     ← 状态变化时推                         │
│  用途: rosbag 录制、rqt_plot 可视化                      │
├─────────────────────────────────────────────────────────┤
│  ROS Service (配置/命令, 编译可选)                        │
│  /motor/startup /enable /calib /set_pid ...             │
│  直接调 motor_hal SDO API，不走 SHM                      │
└─────────────────────────────────────────────────────────┘
```

**原则**: SHM 只管实时数据流。调试和配置走 ROS。非 RT 消费者自己从 SHM pull。

---

## 3. 实时 vs 非实时数据流

```
                        实时路径                          非实时路径
                    ┌───────────────┐              ┌─────────────────┐
                    │   SHM 共享内存  │              │  ROS Service     │
                    │   (零拷贝)      │              │  /motor/startup  │
                    │                │              │  /motor/calib    │
                    │ 反馈帧 ← 上报   │              │  /motor/enable   │
                    │ mailbox → 控制  │              └────────┬────────┘
                    │ heartbeat → 安全 │                      │
                    └───────┬────────┘              ┌────────┴────────┐
                            │                      │  motor_hal SDO  │
                    ┌───────┴───────┐              │  API (同步阻塞)  │
                    │  motor_hal    │              │  50~200ms       │
                    │  PDO 通道     │              │  SCHED_OTHER    │
                    │  write() <100μs│             └────────┬────────┘
                    └───────┬───────┘                       │
                            │                               │
                    ┌───────┴───────────────────────────────┴──────┐
                    │              CANFD 总线 (同一根)              │
                    └──────────────────────────────────────────────┘

关键: SDO 和 PDO 共用 CAN 总线，但在 HAL 层是独立的代码路径。
SDO 慢(阻塞等响应)不阻塞 PDO 快写。
```

---

## 4. 线程分配

| 线程 | 策略 | 优先级 | 周期 | CPU占用 |
|------|------|:---:|------|:---:|
| 算法(外进程) | SCHED_FIFO | 90 | 1KHz | - |
| RT控制线程 | SCHED_FIFO | 90 | 1KHz | <0.4% |
| CAN接收线程 | SCHED_FIFO | 85 | 事件驱动 | <0.1% |
| RT上报线程 | SCHED_FIFO | 80 | 200Hz | <0.2% |
| 安全监控线程 | SCHED_FIFO | 75 | 100Hz | <0.02% |
| 状态机 | SCHED_OTHER | 0 | 事件驱动 | <1% |
| ROS/Web | SCHED_OTHER | 0 | 按需 | <1% |

**优先级原则**: 控制 > 接收 > 上报 > 安全 > 非RT

---

## 5. 共享内存 (唯一一个)

### 布局

```
/dev/shm/exo_shm  (64KB)

version (4B)
active_idx (4B, atomic)    ← 双Buffer切换索引
fb_buffer[0] (~140B)       ← 算法读
fb_buffer[1] (~140B)       ← 上报写
mailbox { seq_begin, cmd, seq_end }   ← Lock-Free Snapshot
algo_heartbeat (8B, atomic)           ← 100Hz递增
cmd_seq (8B, atomic)                  ← mailbox写计数
algo_ready (8B, atomic)               ← 算法握手
motor_online / calib_state / motor_enabled / motor_fault / node_state (各1B)
```

### Double Buffer

```
上报线程: 写 fb_buffer[active^1] → atomic_store(active_idx, active^1, release)
算法线程: idx = atomic_load(active_idx, acquire) → 读 fb_buffer[idx]

ARMv8弱序下 release/acquire 保证算法看到完整帧, 不会出现撕裂读。
```

### Mailbox Lock-Free Snapshot

```
算法写:
  seq_begin = ++writer_seq;     // release
  cmd.value_right = 2000;
  cmd.value_left  = -1500;
  seq_end = writer_seq;         // release

motor_node 读:
  begin = seq_begin;  end = seq_end;  // acquire
  if (begin != end) return;     // 正在写, 跳过
  if (begin == last_seq) return; // 相同, 跳过
  读 cmd ← 安全
```

**不需要锁。** 进程间 mutex 会影响 RT — 用原子变量保证。

---

## 6. 状态机 (6状态)

```
INIT → READY → CALIBRATING → ENABLED → RUNNING → FAULT
  ↑      ↑                                 │        │
  └──────┴─────────────────────────────────┘────────┘
```

| 状态 | 触发 | 动作 | 下一状态 |
|------|------|------|----------|
| INIT | 进程启动 | 打开CAN、注册电机、创SHM、探测在线 | READY / FAULT |
| READY | 电机全部在线 | 写 motor_online, 等校准命令 | CALIBRATING |
| CALIBRATING | 命令触发 | 设零位、poll 检测 ±1° | ENABLED / FAULT |
| ENABLED | 校准成功 | 等 algo_ready (超时10s→FAULT) | RUNNING / FAULT |
| RUNNING | 算法握手 | 使能、开透传、启动 RT 线程 | FAULT |
| FAULT | 故障 | 停机、清零、标记故障码 | READY |

**实现**: 简单 switch-case + enter/exit 钩子。不用 scxmlcc（外骨骼 6 个状态不需要分层状态机）。

---

## 7. 启动流程

```
motor_node 启动
  ├─ motor_hal_init + add_motor + recv_start
  ├─ exo_shm_open(create=true)
  ├─ 状态机线程:
  │   ├─ [INIT] probe 探测电机 (反馈缓存 + SDO ping)
  │   ├─ [READY] 等 calib 命令
  │   ├─ [CALIBRATING] motor_calib_start → poll
  │   ├─ [ENABLED] 等 algo_ready (10s超时)
  │   └─ [RUNNING] motor_hal_enable + set_mode(CURRENT) + sensor_config
  │                → 启动 RT控制/上报/安全线程
```

---

## 8. 日志方案

RT 线程不能直接写磁盘/控制台（write 可能阻塞）。用 lock-free 队列：

```
RT 线程:
  snprintf(buf, 256, ...) → g_log_queue.enqueue(buf)  ← <1μs

日志线程(非RT):
  循环 pop → fprintf / write 文件  ← I/O 不阻塞 RT
```

使用 moodycamel::ConcurrentQueue（单头文件 BSD 许可，petrobot 已验证）。

---

## 9. 故障恢复

### motor_fault 定义

| Bit | 含义 | 触发 | 自动处理 |
|-----|------|------|----------|
| 0 | 算法心跳丢失 | heartbeat 200ms无变化 | torque=0 |
| 1 | 算法死亡 | heartbeat 500ms无变化 | DS402 Shutdown |
| 2 | 输出停滞 | cmd_seq 200ms无变化 | torque=0 |
| 3 | CAN断线 | 2s未收到CAN帧 | 标记offline |
| 4 | 校准超时 | calib poll超时 | calib_state=3 |
| 5 | 编码器异常 | position恒定3s | 标记FAULT |
| 6 | 驱动器过温 | temp > 80°C | torque=0 |
| 7 | 握手超时 | 10s无algo_ready | 标记FAULT |

**原则**: 可自动恢复不停机，硬件异常人工介入。

---

## 10. CMake 编译控制

```cmake
option(ENABLE_ROS  "Build ROS adapter" OFF)
option(ENABLE_WEBSERVER "Build crow WebServer" OFF)

target_sources(exo_node PRIVATE src/main.cpp src/CanDispatcher.cpp ...)

if(ENABLE_ROS)
    target_sources(exo_node PRIVATE src/RosAdapter.cpp)
    target_link_libraries(exo_node ${catkin_LIBRARIES})
endif()

if(ENABLE_WEBSERVER)
    target_sources(exo_node PRIVATE src/WsAdapter.cpp)
    target_link_libraries(exo_node crow)
endif()
```

生产环境不编译 ROS 和 WebServer。

---

## 11. 实时性能

| 指标 | 要求 | 说明 |
|------|------|------|
| PDO 下发延迟 | <50μs | write() + CAN 控制器 |
| 反馈读取延迟 | <1μs | PI mutex + memcpy 缓存 |
| 控制周期抖动 | <100μs | cyclictest 验证 |
| 闭环总延迟 | ~6.5ms | 算法→控制→CAN→反馈→SHM→算法 |
| SHM 更新延迟 | <5ms | 200Hz 上报周期 |
| RT CPU 占用 | <1% | 4个RT线程大部分时间在sleep |

---

**版本**: v2.0 | **更新**: 2026-06-07 23:00
**v2.0改动**:
- 重新整理架构: 三层数据分层(SHM/ROS Topic/ROS Service)
- 简化观察者: SHM pull 代替 push 通知
- 线程模型: 4RT + 1非RT，去掉了独立ROS线程
- 状态机: 6状态 switch-case，不用 scxmlcc
- 新增: 日志方案、CMake 编译控制、性能表
- 明确: 非 RT 消费者从 SHM pull 数据
