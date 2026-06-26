# stark_periph_manager_node 测试指导

> 版本: feature/exo-node @ 2026-06-09  
> 目标: RV1126B + CANFD + 2髋关节电机 (巨蟹 ID=1,2)

---

## 1. 测试链路

```
┌──────────┐  SHM mailbox  ┌──────────────────┐  PDO 64B广播  ┌──────┐
│ algo_sim │ ────────────→ │ stark_periph_     │ ──────────→  │ 电机 │
│ (模拟算法)│ ←──────────── │ manager_node      │ ←──────────  │ 1,2  │
└──────────┘  SHM fb_buf   │ (motor_node)      │ 反馈帧0x300  └──────┘
                           └──────────────────┘
```

**启动顺序**:
1. 配置 CANFD 接口
2. 启动 motor_node
3. 给电机上电
4. 电机 bootup → motor_node 自动 startup (INIT→DISCOVERY→READY)  
5. 启动 algo_sim → 自动握手 → RUNNING
6. 查看数据: perf_test / 终端日志

---

## 2. 前置条件

### 2.1 电机上电

确保 2 个电机 (ID=1, 右髋 / ID=2, 左髋) 已接通 48V 电源和 CAN 总线。

### 2.2 配置 CANFD

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
ip -details link show can0
# 确认: bitrate 1000000, dbitrate 5000000, fd on
```

### 2.3 编译 (交叉编译到 RV1126B)

```bash
cd joint-motor-hal-c-v2

# 编译 motor_hal 库
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake
make -j4

# 编译 exo_node
cd ../exo_node
mkdir -p build_arm && cd build_arm
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake
make -j4

# 产物:
#   stark_periph_manager_node   ← motor_node 主程序
#   algo_sim                    ← 模拟算法 (单独编译)
#   perf_test                   ← SHM 性能测试 (单独编译)

# 编译测试工具
cd ../test
# algo_sim: g++ -O2 -o algo_sim algo_sim.cpp -I.. -lrt -lpthread
# perf_test: g++ -O2 -o perf_test perf_test.cpp -I.. -lrt -lpthread
```

### 2.4 部署到 RV1126B

```bash
scp stark_periph_manager_node root@<rv1126b_ip>:/data/
scp algo_sim       root@<rv1126b_ip>:/data/
scp perf_test      root@<rv1126b_ip>:/data/
scp ../config/exo_config.json root@<rv1126b_ip>:/data/config/stark/
```

---

## 3. 测试步骤

### 终端 1: 启动 motor_node

```bash
ssh root@<rv1126b_ip>

cd /data
sudo ./stark_periph_manager_node
```

**预期输出**:

```
[INFO] stark_periph_manager_node starting...
[CanDispatcher] CANFD can0: arb=1M data=5M ✓
[CanDispatcher] registered 2 motors (defaults)
[CanDispatcher] ready
[main] CANFD ready, INIT done
[main] mock sensor (IMU+Baro) ready
[RT] Thread started: SCHED_FIFO prio=90 period=1000us cpu=2,3
[main] RT worker started (1KHz, SCHED_FIFO 90)
[main] log drain thread started
[StateMachine] INIT → DISCOVERY
[StateMachine] enter DISCOVERY — waiting for motors...
```

### 终端 2: 给电机上电

**此时给电机上 48V 电源**。电机发送 bootup 帧到 CAN 总线。

motor_node 终端预期输出:

```
[StateMachine] auto-startup: 1 motor(s) done
[StateMachine] auto-startup: 1 motor(s) done
[StateMachine] all 2 motors online (0x03) → READY
[StateMachine] DISCOVERY → READY
[main] node ready
[main]   state: READY
[main]   motors online: 0x03
```

### 终端 2: 启动模拟算法

```bash
ssh root@<rv1126b_ip>
cd /data
sudo ./algo_sim
```

**预期输出**:

```
[algo_sim] SHM at 0x..., starting 1KHz loop...
[algo_sim] loop=1000 | online=0x03 | state=5 | seq=1000 | ...
[algo_sim] loop=2000 | online=0x03 | state=5 | seq=2000 | ...
```

motor_node 终端预期输出:

```
[StateMachine] READY → ENABLED
[StateMachine] ENABLED → RUNNING
LatencyTrace[1000] | fb_read: min=3 avg=5 max=12 us | fb_total: avg=17 us | ctrl: avg=22 us
```

### 终端 3: 查看数据

```bash
ssh root@<rv1126b_ip>
cd /data
./perf_test
```

**预期输出:**

```
┌─────────────────────────────────────────────┐
│              LIVE SHM STATUS                │
├─────────────────────────────────────────────┤
│ active_idx       : 0                        │
│ node_state       : 5 (RUNNING)              │
│ motor_online     : 0x03                     │
│ motor_severity   : 0                        │
│ mailbox.seq      : 12345                    │
├── 耗时追踪 ─────────────────────────────────┤
│  反馈T1→T4 : avg=17us  max=35us             │
│  控制T5→T6 : avg=22us  max=45us             │
│  fb_cache读 : avg=5us  max=12us             │
│  SHM写入    : avg=2us                       │
│  周期超限   : 0                              │
│  样本数     : 5000                           │
├─────────────────────────────────────────────┤
│ Motor 1: pos= 16384 vel=  12 cur= 200mA     │
│ Motor 2: pos=-8192  vel=  -8 cur= 200mA     │
│ IMU:   roll=0.05 pitch=5.10 yaw=0.01        │
│ Baro:  1013.27 hPa, 25.01°C                 │
└─────────────────────────────────────────────┘
```

### 停止测试

```bash
# 终端 2: Ctrl+C 停 algo_sim
# 终端 1: Ctrl+C 停 motor_node
```

---

## 4. 验证清单

| 检查项 | 如何验证 | 预期 |
|------|------|------|
| CANFD 通信 | motor_node 启动日志 | `CANFD can0 opened` |
| 电机 bootup | 上电后终端输出 | `auto-startup: 1 motor(s) done` |
| 电机在线 | motor_node 终端 | `all 2 motors online (0x03)` |
| DS402 使能 | startup 后查询 | motor_node 日志 `OPERATION_ENABLED` |
| SHM 创建 | ls 检查 | `ls -la /dev/shm/exo_shm` 存在 |
| algo→motor_node | algo_sim 输出 | `seq` 递增 |
| motor_node→PDO | 电机转动/位置变化 | `perf_test` motor[0].position 变化 |
| motor 反馈→SHM | perf_test | motor[0].position / velocity 有非零值 |
| IMU mock | perf_test | imu.roll/pitch/yaw 有值 |
| 气压计 mock | perf_test | baro.pressure_hpa ~1013 |
| 延迟追踪 | motor_node 终端每1秒 | `LatencyTrace[1000]` 统计行 |
| 延迟追踪关闭 | `#define EXO_LATENCY_TRACE 0` 重编译 | 无 LatencyTrace 输出 |
| 安全监控 | 停止 algo_sim 看 motor_node | `SAFETY WARN: algo timeout` 然后 `SAFETY FAULT` |

---

## 5. 常见问题

### 5.1 电机未在线

```
[StateMachine] enter DISCOVERY — waiting for motors...
(一直不切 READY)
```

**排查**:
1. 电机是否已上电？(CAN 总线 48V)
2. CAN 接线是否正常？
3. `dmesg | grep can` 看 CAN 控制器状态
4. `candump can0` 看有没有 CAN 帧 (电机上电后应看到 `701#00` bootup 帧)

### 5.2 SCHED_FIFO 失败

```
[RT] SCHED_FIFO failed (need root/CAP_SYS_NICE)
```

**解决**: 用 `sudo` 运行。若已 root 仍失败，检查内核是否编译了 PREEMPT_RT:
```bash
uname -a | grep PREEMPT
```

### 5.3 算法连接后电机不转

**排查**:
1. `perf_test` 看 `motor_severity` 是否为 0
2. `motor_online` 是否为 `0x03`
3. `mailbox.seq` 是否递增
4. 电机是否在工作模式下 (algo_sim 发的是 torque=200mA，电机应有微弱力)

### 5.4 延迟异常大

**排查**:
1. `perf_test` 看 `周期超限` 是否 >0
2. 检查 CPU 负载: `htop`
3. 确认 RT 线程是否在 SCHED_FIFO: `ps -eo pid,cls,rtprio,comm | grep stark`
4. CPU isolation 是否配置: `cat /proc/cmdline | grep isolcpus`

---

## 6. 不使用模拟算法时的手动控制

如果没有 algo_sim，可以用 motor_tool 通过 Unix socket 控制电机:

```bash
# 终端 1: 启动 daemon
motor_tool daemon can0 &

# 终端 2: 启动电机
motor_tool startup 1
motor_tool startup 2

# 终端 2: 力矩控制
motor_tool torque 1 500     # 电机1: 500mA
motor_tool torque 2 -300    # 电机2: -300mA

# 终端 2: 读取反馈
motor_tool read angle 1
motor_tool read all 1

# 终端 2: 停止
motor_tool stop
```

---

## 7. 性能基准 (期望值)

| 指标 | 非RT (SCHED_OTHER) | RT (SCHED_FIFO) |
|------|:---:|:---:|
| 反馈路径 T1→T4 | ~30-50μs | ~12-20μs |
| 控制路径 T5→T6 | ~40-80μs | ~15-25μs |
| 周期抖动 | ±500-2000μs | ±50-100μs |
| 周期超限 | 偶尔 | 0 |
| PDO 下发延迟 | ~50μs | ~25μs |

**验证方法**: 

```
Non-RT: 注释 SetThreadRt() 中的 pthread_setschedparam 调用, 重编译运行
RT:     默认编译运行
```

对比 `LatencyTrace[1000]` 输出的 avg/max 值。预期 RT 模式下延迟明显更低且抖动更小。

---

**文档版本**: v1.0 | **创建**: 2026-06-09

---

## 8. CPU 核心分配

### 四核 A35 分配方案

```
                    isolcpus=2,3
                    ┌───────────────┐    ┌───────────────┐
Core 0:              │ Core 1:       │    │ Core 2:       │ Core 3:
内核 + 中断 + IRQ    │ 非 RT 任务池  │    │ 算法进程       │ RT worker
                    │               │    │ SCHED_FIFO 90  │ SCHED_FIFO 90
                    │ 主线程        │    │ (同事负责)     │ CAN recv
                    │ log drain     │    │               │ SCHED_FIFO 85
                    │ ROS/WEB       │    │               │
                    │ IOT/配网/OTA  │    │               │
                    │ 其他非RT进程  │    │               │
                    └───────────────┘    └───────────────┘
```

| 核心 | 隔离 | 跑什么 | 调度策略 |
|------|:---:|------|:---:|
| Core 0 | ❌ | 内核 + 中断 | kernel |
| Core 1 | ❌ | 全部非 RT 任务 | CFS |
| Core 2 | ✅ | 算法进程 | SCHED_FIFO 90 |
| Core 3 | ✅ | RT worker + CAN recv | SCHED_FIFO 90+85 |

### 关键原则

1. **RT 线程各占一个核** — 同优先级 `SCHED_FIFO` 不抢占，挤一起会饿死
2. **中断走 Core 0** — `echo 1 > /proc/irq/<can_irq>/smp_affinity`
3. **非 RT 进程绑 Core 0,1** — `taskset -c 0,1 <cmd>` 避免干扰隔离核
4. **motor_node 只绑 Core 3** — `cpu_affinity = {3, -1}` (代码默认)
5. **算法绑 Core 2** — 同事自己设置

### 启动非 RT 进程的推荐做法

```bash
# 所有非 RT 服务都用 taskset 限制到 core 0,1
taskset -c 0,1 nohup /usr/bin/iot_service &
taskset -c 0,1 nohup /usr/bin/ota_daemon &
taskset -c 0,1 ./webserver &
```

### 内核 cmdline 参考

```
isolcpus=2,3 irqaffinity=0,1 rcu_nocbs=2,3 nohz_full=2,3
```

- `isolcpus=2,3`: 隔离 core 2,3
- `irqaffinity=0,1`: 默认中断只走 core 0,1
- `rcu_nocbs=2,3`: RCU 回调不在 core 2,3 上执行
- `nohz_full=2,3`: core 2,3 不需要时关闭 tick (降低抖动)

