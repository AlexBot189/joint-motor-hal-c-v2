# 外骨骼 CANFD 框架 — 已知问题与技术陷阱

> 来源: 设计文档审查 + 代码审查  
> 版本: 跟随 exo_periph_design.md  
> 更新: 2026-06-06

---

## 一、已修复 (v1.1 → v1.2 → v1.3)

### 1. active_idx 非原子变量 (ARMv8 弱内存序)
- **现象**: `active_idx = write` 不保证 `fb_buffer` 先于 `active_idx` 对算法线程可见
- **后果**: 算法读到半帧数据，左右腿不同步，步态计算错误，极难复现
- **修复**: `atomic_uint` + `memory_order_release/acquire`

### 2. Mailbox 撕裂写
- **现象**: `cmd.motor_id/cmd/cmd/value` 三个字段分开写，RT 线程可能读到跨周期混合数据
- **后果**: 算法写了新力矩但 RT 线程读到旧力矩,或反之
- **修复**: Lock-Free Snapshot (`seq_begin/cmd/seq_end`)

### 3. MotorCommand 无法左右独立控制
- **现象**: 外骨骼有摆动相和支撑相，左右力矩必然不同，但只有一个 `value` 字段
- **后果**: 只能双电机同步控制，算法同事无法实现步态
- **修复**: 改为 `value_right` / `value_left`

### 4. cmd_seq 参与停机判定导致误判
- **现象**: 用户站立时力矩恒定，算法不更新 `cmd_seq`，安全监控误判"算法输出停滞"
- **后果**: 力矩清零 → 人摔倒
- **修复**: `cmd_seq` 只记日志，`heartbeat` 是唯一停机依据

### 5. 安全监控线程直接写 PDO (sockfd 多线程竞争)
- **现象**: 控制线程和安全监控线程同时调 `motor_hal_set_torque()`，同时 `write` 同一个 socket fd
- **后果**: POSIX 未定义行为 (实际 Linux CAN RAW 单帧原子, 但不可依赖)
- **修复**: 安全监控设 `atomic_flag`，控制线程在 loop 开头检查并处理

### 6. 启动探测阻塞等电
- **现象**: 节点启动后调 `motor_hal_wait_bootup(timeout=5000)`，电机还没上电时每 5 秒卡一次
- **后果**: 启动流程卡死，系统就绪延迟
- **修复**: 只探测一次，不通则标记 offline，recv 线程收到 Bootup 后事件驱动通知

### 7. 电机在线判断依赖 Bootup
- **现象**: 节点重启时电机已在线，Bootup 早已发过，不会再收到
- **后果**: `startup` 永远不执行
- **修复**: PDO 反馈帧收到 或 SDO 读成功 = 在线

### 8. READY_FOR_CONTROL 无超时保护
- **现象**: 校准完成后等算法握手 `algo_ready==1`，算法没启动就永久等待
- **后果**: 系统卡死
- **修复**: 30s 超时 → FAULT

### 9. 校准完成立即使能 (安全问题)
- **现象**: 校准成功 → 直接 DS402 使能 → 开透传
- **后果**: 算法未启动时电机已使能，缺乏安全保护
- **修复**: 改为 `CALIB_DONE` → `READY_FOR_CONTROL` → 等 `algo_ready==1` → 才 ENABLE

### 10. FeedbackFrame 缺版本号
- **现象**: 结构体字段变更后算法侧不重新编译
- **后果**: 内存布局不匹配，crash
- **修复**: `EXO_SHM_VERSION` 版本号 + 启动时断言

### 11. 电机索引无显式绑定
- **现象**: 依赖 `motor_hal_add_motor` 注册顺序决定 `motor[0]/[1]` 的物理 ID
- **后果**: 注册顺序变更后 `motor[0]` 不再是右腿
- **修复**: `#define MOTOR_IDX_RIGHT 0` / `MOTOR_IDX_LEFT 1`

---

## 二、仍需注意 (开发时检查)

### 12. FeedbackFrame 的 `_pad` 未精确计算
- **描述**: `_pad[4006]` 是估算值，结构体字段固定后需精确计算确保 64KB 对齐
- **检查方法**: `assert(sizeof(ExoShm) == 65536)`
- **状态**: ⚠️ 开发时验证

### 13. PI mutex 在接收线程/上报线程之间的优先级倒置
- **描述**: 上报线程(80)持 feedback 缓存锁时被接收线程(85)抢占
- **影响**: 极小 (持锁 < 1μs)，可接受
- **状态**: ✅ 无需处理

### 14. 上报线程何时开始写双 Buffer
- **描述**: 节点启动即开始 vs 校准完成后开始?
- **决定**: 启动即开始写（包含 `calib_state` 字段），算法侧自检 `calib_state != 2` → 跳过
- **状态**: ✅ 设计已确认

### 15. IMU 数据来源不明确
- **描述**: `FeedbackFrame.imu` 字段由谁填充？motor_node 内嵌 imu_hal，还是独立 imu_node 写共享内存？
- **状态**: ⚠️ 待确定（IMU 驱动开发时决定）

### 16. GPIO 按键引脚未指定
- **描述**: 校准按键、模式切换按键的 GPIO 引脚未在文档中定义
- **状态**: ⚠️ 硬件确定后补充

### 17. CanDispatcher 的 `NotifyObserver` 并发
- **描述**: ROS 主线程和状态机线程可能同时调 `NotifyObserver`
- **petrobot 原框架**: `UartDispatcher` 单线程消费，无并发问题
- **新框架**: 增加 ROS 旁路后，RT 上报线程 push ConcurrentQueue，ROS 主线程 pop → NotifyObserver。单消费者，安全
- **状态**: ✅ 保持单消费者模式

### 18. ConcurrentQueue batch size
- **描述**: RT 上报线程 200Hz push FeedbackFrame，ROS 主线程 pop 并序列化为 ROS message
- **风险**: ROS 主线程 pop 速度跟不上 push 速度时队列膨胀
- **缓解**: moodycamel::ConcurrentQueue 是 lock-free，内存用完后 allocate 新 block，不会阻塞 RT 线程
- **状态**: ✅ 设计上安全

---

## 三、硬件/环境依赖

### 19. RT-Linux 内核配置
- `CONFIG_PREEMPT_RT=y`
- 线程优先级 `SCHED_FIFO` + `mlockall(MCL_CURRENT|MCL_FUTURE)` (防页面换出)
- 控制线程 `sched_setaffinity` 绑定到独立 CPU core (隔离噪声)

### 20. CANFD 接口配置
```bash
ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
ip link set can0 up
```
- 终端电阻: 120Ω × 2 (总线两端), 测量应为 60Ω
- 看门狗: 500ms 超时自锁, motor_hal_startup 中自动关闭

### 21. 共享内存权限
```bash
# motor_node 以 root 运行时创建
/dev/shm/exo_shm  # 权限 0666
# 算法进程必须能读(fb)+写(mailbox)
```

### 22. 校准前必须满足的条件
- 电机已上电 (物理按键供电)
- 电机已 startup (心跳+关狗+固件验证完成, auto_enable=false)
- `motor_online == 0x03`
- 编码器正常工作

---

**文档位置**: `doc/exo_periph_design.md` (主设计文档)  
**问题追踪**: 本文档 (开发时查阅)
