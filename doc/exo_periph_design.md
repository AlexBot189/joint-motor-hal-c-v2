# 外骨骼外设节点框架设计文档 v1.3

> 项目: 外骨骼机器人 RV1126B CANFD 控制  
> 框架: 基于 petrobot_periph_manager 量产验证框架  
> 核心技术: RT-Linux + POSIX SHM + Double Buffer + Mailbox + Lock-Free Snapshot + 安全监控  
> 日期: 2026-06-05

---

## 1. 总体架构

```
┌──────────────────────────────────────────────────────────────────┐
│                    算法进程 (算法同事负责)                          │
│                                                                    │
│  算法主循环 (1KHz, SCHED_FIFO 90):                                 │
│    读 shm→active_idx → 检测双Buffer切换                            │
│    读 shm→fb_buffer[active]→motor[0]  右腿 (右电机ID=1)            │
│    读 shm→fb_buffer[active]→motor[1]  左腿 (左电机ID=2)            │
│    读 shm→fb_buffer[active]→imu       姿态数据                     │
│    步态计算 → 力矩                                                   │
│    写 shm→mailbox.cmd = {TORQUE, motor_id=0, value=2000}          │
│    写 shm→mailbox.seq++                                            │
│    写 shm→algo_heartbeat++                    ★ 100Hz心跳          │
│                                                                    │
│  不需要: include motor_hal.h                                       │
│  不需要: 知道 CANopen/SDO/PDO/透传                                  │
│  只需要: include exo_shm.h                                        │
└──────────────────────────┬───────────────────────────────────────┘
                           │ /dev/shm/exo_shm  (共享内存)
                           │ 算法读: fb_active + imu
                           │ 算法写: mailbox.cmd + algo_heartbeat
═══════════════════════════╪══════════════════════════════════════════
                           │          系统侧 (你的工作)
┌──────────────────────────┼───────────────────────────────────────┐
│          motor_node (基于 petrobot 框架)                           │
│                                                                    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │ CanDispatcher : IMsgInternalDispatcher (观察者模式核心)      │  │
│  │                                                              │  │
│  │  6 个线程:                                                    │  │
│  │                                                              │  │
│  │  [RT] 控制线程 (prio=90, 1KHz):                              │  │
│  │    mailbox.cmd → motor_hal_set_torque() → PDO → CANFD       │  │
│  │                                                              │  │
│  │  [RT] CAN接收线程 (prio=85, 事件驱动, HAL内部):              │  │
│  │    CAN帧 → _dispatch_frame → SDO队/反馈缓存/传感器缓存       │  │
│  │                                                              │  │
│  │  [RT] 上报线程 (prio=80, 200Hz):                             │  │
│  │    feedback缓存 → 组装FeedbackFrame → 写双Buffer备用区        │  │
│  │    sensor缓存 → 组装FeedbackFrame                             │  │
│  │    写完切换 active_idx                                        │  │
│  │                                                              │  │
│  │  [RT] 安全监控线程 (prio=75, 100Hz):  ★ 新增                 │  │
│  │    读 shm→algo_heartbeat                                     │  │
│  │    200ms无变化 → torque=0                                     │  │
│  │    500ms无变化 → DS402 Shutdown                               │  │
│  │                                                              │  │
│  │  [非RT] 状态机线程 (prio=0):                                  │  │
│  │    消费 event_queue → motor_hal_startup/calib/enable          │  │
│  │    主动探测电机在线 (Bootup + SDO读双保险)                      │  │
│  │                                                              │  │
│  │  [非RT] ROS线程 (prio=0):                                    │  │
│  │    ROS Service: /motor/startup /motor/calib /motor/config    │  │
│  │    ROS Topic: /motor/feedback /motor/state (调试用)           │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  依赖:                                                             │
│    libmotor_hal.a    — CANFD HAL + SDO/PDO/反馈/透传 + 校准 (已验证)  │
│    libimu_hal.a      — ICM45608 SPI (后续)                         │
│    libsensor_hal.a   — 气压计/霍尔 (后续)                          │
└────────────┬─────────────────────────────────────────────────────┘
             │
    ┌────────┴────────┐
    │   CANFD 5Mbps    │
    ├─────────────────┤
    │  右电机 ID=1     │
    │  左电机 ID=2     │
    └─────────────────┘
```

---

## 2. 共享内存布局 (最终版)

```c
// exo_shm.h — 系统侧和算法侧共享的唯一接口文件

#include <stdint.h>
#include <stdatomic.h>

#define EXO_SHM_NAME   "/exo_shm"
#define EXO_SHM_SIZE   (64 * 1024)
#define EXO_SHM_VERSION 1              // ★ 版本号, 升级时递增

// ─── 反馈数据: 一帧 = 左右电机 + IMU, Double Buffer保证一致性 ───

typedef struct {
    // 电机反馈 (0x300 反馈帧, 大端)
    int16_t  position;       // 编码器count, [-32768,32767] → [-180°,180°]
    int16_t  velocity;       // RPM
    int16_t  current_iq;     // mA
    uint16_t error_code;     // 故障码
    int16_t  temperature;    // 0.1°C
    uint8_t  status_byte;    // bit7:使能 bit6:抱闸 bit5:错误 bit4:到位
    uint8_t  mode;           // 当前控制模式

    // 传感器透传 (0x680, 小端bit-packed)
    uint16_t hall_adc0;      // 线性霍尔A, U12, 0~4095
    uint16_t hall_adc1;
    uint16_t hall_adc2;
    uint16_t force_raw;      // DF181力矩, U14, 0~16383
    uint16_t knee_adc;       // 膝关节电位器, U12, 0~4095
    uint8_t  key_landing;    // 着地开关, 0=低 1=高
    uint8_t  data_valid;     // 力矩有效标志

    uint64_t timestamp_us;
} MotorData;

// 索引约定: motor[0]=右腿(CAN ID=1), motor[1]=左腿(CAN ID=2)
// 代码显式绑定, 不依赖motor_hal_add_motor注册顺序
#define MOTOR_IDX_RIGHT  0
#define MOTOR_IDX_LEFT   1

typedef struct {
    MotorData motor[2];      // [0]=右(ID=1), [1]=左(ID=2)

    struct {
        float    roll, pitch, yaw;
        float    acc_x, acc_y, acc_z;
        float    gyro_x, gyro_y, gyro_z;
        uint64_t timestamp_us;
    } imu;
} FeedbackFrame;

// ─── 命令: Mailbox 模式 (只存最新值) ───

typedef enum {
    EXO_CMD_TORQUE  = 1,     // 力矩, value=mA
    EXO_CMD_SPEED   = 2,     // 速度, value=RPM×100
    EXO_CMD_POS     = 3,     // 位置, value=°×100
} ExoCmdType;

typedef struct {
    uint8_t  cmd;              // EXO_CMD_TORQUE/SPEED/POS
    int32_t  value_right;       // 右腿 (CAN ID=1)
    int32_t  value_left;        // 左腿 (CAN ID=2)
    uint64_t timestamp_us;
} MotorCommand;
// 外骨骼左右独立, Mailbox一次覆盖两侧

typedef struct {
    atomic_uint_least64_t seq_begin;  // ★ 写前递增
    MotorCommand cmd;                   // 命令数据
    atomic_uint_least64_t seq_end;    // ★ 写后 = seq_begin
} MotorMailbox;  // Lock-Free Snapshot: 读端seq_begin==seq_end保证完整性

// ─── 共享内存总结构 ───

typedef struct {
    // ─── 版本 (系统写, 算法读) ───
    uint32_t        version;          // ★ EXO_SHM_VERSION, 升级检测

    // ─── 双Buffer反馈区 (系统写, 算法读) ───
    atomic_uint     active_idx;       // ★ release/acquire 语义, ARMv8弱序安全
    FeedbackFrame   fb_buffer[2];     // 双Buffer

    // ─── Mailbox命令区 (算法写, 系统读) ───
    MotorMailbox    mailbox;           // Lock-Free Snapshot

    // ─── 安全监控 (算法写, 系统读) ───
    atomic_uint_least64_t  algo_heartbeat;  // ★ 100Hz递增 (线程存活)
    atomic_uint_least64_t  cmd_seq;         // ★ 每次写mailbox后递增 (结果有效)
    atomic_uint_least64_t  algo_ready;      // ★ 算法握手: 1=已就绪 (第六处)

    // ─── 状态区 (系统写, 算法读) ───
    uint8_t         motor_online;       // bit0=右 bit1=左
    uint8_t         calib_state;        // 0=空闲 1=校准中 2=完成 3=超时
    uint8_t         motor_enabled;       // bit0=右使能 bit1=左使能
    uint8_t         motor_fault;         // 故障标志 (bit定义见故障恢复章)
    uint8_t         node_state;          // ★ 系统状态 (OFF/READY/CALIB/ENABLED/RUNNING/FAULT)

    uint8_t         _pad[4006];          // 对齐64KB
} ExoShm;

// ─── 工具函数 ───
ExoShm* exo_shm_open(const char* name, bool create);
// 算法侧: assert(shm->version == EXO_SHM_VERSION); // 升级检测
void    exo_shm_close(ExoShm* shm);
```

---

## 3. 线程模型与优先级 (最终版)

```
线程                    核心                    策略         优先级     周期
══════════════════════════════════════════════════════════════════════
算法线程 (外进程)        步态计算                SCHED_FIFO   90       1KHz
RT控制线程              消费Mailbox→PDO         SCHED_FIFO   90       1KHz
CAN接收线程 (HAL内部)    CANFD收帧+分发           SCHED_FIFO   85       事件驱动
RT上报线程              反馈→双Buffer           SCHED_FIFO   80       200Hz
安全监控线程             算法心跳→紧急停机         SCHED_FIFO   75       100Hz
状态机线程              startup/calib/enable     SCHED_OTHER  0        事件驱动
ROS主线程               调试服务                 SCHED_OTHER  0        按需
```

**优先级设计原则:**
- 控制线程最高 → PDO 写电机的确定性延迟最重要
- 接收线程序二 → CAN 帧不能丢
- 上报线程第三 → 数据完整性, 但丢一帧问题不大
- 安全监控第四 → 频率低 (100Hz), 但要高于非RT线程
- 非RT线程最低 → SDO 阻塞不影响 RT

---

## 4. 启动流程 (Bootup双保险)

```
系统上电
  │
  ▼
motor_node 启动
  ├─ motor_hal_init("can0", 1M, 5M)          // 打开CANFD
  ├─ motor_hal_add_motor(cfg_r)              // 注册右电机 (auto_enable=false)
  ├─ motor_hal_add_motor(cfg_l)              // 注册左电机
  ├─ motor_hal_recv_start()                  // 启动接收线程
  ├─ exo_shm_open(create=true)               // 创建共享内存
  │
  └─ 状态机线程 → 主动探测
       for (id = 1; id <= 2; id++) {
           // PDO反馈帧正常接收 = 驱动板已在线
           if (motor_hal_get_feedback(id, &fb) == 0 && fb.status_byte != 0) {
               online_mask |= (1 << (id-1));
               continue;
           }
           // 主动 SDO 读 + Ping
           uint32_t ver = 0;
           if (motor_hal_sdo_read_u32(id, 0x100A, 0, &ver) == 0) {
               online_mask |= (1 << (id-1));
               continue;
           }
           // 都不通: 电机未上电, 不阻塞, recv线程收到Bootup后事件通知
           printf("Motor %d: offline (no PDO, no SDO)\n", id);
       }
       shm->motor_online = online_mask;
                    │
  ════════════════════╪══════════════════════════════════
                    │  ⬆ 物理按键给电机供电
                    │    驱动板上电 → 自动发 0x701/0x702
                    ▼
  两个电机都在线 (motor_online == 0x03)
    → motor_hal_startup(id, auto_enable=false)  // SDO: 心跳+关狗+固件
    → 停在 SWITCH_ON_DISABLED (READY状态, 不使能)
    → shm->motor_enabled = 0x00
                    │
  ════════════════════╪══════════════════════════════════
                    │  ⬆ 按键触发校准 (GPIO中断 → event_queue)
                    ▼
  状态机线程消费 EVENT_CALIB
    → motor_calib_start(id_r=1, id_l=2, timeout=10000)
       ├─ 设零位 (SDO写)
       ├─ 进入 CHECKING
       │   poll: motor_hal_get_feedback() → 读缓存
       │   左右 |angle| < 1.0°? → DONE
       │   超时 10s? → TIMEOUT → shm->calib_state=3
       └─ DONE:
            ├─ shm->calib_state = 2
            ├─ shm->node_state = READY_FOR_CONTROL
            └─ 等待算法握手 (shm->algo_ready == 1)
                 ↓ 算法就绪
            ├─ DS402使能 (0x06→0x07→0x0F)   ← 至此才使能
            ├─ 切电流模式 (0x6060=0x0A)
            ├─ 开传感器透传 250Hz
            ├─ shm->node_state = RUNNING
            └─ shm->motor_enabled = 0x03
                    │
                    ▼
  RT上报线程开始写双Buffer
  算法检测到 active_idx 变化 → 读数据 → 步态 → 写mailbox
  RT控制线程消费mailbox → PDO下发
  安全监控线程持续检测 algo_heartbeat
```

**关键:**
- 节点重启时 (电机已在线), 主动SDO探测 + Bootup等待双保险
- 状态机线程只消费事件, GPIO回调只发事件, 互不阻塞

---

## 5. 数据流 (三条通路)

```
通路1: 实时反馈 (系统 → 算法, 200Hz)
════════════════════════════════════════
HAL接收线程                RT上报线程                    共享内存              算法进程
──────────────────────────────────────────────────────────────────────────────
CAN帧 → _dispatch_frame
 ├─ 0x300 → fb_cache       motor_hal_get_feedback()
 ├─ 0x680 → sensor_cache   motor_hal_get_sensor()
 └─                        ↓
                      组装 FeedbackFrame
                      写 fb_buffer[active^1]              fb_buffer[active^1] 填满
                      atomic_store_explicit(&shm->active_idx, active^1, memory_order_release)                                     atomic_load_explicit(&shm->active_idx, memory_order_acquire)  != last 检测到切换  // ARMv8弱序安全
                      → ConcurrentQueue (ROS旁路)         读 fb_buffer[active] ✓ 完整帧
                      延迟: ~5ms (200Hz)

通路2: 实时控制 (算法 → 系统, 1KHz)
════════════════════════════════════════
算法进程                    共享内存                     RT控制线程              硬件
──────────────────────────────────────────────────────────────────────────────
步态 → 力矩
  mailbox.cmd = {...}      覆盖写                        mailbox.seq 变化
  mailbox.seq++            原子递增                      读 mailbox.cmd
  algo_heartbeat++         原子递增                      motor_hal_set_torque()
                                                         → PDO 大端组帧
                                                         → write(sockfd)       → CANFD
                                                         延迟: ~50μs

通路3: 安全监控 (系统自检, 100Hz)
════════════════════════════════════════
RT安全线程 读 shm→algo_heartbeat
  200ms 无变化 → motor_hal_set_torque(0)  // 力矩清零
  500ms 无变化 → motor_hal_disable(id)    // DS402 Shutdown
                → shm→motor_enabled = 0
```

---

## 6. CanDispatcher 实现概要 (~300行)

```cpp
class CanDispatcher : public IMsgInternalDispatcher {
    motor_hal_t    *m_hal;
    ExoShm         *m_shm;

    std::thread     m_ctrlThread;     // RT prio=90
    std::thread     m_reportThread;   // RT prio=80
    std::thread     m_safetyThread;   // RT prio=75
    std::thread     m_stateThread;    // 非RT, event消费
    std::thread     m_rosPollThread;  // 非RT, ROS旁路

    moodycamel::ConcurrentQueue<Event>   m_events;   // 状态机事件队列
    moodycamel::ConcurrentQueue<FeedbackFrame> m_rosQueue;

    std::atomic_bool m_running;

    // IMsgInternalDispatcher 接口实现
    bool InitDispatcher() override;
    bool DestroyDispatcher() override;
    void Send(const std::string& data) override;
    void RegisterObserver(ListenerType, shared_ptr<IListener>) override;
    void NotifyObserver(const boost::any&) override;

private:
    // RT线程
    void RtCtrlLoop();      // 1KHz: 读mailbox → PDO
    void RtReportLoop();    // 200Hz: 读缓存 → 双Buffer + ROS旁路
    void RtSafetyLoop();    // 100Hz: 检测algo_heartbeat

    // 非RT线程
    void StateMachineLoop(); // 消费event → startup/calib/enable
    void RosPollLoop();      // 从 rosQueue → NotifyObserver
};
```

**与 UartDispatcher 对比:**

| UartDispatcher | CanDispatcher | 减少 |
|---------------|--------------|------|
| recv线程: read串口+帧解析 | CAN接收线程: HAL内部自动 | 简化 |
| send线程: write串口 | 控制线程: PDO写socket | 简化 |
| heart线程: 串口心跳 | 不需要 | 删除 |
| Decode*Msg: 15+个函数 | 1个: FeedbackFrame组装 | 大幅简化 |
| ~1500行 | ~350行 | -75% |

---

## 7. 状态机事件模型

```cpp
enum EventType {
    EVENT_POWER_ON = 0,    // 电机上电 (Bootup收到或主动探测)
    EVENT_MOTOR_READY,     // startup完成, 进入 READY 状态
    EVENT_CALIB,           // 按键触发校准
    EVENT_CALIB_DONE,      // 校准成功
    EVENT_CALIB_FAIL,      // 校准超时/失败
    EVENT_ALGO_TIMEOUT,    // 算法心跳超时 → 力矩清零
    EVENT_ALGO_DEAD,       // 算法死透 → 脱使能
};

// GPIO回调 (快进快出)
void gpio_calib_callback() {
    m_events.enqueue({EVENT_CALIB});
}

// 状态机线程 (允许阻塞)
void StateMachineLoop() {
    Event ev;
    while (m_running) {
        if (m_events.try_dequeue(ev)) {
            switch (ev.type) {
            case EVENT_CALIB:
                motor_calib_start(m_calib, &calib_cfg);
                while (motor_calib_poll(m_calib) == MOTOR_CALIB_CHECKING)
                    sleep_ms(20);
                if (motor_calib_get_state(m_calib) == MOTOR_CALIB_DONE)
                    m_events.enqueue({EVENT_CALIB_DONE});
                else
                    m_events.enqueue({EVENT_CALIB_FAIL});
                break;
            case EVENT_CALIB_DONE:
                m_shm->calib_state = 2;
                m_shm->motor_enabled = 0x03;
                break;
            case EVENT_CALIB_FAIL:
                m_shm->calib_state = 3;
                break;
            }
        }
        sleep_ms(10);
    }
}
```

---

## 8. 安全监控详细逻辑

```cpp
void RtSafetyLoop() {   // SCHED_FIFO 75, 100Hz
    uint64_t last_hb = 0;
    uint64_t stall_us = 0;
    bool torque_cleared = false;
    bool motor_disabled = false;

    while (m_running) {
        uint64_t hb = __atomic_load_n(&m_shm->algo_heartbeat, __ATOMIC_RELAXED);

        // cmd_seq: 只记录日志, 不参与停机判定
        // 理由: 站立时力矩恒定, 算法可能不更新cmd_seq, 不能用它判异常
        uint64_t cs = atomic_load_explicit(&m_shm->cmd_seq, memory_order_relaxed);
        static uint64_t last_cs = 0;
        if (cs == last_cs) { /* 正常: 力矩恒定不变 */ }
        last_cs = cs;

        // 仅用 heartbeat 判断算法存活
        if (hb == last_hb && m_shm->motor_enabled) { → 算法异常
            // 心跳无变化 + 电机已使能 → 算法可能挂了
            if (stall_us == 0) stall_us = now_us();

            uint64_t elapsed = now_us() - stall_us;

            if (elapsed > 200000 && !torque_cleared) {    // 200ms
                ECO_ERROR("ALGO HEARTBEAT LOST: torque→0");
                motor_hal_set_torque(m_hal, 1, 0);
                motor_hal_set_torque(m_hal, 2, 0);
                torque_cleared = true;
                m_shm->motor_fault |= 0x01;  // 故障: 算法心跳丢失
            }

            if (elapsed > 500000 && !motor_disabled) {   // 500ms
                ECO_ERROR("ALGO DEAD: DS402 Shutdown");
                atomic_store(&m_safety_shutdown, 1);  // 设标志, 状态机线程消费
                motor_disabled = true;
                m_shm->motor_enabled = 0x00;
                m_shm->motor_fault |= 0x02;  // 故障: 算法死亡,已停机
            }
        } else if (hb != last_hb) {
            // 心跳恢复
            last_hb = hb;
            stall_us = 0;
            if (torque_cleared) {
                ECO_INFO("ALGO RECOVERED: torque restored");
                torque_cleared = false;
                m_shm->motor_fault &= ~0x01;
            }
            if (motor_disabled) {
                ECO_WARN("Motor disabled by safety. Manual restart required.");
                // 不复使能, 需要人工干预
            }
        }

        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_10ms, NULL);
    }
}
```

---

## 9. ROS 非实时接口

```yaml
services:
  /motor/startup:        # 启动电机 (等效 GD32 V-B/C)
    request:  {motor_id: uint8}
    response: {ok: bool}
  /motor/enable:         # 使能 (等效 GD32 V-F 反操作)
    request:  {motor_id: uint8}
    response: {ok: bool}
  /motor/disable:        # 脱使能 (等效 GD32 V-F)
    request:  {motor_id: uint8}
    response: {ok: bool}
  /motor/calib:          # 校准 (等效 GD32 V-D)
    request:  {id_r: uint8, id_l: uint8, timeout_ms: uint16}
    response: {ok: bool, state: uint8}
  /motor/sdo_write:      # 通用SDO写 (调试)
    request:  {motor_id: uint8, index: uint16, sub: uint8, value: uint32, size: uint8}
    response: {ok: bool}
  /motor/sdo_read:       # 通用SDO读 (调试)
    request:  {motor_id: uint8, index: uint16, sub: uint8}
    response: {ok: bool, value: uint32}
  /motor/config:         # 配置
    request:  {watchdog_disable: bool, heartbeat_ms: uint16, ...}
    response: {ok: bool}

topics (调试用, 非实时):
  /motor/feedback:       # 电机反馈 (低频, ~10Hz)
  /motor/sensor:         # 传感器透传数据
  /imu/data:             # IMU数据
  /motor/state:          # 状态变更通知
```

---

## 10. 算法侧使用示例

```c
#include "exo_shm.h"
#include <time.h>

int main() {
    ExoShm *shm = exo_shm_open(EXO_SHM_NAME, false);  // 打开已创建的
    uint32_t last_active = shm->active_idx;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    while (1) {
        // === 1. 等新反馈帧 (自旋等双Buffer切换) ===
        while (atomic_load_explicit(&shm->active_idx, memory_order_acquire) == last_active) {
            __asm__ volatile("yield");
        }
        last_active = atomic_load_explicit(&shm->active_idx, memory_order_acquire);

        // === 2. 读完整一帧 (Double Buffer保证一致性) ===
        FeedbackFrame *fb = &shm->fb_buffer[last_active];
        int16_t pos_r = fb->motor[MOTOR_IDX_RIGHT].position;   // 右腿 ID=1
        int16_t pos_l = fb->motor[MOTOR_IDX_LEFT].position;    // 左腿 ID=2
        int16_t vel_r = fb->motor[0].velocity;
        int16_t vel_l = fb->motor[1].velocity;
        float   pitch = fb->imu.pitch;
        float   roll  = fb->imu.roll;
        uint8_t sw_r  = fb->motor[0].key_landing;  // 着地

        // === 3. 步态计算 ===
        int32_t torque_r = gait_calculate_r(pos_r, vel_r, pitch, roll, sw_r);
        int32_t torque_l = gait_calculate_l(pos_l, vel_l, pitch, roll, sw_r);

        // === 4. 下发指令 (Mailbox覆盖写) ===
        // Lock-Free Snapshot: 左右力矩独立, 一次写入
        atomic_fetch_add(&shm->mailbox.seq_begin, 1);
        shm->mailbox.cmd.cmd         = EXO_CMD_TORQUE;
        shm->mailbox.cmd.value_right = torque_r;   // 右 ID=1
        shm->mailbox.cmd.value_left  = torque_l;   // 左 ID=2
        atomic_store(&shm->mailbox.seq_end, atomic_load(&shm->mailbox.seq_begin));

        // === 5. 心跳 + 命令序列号 (100Hz) ===
        static int heartbeat_count = 0;
        if (++heartbeat_count >= 10) {
            heartbeat_count = 0;
            atomic_fetch_add(&shm->algo_heartbeat, 1);
        }
        atomic_fetch_add(&shm->cmd_seq, 1);  // ★ 证明算法结果在更新
    }
}
```

---

## 11. 目录结构

```
exo_periph_manager/
├── CMakeLists.txt                       # petrobot框架 + motor_hal链接
├── src/
│   ├── main.cpp                         # 入口: Factory → CanDispatcher → spin
│   │
│   ├── interface/                       # ★ 从 petrobot 复制
│   │   ├── Defines.hpp                  #   加 MOTOR_FEEDBACK/MOTOR_SENSOR 到MsgType
│   │   ├── IListener.hpp               #   不改
│   │   ├── IMsgInternalDispatcher.hpp   #   不改 (UartDispatcher的父接口)
│   │   └── Factory.hpp/cpp             #   加 CanDispatcher 创建分支 + 单例
│   │
│   ├── protocol/
│   │   ├── CanDispatcher.hpp            # ★ 新建, ~80行
│   │   └── CanDispatcher.cpp            # ★ 新建, ~350行
│   │       ├─ InitDispatcher: hal_init + shm_open + 启动6个线程
│   │       ├─ RtCtrlLoop:     mailbox → motor_hal_set_torque()
│   │       ├─ RtReportLoop:   feedback → 双Buffer + ConcurrentQueue
│   │       ├─ RtSafetyLoop:   algo_heartbeat → torque→0 / DS402 Shutdown
│   │       ├─ StateMachineLoop: event_queue → startup/calib/enable
│   │       └─ RosPollLoop:    ConcurrentQueue → NotifyObserver
│   │
│   ├── ros_adapter/
│   │   ├── RosAdapter.hpp/cpp           # PubMotorFeedback + MotorService
│   │   └── RosTopic.hpp                 # (可选) 调试topic定义
│   │
│   ├── shm/
│   │   ├── exo_shm.h                    # ★ 新建: 共享内存结构 + 双Buffer + Mailbox
│   │   └── exo_shm.cpp                  # ★ 新建: shm_open/mmap (~30行)
│   │
│   ├── config/
│   │   └── Defines.hpp                  # + MotorOption + CalibOption
│   │
│   └── 3rd_party/                       # ★ 从 petrobot 复制
│       ├── concurrentqueue/concurrentqueue.h    # moodycamel lock-free
│       └── nlohmann/json.hpp                   # JSON (调试)
│
└── launch/
    └── motor_node.launch
```

---

## 12. 与现有项目的关系

```
代码来源                    处理方式               最终文件数
══════════════════════════════════════════════════════════
petrobot/interface/         复制, 只改Defines      ~5个文件
petrobot/protocol/IMsg*    复制, 不改              ~1个文件
petrobot/ros_adapter/       复制, +30行             ~2个文件
petrobot/3rd_party/         直接复制                 ~2个文件
motor_hal_c                 编译为libmotor_hal.a    不动 (17个文件, 含校准)
exo_shm                     新建                    +2个文件
CanDispatcher               新建                    +2个文件
```

---

## 13. 算法同事交付清单

```
交付文件:
  ├── exo_shm.h                  # 接口头文件 (就是文档)
  └── /dev/shm/exo_shm           # 运行时由 motor_node 自动创建

算法同事需要:
  1. #include "exo_shm.h"
  2. exo_shm_open(EXO_SHM_NAME, false);  // 打开
  3. while (1) { 读 active_idx; 读 fb_buffer; 步态; 写 mailbox; 写 heartbeat; }
  4. gcc -O2 gait_control.c -lrt

算法同事不需要:
  ✗ motor_hal.h / motor_hal_types.h / canopen_frames.h
  ✗ CANopen 协议: SDO/PDO/NMT/COB-ID
  ✗ 透传协议: 0x680 bit-packed 解析
  ✗ 反馈帧: 0x300 大端字节序
  ✗ CANFD 初始化/配置
  ✗ ROS (可选)

通信语义:
  读: 等待 active_idx 变化 → 读 fb_buffer[active] → 保证左右腿同帧
  写: 覆盖 mailbox.cmd → mailbox.seq++ → 系统读最新
  安全: algo_heartbeat++ (100Hz) → 系统监控, 心跳丢失自动停机
```

---

## 14. 关键设计决策

| 决策 | 选择 | 原因 |
|------|------|------|
| IPC | POSIX SHM (`shm_open+mmap`) | 零拷贝, 确定性延迟, 工业标准 |
| 反馈 | Double Buffer | 保证左右腿数据一致性, 消除撕裂读 |
| 命令 | Mailbox | 只读最新, 不排队, 适合状态驱动控制 |
| 安全 | 算法心跳 → 力矩清零 → 脱使能 | 人机共融设备必须, 200ms/500ms两级 |
| 优先级 | Control>RX>Feedback>Safety | 写电机最高, 收帧次之 |
| 状态机 | 事件队列解耦GPIO和SDO | GPIO快进快出, SDO可阻塞 |
| 探测 | Bootup + SDO读双保险 | 覆盖节点重启场景 |
| 校准 | 超时退出 | 10s, 霍尔/编码器故障不停死 |

---

**版本**: v1.1 | **更新**: 2026-06-05 23:30
**修正**: GPIO解耦 + Bootup双保险 + 安全监控 + 线程优先级 + 状态机事件

## 15. 系统状态机 (v1.2 新增)

```
                    ┌─────────────────────────────────┐
                    │        系统状态机 (node_state)     │
                    └─────────────────────────────────┘

   ┌──────┐   节点启动    ┌───────┐   双电机在线    ┌───────┐
   │ OFF  │─────────────→│ READY │───────────────→│ CALIB │
   └──────┘              └───┬───┘  startup完成   └───┬───┘
                             │                        │
                             │                        ├─ 校准成功 → CALIB_DONE
                             │                        ├─ 超时10s  → FAULT
                             │                        └─ 编码器故障 → FAULT
                             │
                             ▼
                      ┌─────────────────┐
                      │ READY_FOR_CTRL  │  ← ★ 等算法握手
                      └────────┬────────┘
                               │ algo_ready == 1
                               ▼
                      ┌─────────────────┐
                      │   ENABLED       │  ← DS402使能 + 开透传
                      └────────┬────────┘
                               │ 算法心跳正常
                               ▼                         安全监控触发
                      ┌─────────────────┐           ┌─────────────────┐
                      │   RUNNING       │──────────→│     FAULT       │
                      │   (正常运行)     │           │                 │
                      └────────┬────────┘           │ CAN断线          │
                               │                    │ 编码器异常        │
                               │ algo_heartbeat     │ 驱动器过温        │
                               │ 丢失 / cmd_seq     │ 算法超时          │
                               │ 停滞               │ 算法心跳200ms    │
                               └────────────────────│ 算法死亡500ms    │
                                                    └────────┬────────┘
                                                             │
                                                             ▼
                                                      ┌─────────────┐
                                                      │ FAULT 恢复   │
                                                      │ (见§16)     │
                                                      └─────────────┘

状态转换表:
  源状态           事件                        目标状态          动作
  ──────────────────────────────────────────────────────────────────
  OFF             节点启动                     READY           创建SHM, 初始化CANFD
  READY           startup(心跳+关狗)完成        CALIB            (等按键)
  CALIB           校准成功                     CALIB_DONE       版本号写1
  CALIB           校准超时                     FAULT            motor_fault|=0x10
  CALIB           编码器故障                    FAULT            motor_fault|=0x20
  CALIB_DONE      算法握手(algo_ready==1)       ENABLED         DS402使能, 开透传
  CALIB_DONE      超时30s无握手                FAULT            motor_fault|=0x80
  ENABLED         心跳正常+cmd_seq正常          RUNNING          RT线程开始消费mailbox
  ENABLED         心跳丢失200ms                FAULT            torque→0
  RUNNING         心跳丢失200ms                FAULT            torque→0, motor_fault|=0x01
  RUNNING         心跳丢失500ms                FAULT            DS402 Shutdown, motor_fault|=0x02
  RUNNING         cmd_seq停滞200ms             FAULT            torque→0, motor_fault|=0x04
  RUNNING         CAN断线(收不到帧2s)           FAULT            motor_fault|=0x08
  FAULT           按键恢复                     READY           重新startup+校准
```

---

## 16. 故障恢复流程 (v1.2 新增)

### 故障码定义 (motor_fault)

| Bit | 含义 | 触发条件 | 自动处理 |
|-----|------|---------|---------|
| 0 | 算法心跳丢失 | algo_heartbeat 200ms无变化 | torque→0 |
| 1 | 算法死亡 | algo_heartbeat 500ms无变化 | DS402 Shutdown |
| 2 | 算法输出停滞 | cmd_seq 200ms无变化 | torque→0 |
| 3 | CAN通信断开 | 2s未收到任何CAN帧 | 标记offline, 等待恢复 |
| 4 | 校准超时 | motor_calib_poll超时10s | calib_state=3 |
| 5 | 编码器异常 | position始终不变(3s) | 标记FAULT |
| 6 | 驱动器过温 | fb.temperature > 80°C | torque→0, 等降温 |
| 7 | 算法握手超时 | CALIB_DONE后30s无algo_ready | 标记FAULT, 需人工确认算法启动 |

### 故障恢复策略

```
故障类型          恢复方式              恢复后状态        需要操作
─────────────────────────────────────────────────────────────────
算法心跳丢失      torque→0后自动恢复    RUNNING           无需人工 (心跳恢复即重建)
算法死亡          人工重启校准           READY             按键重新校准
算法输出停滞      torque→0后自动恢复    RUNNING           无需人工
CAN断线           自动重连               READY→重新校准     电机重新上电
校准超时           人工重试              READY             按键重新校准
编码器异常         人工检查后重试         READY             检查硬件+重新校准
过温              自动等降温后恢复       RUNNING           无需人工
算法握手超时       人工确认后触发         READY→重新校准     确认算法启动后按键
```

**原则: 可自动恢复的不停机, 硬件异常需人工介入, 进入FAULT后脱使能电机。**

---

## 17. 算法握手协议 (v1.2 新增)

校准成功后**不立即使能电机**, 等待算法确认就绪。

```
motor_node 侧:                   算法侧:
═════════════════════            ═══════════════════
CALIB_DONE
shm->node_state = READY_FOR_CTRL
shm->calib_state = 2
                                 检测到 calib_state == 2
                                 初始化步态模型
                                 设置控制参数
                                 atomic_store(&shm->algo_ready, 1);  ← ★ 握手
                                 ↓
检测到 algo_ready == 1:
  motor_hal_enable(1)            (电机使能, 算法已就绪)
  motor_hal_enable(2)
  motor_hal_sensor_config(1,4,3) (开透传)
  motor_hal_sensor_config(2,4,3)
  shm->motor_enabled = 0x03
  shm->node_state = RUNNING
  ↓
RT控制线程: 开始消费mailbox       ↓
(mailbox可能还是空的,              开始写 mailbox → PDO生效
 这是安全的 — 第一次seq变化后
 才开始发PDO)
```

**超时保护: CALIB_DONE 后 10 秒无 algo_ready → FAULT, motor_fault|=0x80。**

---

## 18. FeedbackFrame 版本兼容 (v1.2 新增)

```c
#define EXO_SHM_VERSION 1

// 算法侧启动时:
ExoShm* shm = exo_shm_open(EXO_SHM_NAME, false);
if (shm->version != EXO_SHM_VERSION) {
    fprintf(stderr, "FATAL: SHM version mismatch (got %u, expected %u)\n",
            shm->version, EXO_SHM_VERSION);
    exit(1);
}

// motor_node 侧创建时:
shm->version = EXO_SHM_VERSION;  // 首次创建时写
```

**升级规则:**
- 新增字段: 加到结构体末尾, 递增 `EXO_SHM_VERSION`
- 修改字段: 递增 `EXO_SHM_VERSION`, 算法侧必须重新编译
- 删除字段: 保留为 `_reserved` 占位, 递增 `EXO_SHM_VERSION`

---

## 19. 实时性能指标 (v1.2 新增)

| 指标 | 要求 | 测试方法 |
|------|------|---------|
| 控制周期 | 1ms (1KHz) | `cyclictest -p90 -t1 -i1000` |
| 控制抖动 | <100μs | `cyclictest` histogram |
| CAN发送延迟 | <50μs | PDO write() 前后 clock_gettime 差 |
| CAN接收延迟 | <200μs | 帧到达 → dispatch 完成时间 |
| SDO读写超时 | 500ms | motor_hal_sdo_write 内部超时 |
| 校准总超时 | 10s | motor_calib_start 的 timeout_ms |
| 算法心跳周期 | 10ms (100Hz) | algo_heartbeat 递增频率 |
| 安全监控检测延迟 | <200ms (心跳丢失→torque清零) | 故意kill算法进程测试 |
| 数据上报频率 | 200Hz (5ms) | fb_buffer active_idx 切换频率 |
| 共享内存更新延迟 | <5ms (上报同步) | seq_num 从写到算法读到的时间差 |
| 故障恢复最大延迟 | 500ms (心跳丢失→Shutdown) | 安全监控的最坏情况延迟 |
| CAN帧丢失率 | <0.01% | can_rx_err_cnt / can_rx_total |

---

**版本**: v1.3 | **更新**: 2026-06-06 00:30
**v1.2修正**: active_idx原子变量 + Mailbox Lock-Free Snapshot + algo_heartbeat+cmd_seq双重检查 + 状态机图 + 故障恢复 + 算法握手 + 版本号 + 性能指标
