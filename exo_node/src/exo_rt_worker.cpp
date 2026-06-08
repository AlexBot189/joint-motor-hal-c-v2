/*
 * @file exo_rt_worker.cpp
 * @brief RT 工作线程实现 — 控制下发 + 反馈上报 + 安全监控 (三合一)
 *
 * ## 线程布局
 *
 *   RT 工作线程  (SCHED_FIFO 90, 1KHz, core 2)
 *     ├─ ProcessMailbox()     读 SHM → PDO multi_ctrl → CANFD
 *     ├─ PublishFeedback()    读 fb_cache → SHM double buffer (200Hz)
 *     └─ SafetyCheck()        seq 停滞 / 编码器 / 过温 / CAN 断线
 *
 *   CAN 接收线程  (SCHED_FIFO 85, HAL 内部)
 *     └─ 收帧 → 分发 (SDO响应/fb_cache/sensor/bootup)
 *
 * ## 关键路径
 *
 *   mailbox → multi_ctrl 路径:
 *     算法写 seq_begin+cid → RT 线程读 (atomic acquire) → multi_ctrl(cmd)
 *   → PDO 64B 广播帧 → 一帧发完所有电机 → < 20μs
 *
 *   反馈上报路径:
 *     fb_cache (PI mutex 读) → 组装 feedback_frame_t
 *   → SHM double buffer (atomic release) → 200Hz 等效
 *
 * ## 安全约束
 *
 *   - 任何阻塞调用 (SDO/sleep/malloc) 禁止出现在 Run() 循环中
 *   - 仅访问 m_hal 的 non-blocking API (get_feedback / multi_ctrl / set_* PDO)
 *   - 锁仅使用 PI mutex 或 lock-free atomic
 */

#include "exo_rt_worker.h"

#include <cstring>
#include <ctime>
#include <sched.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <unistd.h>

namespace stark_periph_manager_node
{

/* ════════════════════════════════════════════════════════════════════
 * 构造 / 析构
 * ════════════════════════════════════════════════════════════════════ */

ExoRtWorker::ExoRtWorker(motor_hal_t* hal, exo_shm_t* shm)
    : m_hal(hal)
    , m_shm(shm)
    , m_running(false)
    , m_last_seq(0)
    , m_seq_stall_us(0)
    , m_can_last_frame_us(0)
    , m_report_divider(m_rt.report_divider)
    , m_cycle_count(0)
    , m_overrun_count(0)
{
    memset(m_last_position, 0, sizeof(m_last_position));
    memset(m_pos_stall_us, 0, sizeof(m_pos_stall_us));
    memset(m_sensor_notified, 0, sizeof(m_sensor_notified));
    m_imu_notified = false;
}

ExoRtWorker::~ExoRtWorker()
{
    if (m_running.load(std::memory_order_acquire)) {
        Stop();
    }
}

/* ════════════════════════════════════════════════════════════════════
 * Start() / Stop()
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::Start()
{
    if (m_running.load(std::memory_order_acquire)) {
        return;
    }

    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&ExoRtWorker::Run, this);
}

void ExoRtWorker::Stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

/* ════════════════════════════════════════════════════════════════════
 * Run() — RT 线程主循环
 *
 * 周期: 1000μs (1KHz)
 * 调度: SCHED_FIFO, prio=90
 * 名称: "exo_rt"
 * ──────────────────────────────────────────────────────────────────── */
void ExoRtWorker::Run()
{
    SetThreadRt();

    /* ── 初始化周期计时 ── */
    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    /* 周期纳秒 = period_us × 1000 */
    const long period_ns = (long)m_rt.period_us * 1000L;

    while (m_running.load(std::memory_order_acquire)) {

        /* ① ProcessMailbox — 读 SHM 算法命令 → multi_ctrl → PDO */
        ProcessMailbox();

        /* ② PublishFeedback — 读 fb_cache → SHM double buffer (200Hz) */
        PublishFeedback();

        /* ③ SafetyCheck — 安全监控 */
        SafetyCheck();

        m_cycle_count++;

        /* ── 精确周期休眠 (clock_nanosleep TIMER_ABSTIME) ── */
        next_wake.tv_nsec += period_ns;
        while (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_nsec -= 1000000000L;
            next_wake.tv_sec++;
        }

        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                  &next_wake, nullptr);
        if (ret != 0) {
            m_overrun_count++;
            /* 超时: 重置到当前时间, 避免雪崩 */
            clock_gettime(CLOCK_MONOTONIC, &next_wake);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════
 * ProcessMailbox() — 读 SHM mailbox → 控制下发
 *
 * Mailbox 并发协议:
 *   算法写: seq_begin++ → write cmd → seq_end = seq_begin
 *   RT 读:  read seq_begin (acquire) → if begin!=end: 正在写, 跳过
 *           if begin==m_last_seq: 无新命令, 返回
 *           else: 读 cmd → multi_ctrl → m_last_seq = begin
 *
 * 返回: true=有命令下发, false=无变化
 * ──────────────────────────────────────────────────────────────────── */
bool ExoRtWorker::ProcessMailbox()
{
    if (!m_shm) return false;

    /* 原子读取 seq_begin (acquire 语义, 保证读到算法侧已完成写入) */
    uint64_t begin = __atomic_load_n(&m_shm->mailbox.seq_begin,
                                     __ATOMIC_ACQUIRE);
    uint64_t end   = __atomic_load_n(&m_shm->mailbox.seq_end,
                                     __ATOMIC_ACQUIRE);

    /* 正在写入: 本周期跳过 */
    if (begin != end) {
        return false;
    }

    /* 无新命令 */
    if (begin == m_last_seq) {
        return false;
    }

    /* 首次收到算法命令 → 可以在这里触发 state:RUNNING */
    if (m_last_seq == 0 && begin > 0) {
        m_shm->node_state = STATE_RUNNING;
    }

    /* ── 读取命令并下发 ── */
    motor_command_t cmd = m_shm->mailbox.cmd;    /* 值拷贝, 不保护 */
    m_last_seq = begin;

    /* 转换 mailbox 格式 → multi_axis_cmd_t */
    multi_axis_cmd_t mcmd = {};
    mcmd.node_id       = cmd.motor_id;
    mcmd.enable        = true;
    mcmd.release_brake = true;
    mcmd.clear_error   = false;
    mcmd.target1       = (int16_t)cmd.value;   /* 单轴: value 直接写入 */
    mcmd.target2       = 0;
    mcmd.feedforward   = 0;

    /* 模式映射: exo_cmd_type_t → motor_mode_t */
    switch (cmd.cmd) {
    case EXO_CMD_TORQUE:
        mcmd.mode = MOTOR_MODE_CURRENT;
        break;
    case EXO_CMD_SPEED:
        mcmd.mode = MOTOR_MODE_PROFILE_VEL;
        break;
    case EXO_CMD_POS:
        mcmd.mode = MOTOR_MODE_CSP;
        break;
    default:
        mcmd.mode = MOTOR_MODE_CURRENT;  /* 默认电流模式 */
        break;
    }

    /* 一帧 multi_ctrl 下发所有轴 */
    if (m_hal) {
        motor_hal_multi_ctrl(m_hal, &mcmd, 1);

        /* 更新 CAN 最后帧时间 (控制帧也算活动) */
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        m_can_last_frame_us = (uint64_t)ts.tv_sec * 1000000ULL
                            + (uint64_t)ts.tv_nsec / 1000ULL;
    }

    return true;
}

/* ════════════════════════════════════════════════════════════════════
 * PublishFeedback() — 读 fb_cache → 写 SHM double buffer
 *
 * 频率: 每 m_report_divider (默认 5) 个周期 = 200Hz
 * 操作:
 *   1. 从 HAL fb_cache 读 1~4 号电机反馈
 *   2. (可选) 读传感器透传数据
 *   3. (可选) 读 IMU 数据
 *   4. 写入 m_shm->fb_buffer[active^1]
 *   5. atomic_store(active_idx, active^1, release)
 *
 * 双 Buffer 防撕裂: 写非活跃 Buffer, 写完再切指针
 * ──────────────────────────────────────────────────────────────────── */
void ExoRtWorker::PublishFeedback()
{
    /* 频分控制 */
    if (--m_report_divider > 0) {
        return;
    }
    m_report_divider = m_rt.report_divider;  /* 重置 */

    if (!m_hal || !m_shm) return;

    /* 非活跃 Buffer 索引 */
    uint32_t active = __atomic_load_n(&m_shm->active_idx,
                                      __ATOMIC_ACQUIRE);
    uint32_t write_idx = active ^ 1;

    feedback_frame_t* fb = &m_shm->fb_buffer[write_idx];

    /* 清空上一帧 */
    memset(fb, 0, sizeof(feedback_frame_t));

    /* ── 填充电机反馈 ── */
    for (uint8_t id = 1; id <= 4; ++id) {
        motor_feedback_t mfb;
        if (motor_hal_get_feedback(m_hal, id, &mfb) == 0) {
            uint8_t idx = id - 1;
            fb->motor[idx].position    = mfb.position;
            fb->motor[idx].velocity    = mfb.velocity;
            fb->motor[idx].current_iq  = mfb.current_iq;
            fb->motor[idx].temperature = mfb.temperature;
            fb->motor[idx].status_byte = mfb.status_byte;
            fb->motor[idx].mode        = mfb.mode;
            fb->motor[idx].error_code  = (uint8_t)(mfb.error_code & 0xFF);
        }
    }

    /* ── 填充传感器透传 (从 motor_hal_get_sensor 读取) ── */
    for (uint8_t id = 1; id <= 4; ++id) {
        motor_sensor_t s;
        int ret = motor_hal_get_sensor(m_hal, id, &s);
        if (ret == 0) {
            uint8_t idx = id - 1;
            fb->sensor[idx].hall_adc0   = s.hall_adc0;
            fb->sensor[idx].hall_adc1   = s.hall_adc1;
            fb->sensor[idx].hall_adc2   = s.hall_adc2;
            fb->sensor[idx].force_raw   = s.force_raw;
            fb->sensor[idx].knee_adc    = s.knee_adc;
            fb->sensor[idx].key_landing = s.hw_sw_pc9;
            fb->sensor[idx].data_valid  = s.data_valid;
        } else if (!m_sensor_notified[id - 1]) {
            /* 传感器未配置/不可用, 仅打印一次 */
            printf("[ExoRtWorker] sensor not configured for motor %d (ret=%d)\n",
                   id, ret);
            m_sensor_notified[id - 1] = true;
        }
    }

    /* ── 填充 IMU (从 IMU HAL 读取, 当前未集成) ── */
    /*
     * IMU (ICM45608) 通过 SPI 直连 SoC, 由独立的 imu_hal 模块采集。
     * 待 imu_hal 模块集成后, 取消注释以下代码:
     *
     *   imu_data_t imu;
     *   if (imu_read(&imu) == 0) {
     *       fb->imu = imu;
     *   }
     */
    if (!m_imu_notified) {
        printf("[ExoRtWorker] IMU read not integrated (imu_hal module pending)\n");
        m_imu_notified = true;
    }

    /* ── 时间戳 ── */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fb->timestamp_us = (uint64_t)ts.tv_sec * 1000000ULL
                     + (uint64_t)ts.tv_nsec / 1000ULL;

    /* ── 切换活跃 Buffer (release 语义, 保证写入完成后算法侧可见) ── */
    __atomic_store_n(&m_shm->active_idx, write_idx, __ATOMIC_RELEASE);
}

/* ════════════════════════════════════════════════════════════════════
 * SafetyCheck() — 安全监控 (全部内聚在 RT 线程)
 *
 * 检查项:
 *   1. seq_begin 停滞 > 200ms → torque=0, severity=WARN
 *   2. seq_begin 停滞 > 500ms → motor_hal_disable, severity=FAULT
 *   3. (TODO) 编码器位置 3s 不变 → FAULT
 *   4. (TODO) 温度 > 过温阈值 → WARN (该电机 torque=0)
 *   5. (TODO) CAN 2s 无帧 → FAULT
 *
 * 反跳逻辑: WARN 立即响应, FAULT 需持续超时才执行 shutdown。
 * ──────────────────────────────────────────────────────────────────── */
void ExoRtWorker::SafetyCheck()
{
    if (!m_shm) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL
                    + (uint64_t)ts.tv_nsec / 1000ULL;

    /* ── seq_begin 停滞检测 ── */
    if (m_last_seq > 0) {
        uint64_t begin = __atomic_load_n(&m_shm->mailbox.seq_begin,
                                         __ATOMIC_ACQUIRE);
        if (begin == m_last_seq) {
            /* seq 未推进 */
            if (m_seq_stall_us == 0) {
                m_seq_stall_us = now_us;  /* 记录停滞开始时间 */
            }

            uint64_t stall_ms = (now_us - m_seq_stall_us) / 1000ULL;

            if (stall_ms > m_safety.algo_shutdown_ms) {
                /* ── FAULT 级别: 算法严重失联 ── */
                m_shm->motor_severity = MOTOR_FAULT;
                m_shm->fault_reason   = FAULT_ALGO_TIMEOUT;

                /* 脱使能所有在线电机 */
                if (m_hal) {
                    uint8_t online = m_shm->motor_online;
                    for (uint8_t id = 1; id <= 4; ++id) {
                        if (online & (1 << (id - 1))) {
                            motor_hal_disable(m_hal, id);
                        }
                    }
                }
                m_shm->node_state = STATE_FAULT;

                fprintf(stderr, "[ExoRtWorker] SAFETY FAULT: algo timeout "
                        "%lu ms, all motors disabled\n", stall_ms);
            }
            else if (stall_ms > m_safety.algo_timeout_ms) {
                /* ── WARN 级别: 降额运行 ── */
                if (m_shm->motor_severity < MOTOR_WARN) {
                    m_shm->motor_severity = MOTOR_WARN;
                    m_shm->fault_reason   = FAULT_ALGO_TIMEOUT;

                    /* torque=0: 用 motor_hal_set_torque(0) 立即停止 */
                    if (m_hal) {
                        uint8_t online = m_shm->motor_online;
                        for (uint8_t id = 1; id <= 4; ++id) {
                            if (online & (1 << (id - 1))) {
                                motor_hal_set_torque(m_hal, id, 0);
                            }
                        }
                    }

                    fprintf(stderr, "[ExoRtWorker] SAFETY WARN: algo timeout "
                            "%lu ms, torque=0\n", stall_ms);
                }
            }
        } else {
            /* seq 恢复推进 → 清除停滞标记 */
            m_seq_stall_us = 0;
            if (m_shm->motor_severity == MOTOR_WARN &&
                m_shm->fault_reason == FAULT_ALGO_TIMEOUT)
            {
                /* 自动恢复 (仅 WARN 级, FAULT 需人工干预) */
                m_shm->motor_severity = MOTOR_OK;
                m_shm->fault_reason   = FAULT_NONE;
                printf("[ExoRtWorker] Safety recovered, severity=OK\n");
            }
        }
    }

    /* ── (TODO) 编码器停滞检测 ── */
    /*
    for (uint8_t id = 1; id <= 4; ++id) {
        motor_feedback_t fb;
        if (motor_hal_get_feedback(m_hal, id, &fb) == 0) {
            uint8_t idx = id - 1;
            if (fb.position == m_last_position[idx]) {
                if (m_pos_stall_us[idx] == 0) {
                    m_pos_stall_us[idx] = now_us;
                }
                uint64_t stall_s = (now_us - m_pos_stall_us[idx]) / 1000000ULL;
                if (stall_s > m_safety.encoder_stall_s) {
                    m_shm->motor_severity = MOTOR_FAULT;
                    m_shm->fault_reason   = FAULT_ENCODER_FAULT;
                    motor_hal_disable(m_hal, id);
                }
            } else {
                m_last_position[idx] = fb.position;
                m_pos_stall_us[idx] = 0;
            }
        }
    }
    */

    /* ── (TODO) 过温检测 ── */
    /*
    for (uint8_t id = 1; id <= 4; ++id) {
        int32_t temp;
        if (motor_hal_get_motor_temp(m_hal, id, &temp) == 0) {
            if (temp > m_safety.overtemp_celsius * 10) {
                m_shm->motor_severity = MOTOR_WARN;
                m_shm->fault_reason   = FAULT_OVERTEMP;
                motor_hal_set_torque(m_hal, id, 0);  // 该电机降额
            }
        }
    }
    */

    /* ── (TODO) CAN 断线检测 ── */
    /*
    if (m_can_last_frame_us > 0) {
        uint64_t offline_ms = (now_us - m_can_last_frame_us) / 1000ULL;
        if (offline_ms > m_safety.can_offline_ms) {
            m_shm->motor_severity = MOTOR_FAULT;
            m_shm->fault_reason   = FAULT_CAN_OFFLINE;
            // 无需 disable (已无法通信), 但需通知上层
            m_shm->node_state = STATE_FAULT;
        }
    }
    */
}

/* ════════════════════════════════════════════════════════════════════
 * SetThreadRt() — 设置 RT 线程属性
 *
 *   - 调度策略: SCHED_FIFO
 *   - 优先级:   m_rt.priority (默认 90)
 *   - 名称:     "exo_rt"
 *   - CPU 亲和: m_rt.cpu_affinity (默认 core 2)
 *
 * 注意: 需要 root 或 CAP_SYS_NICE 权限。
 * ──────────────────────────────────────────────────────────────────── */
void ExoRtWorker::SetThreadRt()
{
    /* ── 线程名称 ── */
    prctl(PR_SET_NAME, "exo_rt", 0, 0, 0);

    /* ── 调度策略 + 优先级 ── */
    struct sched_param param;
    param.sched_priority = m_rt.priority;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        fprintf(stderr, "[ExoRtWorker] pthread_setschedparam failed: %d "
                "(need root/CAP_SYS_NICE)\n", ret);
    }

    /* ── CPU 亲和 ── */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(m_rt.cpu_affinity[0], &cpuset);
    if (m_rt.cpu_affinity[1] >= 0) {
        CPU_SET(m_rt.cpu_affinity[1], &cpuset);
    }

    ret = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (ret != 0) {
        fprintf(stderr, "[ExoRtWorker] pthread_setaffinity_np failed: %d\n",
                ret);
    }

    printf("[ExoRtWorker] Thread started: SCHED_FIFO prio=%d period=%dus "
           "cpu=%d,%d\n",
           m_rt.priority, m_rt.period_us,
           m_rt.cpu_affinity[0], m_rt.cpu_affinity[1]);
}

}  /* namespace stark_periph_manager_node */
