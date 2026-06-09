/*
 * exo_rt_worker.cpp — RT 工作线程实现
 *
 * ★ 关键约束:
 *   - Run() 循环内禁止任何阻塞调用 (SDO/sleep/malloc)
 *   - 仅使用 motor_hal non-blocking API (get_feedback / multi_ctrl / multi_pdo)
 *   - 安全动作 (torque=0 / disable) 必须走 PDO 路径, 不能走 SDO
 *   - 延迟追踪每周期记录 T0~T8, 支持端到端性能分析
 */
#include "exo_rt_worker.h"
#include "exo_rt_log.h"
#include "exo_latency_trace.h"
#include "exo_motor_ctrl.h"
#include "exo_mock_sensor.h"

#include <cstring>
#include <ctime>
#include <cassert>
#include <sched.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace stark_periph_manager_node {

/* ════════════════════════════════════════════════════════════════════
 * 构造 / 析构
 * ════════════════════════════════════════════════════════════════════ */

ExoRtWorker::ExoRtWorker(motor_hal_t* hal, exo_shm_t* shm,
                         ExoMotorCtrl* ctrl, ExoMockSensor* mock_sensor)
    : m_hal(hal)
    , m_shm(shm)
    , m_ctrl(ctrl)
    , m_mock_sensor(mock_sensor)
    , m_last_seq(0)
    , m_seq_stall_us(0)
    , m_can_last_frame_us(0)
    , m_latency_idx(0)
    , m_report_divider(m_rt.report_divider)
    , m_cycle_count(0)
    , m_overrun_count(0)
{
    memset(m_last_position, 0, sizeof(m_last_position));
    memset(m_pos_stall_us, 0, sizeof(m_pos_stall_us));
    memset(m_sensor_notified, 0, sizeof(m_sensor_notified));
    memset(m_latency_history, 0, sizeof(m_latency_history));
    m_imu_notified = false;

    /* Bug#1: 对齐 SHM 残留 seq_begin, 防止假 FAULT */
    if (m_shm) {
        m_last_seq = __atomic_load_n(&m_shm->mailbox.seq_begin, __ATOMIC_ACQUIRE);
    }
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
    if (m_running.load(std::memory_order_acquire)) return;
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

void ExoRtWorker::SetRtConfig(const RtConfig& cfg)
{
    m_rt = cfg;
    m_report_divider = cfg.report_divider;  /* 立即生效 */
}

/* ════════════════════════════════════════════════════════════════════
 * Run() — RT 线程主循环 1KHz
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::Run()
{
    SetThreadRt();

    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    const long period_ns = (long)m_rt.period_us * 1000L;

    while (m_running.load(std::memory_order_acquire)) {

        /* ── 更新 mock 传感器 ── */
        MockSensorTick();

        /* ① 控制下发: 读 SHM mailbox → multi_ctrl → PDO */
        ProcessMailbox();

        /* ② 反馈上报: 读 fb_cache → SHM double buffer (200Hz) */
        PublishFeedback();

        /* ③ 安全监控 */
        SafetyCheck();

        /* ④ 提交延迟样本 */
        m_tracer.commit_sample();

        m_cycle_count++;

        /* ── 精确周期休眠 ── */
        next_wake.tv_nsec += period_ns;
        while (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_nsec -= 1000000000L;
            next_wake.tv_sec++;
        }

        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                  &next_wake, nullptr);
        if (ret != 0) {
            m_overrun_count++;
            clock_gettime(CLOCK_MONOTONIC, &next_wake);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════
 * ProcessMailbox() — 读 SHM mailbox → PDO multi_ctrl 广播
 *
 * ★ v3: 支持双电机同时控制 (一次 multi_ctrl 发完 ID 1,2)
 *
 * Mailbox 并发协议:
 *   算法写: seq_begin++ → write cmd[0], cmd[1] → seq_end = seq_begin
 *   RT 读:  read seq_begin (acquire)
 *          if begin != end: 正在写, 跳过
 *          if begin == m_last_seq: 无新命令, 返回
 *          else: 读全部 cmd → multi_ctrl → m_last_seq = begin
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::ProcessMailbox()
{
    if (!m_shm || !m_hal) return;

    uint64_t begin = __atomic_load_n(&m_shm->mailbox.seq_begin, __ATOMIC_ACQUIRE);
    uint64_t end   = __atomic_load_n(&m_shm->mailbox.seq_end,   __ATOMIC_ACQUIRE);

    if (begin != end)  return;   /* 算法正在写 */
    if (begin == m_last_seq) return;   /* 无新命令 */

    /* ── 首次收到算法命令 → 设标志, 让主循环调 state_transition ── */
    if (m_last_seq == 0 && begin > 0) {
        m_pending_state = STATE_RUNNING;  /* RT 线程只设标志, 不写 shm->node_state */
    }

    /* ── 读取双电机命令 (值拷贝, snapshot 协议保证安全) ── */
    motor_command_t cmd0 = m_shm->mailbox.cmd[0];
    motor_command_t cmd1 = m_shm->mailbox.cmd[1];
    m_last_seq = begin;

    /* ── 构造 multi_axis_cmd_t (一次广播发完双电机) ── */
    multi_axis_cmd_t mcmds[EXO_MOTOR_COUNT] = {};
    uint8_t mcount = 0;

    for (int i = 0; i < EXO_MOTOR_COUNT; i++) {
        motor_command_t c = (i == 0) ? cmd0 : cmd1;
        uint8_t mid = c.motor_id;
        if (mid < 1 || mid > 2) continue;   /* 无效 motor_id */

        auto& mc = mcmds[mcount];
        mc.node_id       = mid;
        mc.enable        = true;
        mc.release_brake = true;
        mc.clear_error   = false;

        switch (c.cmd) {
        case EXO_CMD_TORQUE:
            mc.mode    = MOTOR_MODE_CURRENT;
            mc.target1 = (int16_t)c.value;   /* mA */
            break;
        case EXO_CMD_SPEED:
            mc.mode    = MOTOR_MODE_PROFILE_VEL;
            mc.target1 = (int16_t)(c.value / 100);  /* rpm×100 → rpm */
            break;
        case EXO_CMD_POS:
            mc.mode    = MOTOR_MODE_CSP;
            mc.target1 = motor_deg_to_counts((float)c.value / 100.0f);
            break;
        default:
            mc.mode    = MOTOR_MODE_CURRENT;
            mc.target1 = 0;
            break;
        }

        mcount++;
    }

    if (mcount > 0) {
        /* ── T5: 读 mailbox → 即将发 PDO ── */
        m_tracer.mark_mailbox_read();

        /* PDO 广播 — 一帧 64B CANFD 发完所有电机 */
        motor_hal_multi_ctrl(m_hal, mcmds, mcount);

        /* ── T6: PDO 发出 ── */
        m_tracer.mark_pdo_sent();
    }
}

/* ════════════════════════════════════════════════════════════════════
 * PublishFeedback() — 读 fb_cache + 组装 feedback_frame_t → SHM
 *
 * 频率: 每 m_report_divider (5) 个周期 = 200Hz
 *
 * 延迟追踪时间戳:
 *   T2: 读 fb_cache 完成
 *   T3: SHM 双 Buffer 切换 (atomic_store 前)
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::PublishFeedback()
{
    if (--m_report_divider > 0) return;
    m_report_divider = m_rt.report_divider;

    if (!m_hal || !m_shm) return;

    uint32_t active    = __atomic_load_n(&m_shm->active_idx, __ATOMIC_ACQUIRE);
    uint32_t write_idx = active ^ 1;

    feedback_frame_t* fb = &m_shm->fb_buffer[write_idx];
    memset(fb, 0, sizeof(feedback_frame_t));

    struct timespec ts;

    /* ── T1: 开始读 fb_cache ── */
    m_tracer.mark_fb_read_start();

    /* ── 填充电机反馈 (从 HAL 反馈缓存) ── */
    for (uint8_t id = 1; id <= EXO_MOTOR_COUNT; ++id) {
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

    /* ── T2: fb_cache 读取完成 ── */
    m_tracer.mark_fb_read_done();

    /* ── 填充传感器透传 ── */
    for (uint8_t id = 1; id <= EXO_MOTOR_COUNT; ++id) {
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
            RT_LOG("sensor not configured for motor %d (ret=%d)", id, ret);
            m_sensor_notified[id - 1] = true;
        }
    }

    /* ── 填充 mock IMU ── */
    if (m_mock_sensor) {
        fb->imu  = m_mock_sensor->GetImu();
        fb->baro = m_mock_sensor->GetBaro();
    }

    /* ── T3: mock传感器+IMU+气压计 组装完成 ── */
    m_tracer.mark_mock_sensor_done();

    /* ── T4: SHM 切换 ── */
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fb->ts_shm_write      = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    fb->ts_frame_assembly = fb->ts_shm_write;
    fb->timestamp_us      = fb->ts_shm_write;

    /* ── 切换活跃 Buffer ── */
    __atomic_store_n(&m_shm->active_idx, write_idx, __ATOMIC_RELEASE);

    /* ── T4: SHM 双 Buffer 切换完成 ── */
    m_tracer.mark_shm_write_done();

    /* ── 填充 SHM 耗时统计 (供 perf_test 读取) ── */
    {
        latency_stats_t st = {};
        m_tracer.fill_shm_stats(st);
        m_shm->fb_read_avg_us    = (uint16_t)st.fb_read_avg;
        m_shm->fb_read_max_us    = (uint16_t)st.fb_read_max;
        m_shm->fb_total_avg_us   = (uint16_t)st.fb_total_avg;
        m_shm->fb_total_max_us   = (uint16_t)st.fb_total_max;
        m_shm->ctrl_total_avg_us = (uint16_t)st.ctrl_total_avg;
        m_shm->ctrl_total_max_us = (uint16_t)st.ctrl_total_max;
        m_shm->trace_cycle_count = st.cycle_count;
        m_shm->shm_write_avg_us  = (uint16_t)st.shm_write_avg;
    }

    m_shm->cycle_overrun_count = (uint16_t)(m_overrun_count & 0xFFFF);
}

/* ════════════════════════════════════════════════════════════════════
 * SafetyCheck() — 安全监控
 *
 * ★ 安全动作必须走 PDO, 不在 RT 线程中调 SDO.
 *
 * 检查项:
 *   1. seq_begin 停滞 > 200ms → PDO torque=0, WARN
 *   2. seq_begin 停滞 > 500ms → PDO Shutdown, FAULT
 *   3. (TODO) 编码器停滞
 *   4. (TODO) 过温检测
 *   5. (TODO) CAN 断线
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::SafetyCheck()
{
    if (!m_shm) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    /* ── seq_begin 停滞检测 ── */
    if (m_last_seq > 0) {
        uint64_t begin = __atomic_load_n(&m_shm->mailbox.seq_begin, __ATOMIC_ACQUIRE);

        if (begin == m_last_seq) {
            if (m_seq_stall_us == 0) {
                m_seq_stall_us = now_us;
            }

            uint64_t stall_ms = (now_us - m_seq_stall_us) / 1000ULL;

            if (stall_ms > m_safety.algo_shutdown_ms) {
                /* FAULT: 算法严重失联 → PDO 降险 (只触发一次) */
                if (!m_fault_triggered) {
                    m_fault_triggered   = true;
                    m_shm->motor_severity = MOTOR_FAULT;
                    m_shm->fault_reason   = FAULT_ALGO_TIMEOUT;
                    m_pending_state = STATE_FAULT;
                    _safe_disable_all();
                    RT_LOG("SAFETY FAULT: algo timeout %llu ms, all disabled",
                           (unsigned long long)stall_ms);
                }
            }
            else if (stall_ms > m_safety.algo_timeout_ms) {
                /* WARN: torque=0 via PDO (RT safe) */
                if (m_shm->motor_severity < MOTOR_WARN) {
                    m_shm->motor_severity = MOTOR_WARN;
                    m_shm->fault_reason   = FAULT_ALGO_TIMEOUT;
                    _safe_torque_zero_all();
                    RT_LOG("SAFETY WARN: algo timeout %llu ms, torque=0",
                           (unsigned long long)stall_ms);
                }
            }
        }
        else {
            /* seq 恢复 */
            m_seq_stall_us = 0;
            m_fault_triggered = false;
            if (m_shm->motor_severity == MOTOR_WARN &&
                m_shm->fault_reason == FAULT_ALGO_TIMEOUT)
            {
                /* WARN 级自动恢复 */
                m_shm->motor_severity = MOTOR_OK;
                m_shm->fault_reason   = FAULT_NONE;
            }
        }
    }
}

/* ════════════════════════════════════════════════════════════════════
 * _safe_torque_zero_all — PDO 路径 torque=0 (RT 安全)
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::_safe_torque_zero_all()
{
    if (!m_hal) return;

    multi_axis_cmd_t cmds[EXO_MOTOR_COUNT] = {};
    for (int i = 0; i < EXO_MOTOR_COUNT; i++) {
        cmds[i].node_id       = (uint8_t)(i + 1);
        cmds[i].mode          = MOTOR_MODE_CURRENT;
        cmds[i].enable        = true;
        cmds[i].release_brake = true;
        cmds[i].target1       = 0;  /* torque=0 */
    }
    motor_hal_multi_ctrl(m_hal, cmds, EXO_MOTOR_COUNT);
}

void ExoRtWorker::_safe_disable_all()
{
    if (!m_hal) return;

    /* ★ 第一步: PDO 降险
     * enable=false + torque=0 → 立即切断功率输出
     * 不触发 DS402 状态机, 但电机会停止出力.
     * 第二步 SDO 0x6040=0x06 由主循环补发 (非 RT 路径). */
    multi_axis_cmd_t cmds[EXO_MOTOR_COUNT] = {};
    for (int i = 0; i < EXO_MOTOR_COUNT; i++) {
        cmds[i].node_id       = (uint8_t)(i + 1);
        cmds[i].mode          = MOTOR_MODE_CURRENT;
        cmds[i].enable        = false;  /* 停机 */
        cmds[i].release_brake = false;
        cmds[i].target1       = 0;
    }
    motor_hal_multi_ctrl(m_hal, cmds, EXO_MOTOR_COUNT);
}

/* ════════════════════════════════════════════════════════════════════
 * MockSensorTick() — 更新 mock IMU + 气压计
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::MockSensorTick()
{
    if (!m_mock_sensor) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    m_mock_sensor->Tick(now_us);
}

/* ════════════════════════════════════════════════════════════════════
 * SetThreadRt() — RT 线程属性
 * ════════════════════════════════════════════════════════════════════ */

void ExoRtWorker::SetThreadRt()
{
    if (!m_rt.enable_rt) {
        prctl(PR_SET_NAME, "exo_nrt", 0, 0, 0);
        printf("[ExoRtWorker] Thread: SCHED_OTHER (non-RT mode, no affinity)\n");
        return;
    }

    prctl(PR_SET_NAME, "exo_rt", 0, 0, 0);

    struct sched_param param;
    param.sched_priority = m_rt.priority;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        RT_LOG("SCHED_FIFO failed (need root/CAP_SYS_NICE)");
    }

    /* 锁定当前+未来全部内存页, 防止 RT 线程缺页中断 */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RT_LOG("mlockall failed (page faults possible)");
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(m_rt.cpu_affinity[0], &cpuset);
    if (m_rt.cpu_affinity[1] >= 0) {
        CPU_SET(m_rt.cpu_affinity[1], &cpuset);
    }
    ret = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (ret != 0) {
        RT_LOG("CPU affinity failed");
    }

    printf("[ExoRtWorker] Thread: SCHED_FIFO prio=%d period=%dus cpu=%d,%d\n",
           m_rt.priority, m_rt.period_us,
           m_rt.cpu_affinity[0], m_rt.cpu_affinity[1]);
}

/* ════════════════════════════════════════════════════════════════════
 * GetAvgLatencyUs() — 诊断: 最近 64 次闭环延迟平均值
 * ════════════════════════════════════════════════════════════════════ */

uint32_t ExoRtWorker::GetAvgLatencyUs() const
{
    uint64_t sum = 0;
    uint32_t n   = (m_latency_idx < 64) ? m_latency_idx : 64;
    if (n == 0) return 0;

    for (uint32_t i = 0; i < n; i++) {
        sum += m_latency_history[i];
    }
    return (uint32_t)(sum / n);
}

/* ════════════════════════════════════════════════════════════════════
 * GetPendingState() — 主循环读取 RT 线程请求的状态切换
 *
 * atomic exchange: 读当前值并清零为 STATE_INIT
 * STATE_INIT 表示无待处理请求.
 * ════════════════════════════════════════════════════════════════════ */

exo_state_t ExoRtWorker::GetPendingState()
{
    return (exo_state_t)__atomic_exchange_n(
        &m_pending_state, (uint32_t)STATE_INIT, __ATOMIC_ACQUIRE);
}

}  /* namespace stark_periph_manager_node */
