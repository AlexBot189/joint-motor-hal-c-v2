/*
 * stark_rt_worker.cpp — RT 工作线程实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 关键约束:
 *   Run() 循环内禁止阻塞调用 (SDO/sleep/malloc)
 *   仅用 motor_hal non-blocking API (get_feedback / multi_ctrl / multi_pdo)
 *   安全动作 (torque=0 / disable) 走 PDO 路径
 */
#include "motor/motor_rt_worker.h"
#include "utils/rt_log.h"
#include "utils/latency_trace.h"
#include "motor/motor_ctrl.h"
#include "imu/imu_sensor.h"

#include <cstring>
#include <ctime>
#include <cassert>
#include <sched.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace stark_periph_manager_node {

/* 构造 / 析构 */

StarkRtWorker::StarkRtWorker(motor_hal_t* hal, stark_shm_t* shm,
                         StarkMotorCtrl* ctrl, ImuHALSensor* imu_sensor,
                         int motor_count)
    : m_hal(hal)
    , m_shm(shm)
    , m_ctrl(ctrl)
    , m_imu_sensor(imu_sensor)
    , m_motor_count(motor_count)
    , m_can_last_frame_us(0)
    , m_latency_idx(0)
    , m_report_divider(m_rt.report_divider)
    , m_report_enabled(false)
    , m_report_period_ms(5)
    , m_periodic_last_cycle(0)
    , m_cycle_count(0)
    , m_overrun_count(0)
{
    memset(m_last_position, 0, sizeof(m_last_position));
    memset(m_pos_stall_us, 0, sizeof(m_pos_stall_us));
    memset(m_sensor_notified, 0, sizeof(m_sensor_notified));
    memset(m_latency_history, 0, sizeof(m_latency_history));
    m_imu_notified = false;
}

StarkRtWorker::~StarkRtWorker()
{
    if (m_running.load(std::memory_order_acquire)) {
        Stop();
    }
}

/* Start() / Stop() */

void StarkRtWorker::Start()
{
    if (m_running.load(std::memory_order_acquire)) return;
    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&StarkRtWorker::Run, this);
}

void StarkRtWorker::Stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void StarkRtWorker::SetRtConfig(const RtConfig& cfg)
{
    m_rt = cfg;
    m_report_divider = cfg.report_divider;  /* 立即生效 */
}

void StarkRtWorker::SetReportEnabled(bool enabled, uint32_t period_ms)
{
    m_report_enabled = enabled;
    if (period_ms >= 1 && period_ms <= 1000) {
        m_report_period_ms = period_ms;
    }
    /* 首次触发: 从当前 RT 周期开始计时 */
    m_periodic_last_cycle = m_cycle_count;
    if (m_shm) {
        m_shm->periodic_enabled   = enabled ? 1 : 0;
        m_shm->periodic_period_ms = m_report_period_ms;
    }
}

/* Run() — RT 线程主循环 1KHz */

void StarkRtWorker::Run()
{
    SetThreadRt();

    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    const long period_ns = (long)m_rt.period_us * 1000L;

    while (m_running.load(std::memory_order_acquire)) {

        ProcessMgmt();
        ProcessMailbox();
        PublishFeedback();
        SafetyCheck();

        /* ⑤ 提交延迟样本 */
        m_tracer.commit_sample();

        m_cycle_count++;

        if (m_shm) m_shm->rt_cycle = (uint32_t)(m_cycle_count & 0xFFFFFFFF);

        /* 精确周期休眠 */
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

/*
 * ProcessMgmt() — 读 SHM mgmt 通道, 处理管理命令 (独立于 mailbox)
 *
 * 每电机独立 slot: mgmt_cmd[id-1] / mgmt_seq[id-1] / mgmt_ack[id-1].
 * 写入方递增 mgmt_seq[i], RT 处理后将 seq 拷贝到 mgmt_ack[i].
 * 多电机命令互不覆盖, 不依赖 mailbox seq.
 */

void StarkRtWorker::ProcessMgmt()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm || !m_hal) return;

    for (int i = 0; i < m_motor_count; i++) {
        uint8_t seq = __atomic_load_n(&m_shm->mgmt_seq[i], __ATOMIC_ACQUIRE);
        uint8_t ack = __atomic_load_n(&m_shm->mgmt_ack[i], __ATOMIC_RELAXED);

        if (seq == ack) continue;

        uint8_t cmd = m_shm->mgmt_cmd[i];
        uint8_t id  = (uint8_t)(i + 1);

        if (cmd == 0) {
            __atomic_store_n(&m_shm->mgmt_ack[i], seq, __ATOMIC_RELEASE);
            continue;
        }

        switch (cmd) {
        case STARK_CMD_ENABLE:
            motor_hal_pdo_enable(m_hal, id);
            break;
        case STARK_CMD_DISABLE:
            motor_hal_pdo_disable(m_hal, id);
            {
                multi_axis_cmd_t mcmd = {};
                mcmd.node_id       = id;
                mcmd.enable        = false;
                mcmd.release_brake = true;
                mcmd.mode          = MOTOR_MODE_CURRENT;
                mcmd.target1       = 0;
                motor_hal_multi_ctrl(m_hal, &mcmd, 1);
            }
            break;
        case STARK_CMD_ESTOP:
            motor_hal_pdo_estop(m_hal, id);
            {
                multi_axis_cmd_t mcmd = {};
                mcmd.node_id       = id;
                mcmd.enable        = false;
                mcmd.release_brake = false;
                mcmd.mode          = MOTOR_MODE_CURRENT;
                mcmd.target1       = 0;
                motor_hal_multi_ctrl(m_hal, &mcmd, 1);
            }
            break;
        case STARK_CMD_RECOVER:
            motor_hal_pdo_recover(m_hal, id);
            break;
        case STARK_CMD_CLEAR_FAULT:
            motor_hal_pdo_clear_fault(m_hal, id);
            motor_hal_pdo_enable(m_hal, id);
            motor_hal_ctrl_raw(m_hal, id, MOTOR_MODE_CURRENT, 0, 0, 0);
            break;
        default:
            break;
        }

        __atomic_store_n(&m_shm->mgmt_ack[i], seq, __ATOMIC_RELEASE);
    }
}

/*
 * ProcessMailbox() — 读 SHM mailbox ,  PDO multi_ctrl 广播
 *
 * 支持双电机同时控制 (一次 multi_ctrl 发完 ID 1,2).
 * SPSC 环形缓冲: 算法写 slot[seq_write % DEPTH], RT 消费 seq_read 到 seq_write.
 */

void StarkRtWorker::ProcessMailbox()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm || !m_hal) return;

    uint64_t r = __atomic_load_n(&m_shm->mailbox.seq_read,  __ATOMIC_ACQUIRE);
    uint64_t w = __atomic_load_n(&m_shm->mailbox.seq_write, __ATOMIC_ACQUIRE);

    if (r >= w) return;  /* 无新数据 */

    /* 首次收到算法命令: 握手 */
    if (!m_handshake_done.load(std::memory_order_acquire) && w > 0) {
        m_handshake_done.store(true, std::memory_order_release);
    }

    /* 消费所有待处理帧 */
    uint64_t count = w - r;
    if (count > STARK_MBOX_DEPTH) count = STARK_MBOX_DEPTH;

    for (uint64_t n = 0; n < count; n++) {
        uint64_t idx = (r + n) % STARK_MBOX_DEPTH;
        motor_command_t cmd0 = m_shm->mailbox.frames[idx].cmd[0];
        motor_command_t cmd1 = m_shm->mailbox.frames[idx].cmd[1];

        /* Byte0 管理命令 (改 pdo_byte0) */
        for (int i = 0; i < m_motor_count; i++) {
            motor_command_t c = (i == 0) ? cmd0 : cmd1;
            uint8_t mid = c.motor_id;
            if (mid < 1) continue;

            switch (c.cmd) {
            case STARK_CMD_ENABLE:
                motor_hal_pdo_enable(m_hal, mid); break;
            case STARK_CMD_DISABLE:
                motor_hal_pdo_disable(m_hal, mid);
                { multi_axis_cmd_t mcmd = {}; mcmd.node_id = mid;
                  mcmd.enable = false; mcmd.release_brake = true;
                  mcmd.mode = MOTOR_MODE_CURRENT; mcmd.target1 = 0;
                  motor_hal_multi_ctrl(m_hal, &mcmd, 1); }
                break;
            case STARK_CMD_ESTOP:
                motor_hal_pdo_estop(m_hal, mid);
                { multi_axis_cmd_t mcmd = {}; mcmd.node_id = mid;
                  mcmd.enable = false; mcmd.release_brake = false;
                  mcmd.mode = MOTOR_MODE_CURRENT; mcmd.target1 = 0;
                  motor_hal_multi_ctrl(m_hal, &mcmd, 1); }
                break;
            case STARK_CMD_RECOVER:
                motor_hal_pdo_recover(m_hal, mid); break;
            case STARK_CMD_SET_MODE:
                motor_hal_pdo_set_mode(m_hal, mid, (motor_mode_t)c.value); break;
            case STARK_CMD_CLEAR_FAULT:
                motor_hal_pdo_clear_fault(m_hal, mid);
                motor_hal_pdo_enable(m_hal, mid);
                motor_hal_ctrl_raw(m_hal, mid, MOTOR_MODE_CURRENT, 0, 0, 0);
                break;
            default: break;
            }
        }

        /* 控制命令 (通过 pdo_byte0 发 PDO) */
        if (cmd0.cmd == STARK_CMD_MULTI || cmd1.cmd == STARK_CMD_MULTI) {
            multi_axis_cmd_t mcmds[STARK_MAX_MOTORS] = {};
            int mcount = 0; uint8_t b0;

            if (motor_hal_pdo_consume_byte0(m_hal, cmd0.motor_id, &b0) == 0) {
                mcmds[mcount].node_id       = cmd0.motor_id;
                mcmds[mcount].mode          = (motor_mode_t)(pdo_byte0_get_mode(b0));
                mcmds[mcount].enable        = pdo_byte0_get_enable(b0);
                mcmds[mcount].release_brake = pdo_byte0_get_bus_on(b0);
                mcmds[mcount].target1       = (int16_t)cmd0.value;
                mcmds[mcount].target2       = (uint16_t)cmd0.value2;
                mcmds[mcount].feedforward   = (int16_t)cmd0.feedforward;
                mcount++;
            }
            if (motor_hal_pdo_consume_byte0(m_hal, cmd1.motor_id, &b0) == 0) {
                mcmds[mcount].node_id       = cmd1.motor_id;
                mcmds[mcount].mode          = (motor_mode_t)(pdo_byte0_get_mode(b0));
                mcmds[mcount].enable        = pdo_byte0_get_enable(b0);
                mcmds[mcount].release_brake = pdo_byte0_get_bus_on(b0);
                mcmds[mcount].target1       = (int16_t)cmd1.value;
                mcmds[mcount].target2       = (uint16_t)cmd1.value2;
                mcmds[mcount].feedforward   = (int16_t)cmd1.feedforward;
                mcount++;
            }
            if (mcount > 0) motor_hal_multi_ctrl(m_hal, mcmds, (uint8_t)mcount);
        } else {
            for (int i = 0; i < m_motor_count; i++) {
                motor_command_t c = (i == 0) ? cmd0 : cmd1;
                uint8_t mid = c.motor_id;
                if (mid < 1 || mid > (uint8_t)m_motor_count) continue;

                switch (c.cmd) {
                case STARK_CMD_TORQUE:
                    motor_hal_set_torque(m_hal, mid, (int16_t)c.value); break;
                case STARK_CMD_SPEED:
                    motor_hal_set_velocity(m_hal, mid, (float)c.value / 100.0f); break;
                case STARK_CMD_CSV:
                    motor_hal_set_velocity(m_hal, mid, (float)c.value / 100.0f); break;
                case STARK_CMD_PV:
                    motor_hal_ctrl_raw(m_hal, mid, MOTOR_MODE_PROFILE_VEL,
                                       (int16_t)(c.value / 100),
                                       (uint16_t)(c.value2 / 100), 0); break;
                case STARK_CMD_POS:
                    motor_hal_set_position(m_hal, mid, (float)c.value / 100.0f); break;
                case STARK_CMD_PP:
                    motor_hal_ctrl_raw(m_hal, mid, MOTOR_MODE_PROFILE_POS,
                                       (int16_t)c.value, (uint16_t)c.value2,
                                       (int16_t)c.feedforward); break;
                case STARK_CMD_MIT:
                    motor_hal_mit_control(m_hal, mid,
                        (float)c.mit_pos * (360.0f / 65535.0f) - 180.0f,
                        (float)((int16_t)(c.mit_vel << 4)) / 16.0f,
                        (float)c.mit_kp / 100.0f,
                        (float)c.mit_kd / 100.0f,
                        (float)((int16_t)(c.mit_torque << 4)) / 16.0f); break;
                default: break;
                }
            }
        }
    }

    /* 确认消费 */
    __atomic_store_n(&m_shm->mailbox.seq_read, r + count, __ATOMIC_RELEASE);

    m_tracer.mark_mailbox_read();
    m_tracer.mark_pdo_sent();
}

/*
 * PublishFeedback() — 读 fb_cache + 组装 feedback_frame_t ,  SHM
 *
 * 频率: 每 m_report_divider (5) 个周期 = 200Hz
 */

void StarkRtWorker::PublishFeedback()
{
    /* 周期上报: 基于 RT 周期计数器, 不嵌套在 feedback_frame_t 分频内 */
    if (m_report_enabled && m_shm && m_hal) {
        uint64_t elapsed = m_cycle_count - m_periodic_last_cycle;
        if (elapsed >= m_report_period_ms) {
            m_periodic_last_cycle = m_cycle_count;

            PeriodicUploadData d;
            memset(&d, 0, sizeof(d));

            /* IMU */
            imu_data_t imu_buf;
            bool imu_ok = false;
            if (m_imu_sensor && m_imu_sensor->IsReady()) {
                m_imu_sensor->Read(&imu_buf);
                imu_ok = true;
            }
            if (imu_ok) {
                d.gyro_dps_x  = imu_buf.gyro_x;
                d.gyro_dps_y  = imu_buf.gyro_y;
                d.gyro_dps_z  = imu_buf.gyro_z;
                d.quat_w      = imu_buf.quat_w;
                d.quat_x      = imu_buf.quat_x;
                d.quat_y      = imu_buf.quat_y;
                d.quat_z      = imu_buf.quat_z;
                d.gyro_roll   = imu_buf.roll;
                d.gyro_pitch  = imu_buf.pitch;
                d.gyro_yaw    = imu_buf.yaw;
                d.acc_x       = imu_buf.acc_x;
                d.acc_y       = imu_buf.acc_y;
                d.acc_z       = imu_buf.acc_z;
            }

            /* 双电机 */
            uint32_t motor_ts_min = 0xFFFFFFFF;
            uint32_t sensor_ts_min = 0xFFFFFFFF;

            for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
                motor_feedback_t mfb;
                motor_sensor_t s;
                bool is_right = (id == 1);

                if (motor_hal_get_feedback(m_hal, id, &mfb) == 0) {
                    uint32_t ts = (uint32_t)(mfb.timestamp_us & 0xFFFFFFFF);
                    if (ts < motor_ts_min) motor_ts_min = ts;
                    int32_t vel_x10  = (int32_t)mfb.velocity * 10;
                    int16_t iq_x100  = (int16_t)(mfb.current_iq / 10);
                    int16_t fcode    = (int16_t)mfb.error_code;
                    int16_t mstate   = (int16_t)mfb.status_byte;

                    /* SDO telemetry: 0x300 frame has only Iq valid on RV1126B */
                    int32_t sdo_val = 0;
                    int32_t tmp_x100;
                    if (motor_hal_get_sdo_temperature(m_hal, id, &sdo_val) == 0)
                        tmp_x100 = sdo_val * 10;  /* 0.1°C -> °C×100 */
                    else
                        tmp_x100 = (int32_t)mfb.temperature * 10;

                    int16_t ang_x10;
                    if (motor_hal_get_sdo_position(m_hal, id, &sdo_val) == 0)
                        ang_x10 = (int16_t)(sdo_val * 3600 / 65536);
                    else
                        ang_x10 = (int16_t)((int32_t)mfb.position * 3600 / 65536);

                    if (is_right) {
                        d.RealtimeVelocity = vel_x10;
                        d.motor_abs_angle  = ang_x10;
                        d.cal_Iq_current   = iq_x100;
                        d.motor_temp       = tmp_x100;
                        d.fault_code       = fcode;
                        d.motor_state      = mstate;
                    } else {
                        d.RealtimeVelocity_left = vel_x10;
                        d.motor_abs_angle_left  = ang_x10;
                        d.cal_Iq_current_left   = iq_x100;
                        d.motor_temp_left       = tmp_x100;
                        d.fault_code_left       = fcode;
                        d.motor_state_left      = mstate;
                    }
                }

                if (motor_hal_get_sensor(m_hal, id, &s) == 0) {
                    uint32_t sts = (uint32_t)(s.timestamp_us & 0xFFFFFFFF);
                    if (sts < sensor_ts_min) sensor_ts_min = sts;
                    if (is_right) {
                        d.hall_a_data  = s.hall_adc0;
                        d.hall_b_data  = s.hall_adc1;
                        d.hall_c_data  = s.hall_adc2;
                        d.df181_torque = s.force_raw;
                        d.knee_angle   = (int16_t)s.knee_adc;
                        d.key_landing  = s.hw_sw_pc9;
                        d.torque_valid = s.data_valid;
                    } else {
                        d.hall_a_data_left  = s.hall_adc0;
                        d.hall_b_data_left  = s.hall_adc1;
                        d.hall_c_data_left  = s.hall_adc2;
                        d.df181_torque_left = s.force_raw;
                        d.knee_angle_left   = (int16_t)s.knee_adc;
                        d.key_landing_left  = s.hw_sw_pc9;
                        d.torque_valid_left = s.data_valid;
                    }
                }
            }

            struct timespec now_ts;
            clock_gettime(CLOCK_REALTIME, &now_ts);
            d.timestamp_ms  = (uint32_t)(now_ts.tv_sec * 1000ULL +
                                          now_ts.tv_nsec / 1000000ULL);
            d.frame_cycle   = (uint32_t)m_cycle_count;
            d.motor_ts_us   = (motor_ts_min != 0xFFFFFFFF) ? motor_ts_min : 0;
            d.imu_ts_us     = (uint32_t)(imu_buf.timestamp_us & 0xFFFFFFFF);
            d.sensor_ts_us  = (sensor_ts_min != 0xFFFFFFFF) ? sensor_ts_min : 0;

            memcpy(&m_shm->periodic_data, &d, sizeof(d));
            __atomic_add_fetch(&m_shm->periodic_version, 1, __ATOMIC_RELEASE);
        }
    }

    if (--m_report_divider > 0) return;
    m_report_divider = m_rt.report_divider;

    if (!m_hal || !m_shm) return;
    if (!m_active.load(std::memory_order_acquire)) return;

    /* 读 IMU 一次, feedback_frame_t 和 PeriodicUploadData 共用 */
    imu_data_t imu_local;
    bool imu_valid = false;
    if (m_imu_sensor && m_imu_sensor->IsReady()) {
        m_imu_sensor->Read(&imu_local);
        imu_valid = true;
    }

    uint32_t active    = __atomic_load_n(&m_shm->active_idx, __ATOMIC_ACQUIRE);
    uint32_t write_idx = active ^ 1;

    feedback_frame_t* fb = &m_shm->fb_buffer[write_idx];
    memset(fb, 0, sizeof(feedback_frame_t));

    struct timespec ts;

    /* T1: 开始读 fb_cache */
    m_tracer.mark_fb_read_start();

    /* 填充电机反馈 (从 HAL 反馈缓存) */
    for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
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

    /* T2: fb_cache 读取完成 */
    m_tracer.mark_fb_read_done();

    /* 填充传感器透传 */
    for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
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

    /* 填充 IMU 融合数据 (非阻塞, 硬件未就绪时全零) */
    if (imu_valid) {
        fb->imu = imu_local;
    } else {
        memset(&fb->imu, 0, sizeof(fb->imu));
    }

    /* 气压计: 硬件未接入, 保持全零 */
    memset(&fb->baro, 0, sizeof(fb->baro));

    /* IMU+传感器+气压计 组装完成 */
    m_tracer.mark_mock_sensor_done();

    /* T4: SHM 切换 */
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fb->ts_shm_write      = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    fb->ts_frame_assembly = fb->ts_shm_write;
    fb->timestamp_us      = fb->ts_shm_write;

    /* 切换活跃 Buffer */
    __atomic_store_n(&m_shm->active_idx, write_idx, __ATOMIC_RELEASE);

    /* T4: SHM 双 Buffer 切换完成 */
    m_tracer.mark_shm_write_done();

    /* 填充 SHM 耗时统计 (供 perf_test 读取) */
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

/*
 * SafetyCheck() — 安全监控
 *
 * 检查项:
 *   1. 算法心跳超时 → PDO disable 双电机 (不发 FAULT)
 *   2. (TODO) 编码器停滞
 *   3. (TODO) 过温检测
 *   4. (TODO) CAN 断线
 */

void StarkRtWorker::SafetyCheck()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    /* 算法心跳检测 (替代 seq 超时) */
    uint32_t heartbeat = __atomic_load_n(&m_shm->algo_heartbeat, __ATOMIC_ACQUIRE);
    uint32_t timeout   = m_shm->algo_heartbeat_timeout_ms;
    if (timeout == 0) timeout = m_safety.heartbeat_timeout_ms;

    if (heartbeat != m_last_heartbeat) {
        m_last_heartbeat   = heartbeat;
        m_heartbeat_lost_us = 0;
    } else if (m_handshake_done.load(std::memory_order_acquire)) {
        if (m_heartbeat_lost_us == 0) {
            m_heartbeat_lost_us = now_us;
        }
        uint64_t lost_ms = (now_us - m_heartbeat_lost_us) / 1000ULL;
        if (lost_ms > timeout) {
            _safe_disable_all();
        }
    }
}

/* _safe_disable_all — PDO 路径 enable=false + torque=0 */

void StarkRtWorker::_safe_disable_all()
{
    if (!m_hal) return;

    multi_axis_cmd_t cmds[STARK_MAX_MOTORS] = {};
    for (int i = 0; i < m_motor_count; i++) {
        cmds[i].node_id       = (uint8_t)(i + 1);
        cmds[i].mode          = MOTOR_MODE_CURRENT;
        cmds[i].enable        = false;
        cmds[i].release_brake = false;
        cmds[i].target1       = 0;
    }
    motor_hal_multi_ctrl(m_hal, cmds, (uint8_t)m_motor_count);
}

/* SetThreadRt() — RT 线程属性 */

void StarkRtWorker::SetThreadRt()
{
    if (!m_rt.enable_rt) {
        prctl(PR_SET_NAME, "stark_nrt", 0, 0, 0);
        printf("[StarkRtWorker] Thread: SCHED_OTHER (non-RT mode, no affinity)\n");
        return;
    }

    prctl(PR_SET_NAME, "stark_rt", 0, 0, 0);

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

    printf("[StarkRtWorker] Thread: SCHED_FIFO prio=%d period=%dus cpu=%d,%d\n",
           m_rt.priority, m_rt.period_us,
           m_rt.cpu_affinity[0], m_rt.cpu_affinity[1]);
}

/* GetAvgLatencyUs() — 诊断: 最近 64 次闭环延迟平均值 */

uint32_t StarkRtWorker::GetAvgLatencyUs() const
{
    uint64_t sum = 0;
    uint32_t n   = (m_latency_idx < 64) ? m_latency_idx : 64;
    if (n == 0) return 0;

    for (uint32_t i = 0; i < n; i++) {
        sum += m_latency_history[i];
    }
    return (uint32_t)(sum / n);
}

/*
 * GetPendingState() — 主循环读取 RT 线程请求的状态切换
 * atomic exchange: 读当前值并清零为 STATE_BOOTING
 */

stark_state_t StarkRtWorker::GetPendingState()
{
    return (stark_state_t)__atomic_exchange_n(
        &m_pending_state, (uint32_t)STATE_BOOTING, __ATOMIC_ACQUIRE);
}

}  /* namespace stark_periph_manager_node */
