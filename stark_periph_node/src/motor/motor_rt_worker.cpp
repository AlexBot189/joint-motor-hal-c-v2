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
    , m_last_seq(0)
    , m_seq_stall_us(0)
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

    /* 对齐 SHM 残留 seq_begin, 防止假 FAULT */
    if (m_shm) {
        m_last_seq = __atomic_load_n(&m_shm->mailbox.seq_begin, __ATOMIC_ACQUIRE);
    }
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

        /* 快照: 保存 ProcessMailbox 之前的 seq, 供 SafetyCheck 比较 */
        uint64_t seq_before = m_last_seq;

        /* ① 管理命令: 读 SHM mgmt 通道 (独立于 mailbox, 不丢命令) */
        ProcessMgmt();

        /* ② 控制下发: 读 SHM mailbox ,  multi_ctrl ,  PDO */
        ProcessMailbox();

        /* ③ 反馈上报: 读 fb_cache ,  SHM double buffer (200Hz) */
        PublishFeedback();

        /* ④ 安全监控 */
        SafetyCheck(seq_before);

        /* ⑤ 提交延迟样本 */
        m_tracer.commit_sample();

        m_cycle_count++;

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
 * 支持双电机同时控制 (一次 multi_ctrl 发完 ID 1,2)
 *
 * Mailbox 并发协议:
 *   算法写: seq_begin++ ,  write cmd[0], cmd[1] ,  seq_end = seq_begin
 *   RT 读:  read seq_begin (acquire)
 *          if begin != end: 正在写, 跳过
 *          if begin == m_last_seq: 无新命令, 返回
 *          else: 读全部 cmd ,  multi_ctrl ,  m_last_seq = begin
 */

void StarkRtWorker::ProcessMailbox()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm || !m_hal) return;

    uint64_t begin = __atomic_load_n(&m_shm->mailbox.seq_begin, __ATOMIC_ACQUIRE);
    uint64_t end   = __atomic_load_n(&m_shm->mailbox.seq_end,   __ATOMIC_ACQUIRE);

    if (begin != end)  return;   /* 算法正在写 */
    if (begin == m_last_seq) return;   /* 无新命令 */

    /* 首次收到算法命令 ,  设标志, 让主循环调 state_transition */
    if (!m_handshake_done.load(std::memory_order_acquire) && begin > 0) {
        m_handshake_done.store(true, std::memory_order_release);
        /* 握手完成, 主循环在 READY 状态检测后推进到 RUNNING */
    }

    /* 处理双电机命令 */
    motor_command_t cmd0 = m_shm->mailbox.cmd[0];
    motor_command_t cmd1 = m_shm->mailbox.cmd[1];
    m_last_seq = begin;

    /* 先处理 Byte0 控制命令 (改 pdo_byte0, 不发 target) */
    for (int i = 0; i < m_motor_count; i++) {
        motor_command_t c = (i == 0) ? cmd0 : cmd1;
        uint8_t mid = c.motor_id;
        if (mid < 1) continue;

        switch (c.cmd) {
        case STARK_CMD_ENABLE:
            motor_hal_pdo_enable(m_hal, mid); break;
        case STARK_CMD_DISABLE:
            motor_hal_pdo_disable(m_hal, mid);
            {
                multi_axis_cmd_t mcmd = {};
                mcmd.node_id       = mid;
                mcmd.enable        = false;
                mcmd.release_brake = true;
                mcmd.mode          = MOTOR_MODE_CURRENT;
                mcmd.target1       = 0;
                motor_hal_multi_ctrl(m_hal, &mcmd, 1);
            }
            break;
        case STARK_CMD_ESTOP:
            motor_hal_pdo_estop(m_hal, mid);
            {
                multi_axis_cmd_t mcmd = {};
                mcmd.node_id       = mid;
                mcmd.enable        = false;
                mcmd.release_brake = false;
                mcmd.mode          = MOTOR_MODE_CURRENT;
                mcmd.target1       = 0;
                motor_hal_multi_ctrl(m_hal, &mcmd, 1);
            }
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

    /* 再处理控制命令 (通过已设好的 pdo_byte0 发控制帧) */

    /* 多轴广播: 任意一个 cmd 是 MULTI 就走广播路径, 单电机也支持 */
    if (cmd0.cmd == STARK_CMD_MULTI || cmd1.cmd == STARK_CMD_MULTI) {
        multi_axis_cmd_t mcmds[STARK_MAX_MOTORS] = {};
        int mcount = 0;
        uint8_t b0;

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
            motor_hal_set_torque(m_hal, mid, (int16_t)c.value);
            break;
        case STARK_CMD_SPEED:
            motor_hal_set_velocity(m_hal, mid, (float)c.value / 100.0f);
            break;
        case STARK_CMD_CSV:
            motor_hal_set_velocity(m_hal, mid, (float)c.value / 100.0f);
            break;
        case STARK_CMD_PV:
            motor_hal_ctrl_raw(m_hal, mid, MOTOR_MODE_PROFILE_VEL,
                               (int16_t)(c.value / 100),
                               (uint16_t)(c.value2 / 100), 0);
            break;
        case STARK_CMD_POS:
            motor_hal_set_position(m_hal, mid, (float)c.value / 100.0f);
            break;
        case STARK_CMD_PP:
            motor_hal_ctrl_raw(m_hal, mid, MOTOR_MODE_PROFILE_POS,
                               (int16_t)c.value, (uint16_t)c.value2,
                               (int16_t)c.feedforward);
            break;
        case STARK_CMD_MIT:
            /* MIT 值转 float (uint16, float, uint16, 经 motor_hal_mit_control 编码) */
            motor_hal_mit_control(m_hal, mid,
                (float)c.mit_pos * (360.0f / 65535.0f) - 180.0f,
                (float)((int16_t)(c.mit_vel << 4)) / 16.0f,
                (float)c.mit_kp / 100.0f,
                (float)c.mit_kd / 100.0f,
                (float)((int16_t)(c.mit_torque << 4)) / 16.0f);
            break;
        default: break;  /* Byte0 commands are no-op here */
        }
    }
    } /* end else (non-MULTI path) */

    /* T7: 读 mailbox 完成 */
    m_tracer.mark_mailbox_read();

    /* T8: PDO 已在上面每条命令中发出 */
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
 * 安全动作必须走 PDO, 不在 RT 线程中调 SDO.
 *
 * 检查项:
 *   1. seq_begin 停滞 > 200ms ,  PDO torque=0, WARN
 *   2. seq_begin 停滞 > 500ms ,  PDO Shutdown, FAULT
 *   3. (TODO) 编码器停滞
 *   4. (TODO) 过温检测
 *   5. (TODO) CAN 断线
 */

void StarkRtWorker::SafetyCheck(uint64_t seq_before_process)
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    /* seq_begin 停滞检测 */
    if (seq_before_process > 0) {
        uint64_t begin = __atomic_load_n(&m_shm->mailbox.seq_begin, __ATOMIC_ACQUIRE);

        if (begin == seq_before_process) {
            if (m_seq_stall_us == 0) {
                m_seq_stall_us = now_us;
            }

            uint64_t stall_ms = (now_us - m_seq_stall_us) / 1000ULL;

            if (stall_ms > m_safety.algo_shutdown_ms) {
                /* FAULT: 算法严重失联 ,  PDO 降险 (只触发一次) */
                if (!m_fault_triggered) {
                    m_fault_triggered   = true;
                    m_shm->motor_severity = MOTOR_FAULT;
                    m_shm->fault_reason   = FAULT_ALGO_TIMEOUT;
                    __atomic_store_n(&m_pending_state, (uint32_t)STATE_FAULT, __ATOMIC_RELEASE);
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
            /* seq 恢复 ,  自动恢复 */
            m_seq_stall_us = 0;
            if (m_shm->motor_severity == MOTOR_FAULT &&
                m_shm->fault_reason == FAULT_ALGO_TIMEOUT)
            {
                /* 算法重连 ,  清除 FAULT, 回到 READY (calib_done 保持) */
                m_handshake_done.store(false, std::memory_order_release);
                m_fault_triggered = false;
                m_shm->motor_severity = MOTOR_OK;
                m_shm->fault_reason   = FAULT_NONE;
                __atomic_store_n(&m_pending_state, (uint32_t)STATE_READY, __ATOMIC_RELEASE);
                RT_LOG("SAFETY RECOVER: algo reconnected ,  READY");
            }
            else if (m_shm->motor_severity == MOTOR_WARN &&
                     m_shm->fault_reason == FAULT_ALGO_TIMEOUT)
            {
                /* WARN 级自动恢复 */
                m_fault_triggered = false;
                m_shm->motor_severity = MOTOR_OK;
                m_shm->fault_reason   = FAULT_NONE;
            }
        }
    }
}

/* _safe_torque_zero_all — PDO 路径 torque=0 (RT 安全) */

void StarkRtWorker::_safe_torque_zero_all()
{
    if (!m_hal) return;

    multi_axis_cmd_t cmds[STARK_MAX_MOTORS] = {};
    for (int i = 0; i < m_motor_count; i++) {
        cmds[i].node_id       = (uint8_t)(i + 1);
        cmds[i].mode          = MOTOR_MODE_CURRENT;
        cmds[i].enable        = true;
        cmds[i].release_brake = true;
        cmds[i].target1       = 0;  /* torque=0 */
    }
    motor_hal_multi_ctrl(m_hal, cmds, (uint8_t)m_motor_count);
}

/* _safe_disable_all — PDO 路径 enable=false + torque=0 */

void StarkRtWorker::_safe_disable_all()
{
    if (!m_hal) return;

    /* PDO 降险: enable=false + torque=0
     * 电机记住此状态直到收到新 PDO.
     * 算法失联时 ProcessMailbox 因 seq 不变直接 return,
     * 不会发新 PDO。算法恢复后通过 STARK_CMD_RECOVER/ENABLE 恢复. */
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
