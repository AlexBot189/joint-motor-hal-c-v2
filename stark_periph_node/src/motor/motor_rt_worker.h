/*
 * stark_rt_worker.h — RT 工作线程
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 三合一: 控制下发 + 反馈上报 + 安全监控
 * 周期 1KHz, SCHED_FIFO prio=90
 *
 * 单周期流程:
 *   ① ProcessMailbox()   ,  读 SHM mailbox ,  PDO multi_ctrl
 *   ② PublishFeedback()  ,  fb_cache + IMU ,  SHM double buffer
 *   ③ SafetyCheck()      , 心跳超时脱使能
 */
#pragma once

#include <atomic>
#include <thread>
#include <cstdint>

#include "utils/latency_trace.h"

extern "C" {
#include "motor_hal.h"
#include "stark_shm.h"
}

namespace stark_periph_manager_node {

class StarkMotorCtrl;
class ImuHALSensor;

struct SafetyConfig {
    uint32_t heartbeat_timeout_ms = 1000;
    int32_t  overtemp_celsius  = 80;
    uint32_t can_offline_ms    = 2000;
    uint32_t encoder_stall_s   = 3;
};

struct RtConfig {
    int      priority         = 90;
    uint32_t period_us        = 1000;
    int      report_divider   = 5;     /* 5周期 ,  200Hz */
    int      cpu_affinity[2]  = {3, -1}; /* 只绑 core 3, core 2 留给算法进程 */
    bool     enable_rt        = true;  /* true=SCHED_FIFO, false=SCHED_OTHER */
};

class StarkRtWorker {
public:
    StarkRtWorker(motor_hal_t* hal, stark_shm_t* shm,
                StarkMotorCtrl* ctrl, ImuHALSensor* imu_sensor,
                int motor_count);
    ~StarkRtWorker();

    void Start();
    void Stop();
    bool IsRunning() const { return m_running.load(std::memory_order_acquire); }

    void SetSafetyConfig(const SafetyConfig& cfg) { m_safety = cfg; }
    void SetRtConfig(const RtConfig& cfg);

    /* 诊断 */
    uint64_t GetCycleCount()  const { return m_cycle_count; }
    uint64_t GetOverrunCount() const { return m_overrun_count; }
    uint32_t GetAvgLatencyUs() const;

    /** @brief 获取 RT 线程请求的状态切换 (主循环读取后清零, atomic exchange) */
    stark_state_t GetPendingState();

    /** @brief 算法握手是否已完成 (用于 ENABLED, RUNNING 自动推进) */
    bool IsHandshakeDone() const { return m_handshake_done.load(std::memory_order_acquire); }

    /** @brief 获取延迟追踪器 (外部填充 SHM stats) */
    StarkLatencyTracer* GetTracer() { return &m_tracer; }

    /** @brief 激活/去激活 RT 工作线程 (校准完成后由 main.cpp 设 true) */
    void SetActive(bool active) { m_active.store(active, std::memory_order_release); }
    bool IsActive() const { return m_active.load(std::memory_order_acquire); }

    /** @brief 周期上报开关 + 周期配置 */
    void SetReportEnabled(bool enabled, uint32_t period_ms);
    void Run();
    void ProcessMgmt();
    void ProcessMailbox();
    void PublishFeedback();
    void SafetyCheck();

    void SetThreadRt();

    /* 双电机 torque=0 (PDO, RT安全) */
    void _safe_disable_all();

    motor_hal_t*    m_hal;
    stark_shm_t*      m_shm;
    StarkMotorCtrl*   m_ctrl;
    ImuHALSensor*   m_imu_sensor;

    std::atomic<bool> m_running{false};
    std::thread       m_thread;

    SafetyConfig m_safety;
    RtConfig     m_rt;

    /* 延迟追踪 */
    uint64_t m_can_last_frame_us;
    uint64_t m_latency_history[64];     /* 最近64次闭环延迟 (T8-T0) */
    uint32_t m_latency_idx;

    /* 心跳检测 */
    uint32_t m_last_heartbeat;
    uint64_t m_heartbeat_lost_us;

    /* 周期控制 */
    int      m_report_divider;
    bool     m_report_enabled = false;     /* 周期上报总开关 */
    uint32_t m_report_period_ms = 5;       /* 周期上报间隔 ms */
    uint64_t m_periodic_last_cycle = 0;    /* 上次上报的 RT 周期号 */
    uint64_t m_cycle_count;
    uint64_t m_overrun_count;

    /* 电机数量 (从 config.json 读取, ≤ STARK_MAX_MOTORS) */
    int      m_motor_count;

    /* 停滞检测 */
    int16_t  m_last_position[STARK_MAX_MOTORS];
    uint64_t m_pos_stall_us[STARK_MAX_MOTORS];

    /* 一次性日志 */
    bool     m_sensor_notified[STARK_MAX_MOTORS];
    bool     m_imu_notified;

    /* RT, 主线程 状态切换请求 (atomic, RT 写, 主线程读后清零) */
    uint32_t m_pending_state = STATE_BOOTING;

    /* 去重标志 */
    bool     m_fault_triggered = false;
    std::atomic<bool> m_handshake_done{false};   /* 首次算法握手, 解耦 SHM 残留 seq */
    std::atomic<bool> m_active{false};           /* 校准完成后激活, 控制 ProcessMailbox/SafetyCheck */

    /* 延迟追踪 (STARK_LATENCY_TRACE=0 时零开销) */
    StarkLatencyTracer m_tracer;
};

}  /* namespace stark_periph_manager_node */
