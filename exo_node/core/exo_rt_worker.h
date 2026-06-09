/*
 * exo_rt_worker.h — RT 工作线程
 *
 * 三合一: 控制下发 + 反馈上报 + 安全监控
 *
 * 周期:   1KHz (1000μs)
 * 调度:   SCHED_FIFO, prio=90
 * 名称:   "exo_rt"
 *
 * 单周期时序 (< 80μs, 1ms 预算的 8%):
 *   ① ProcessMailbox()   → 读 SHM mailbox → PDO multi_ctrl      ~25μs
 *   ② PublishFeedback()  → 读 fb_cache → SHM double buffer      ~30μs (每5周期)
 *   ③ SafetyCheck()      → seq停滞 / torque归零 (PDO)           ~10μs
 *   ④ MockSensorTick()   → 模拟 IMU + 气压计                     ~5μs
 *
 * 延迟追踪: 在每个关键节点往 feedback_frame_t 打时间戳,
 *           支持端到端 (T0~T8) 闭环延迟分析.
 */
#pragma once

#include <atomic>
#include <thread>
#include <cstdint>

extern "C" {
#include "motor_hal.h"
#include "exo_shm.h"
}

namespace stark_periph_manager_node {

class ExoMotorCtrl;
class ExoMockSensor;

struct SafetyConfig {
    uint32_t algo_timeout_ms   = 200;
    uint32_t algo_shutdown_ms  = 500;
    int32_t  overtemp_celsius  = 80;
    uint32_t can_offline_ms    = 2000;
    uint32_t encoder_stall_s   = 3;
};

struct RtConfig {
    int      priority         = 90;
    uint32_t period_us        = 1000;
    int      report_divider   = 5;     /* 5周期 → 200Hz */
    int      cpu_affinity[2]  = {2, 3};
};

class ExoRtWorker {
public:
    ExoRtWorker(motor_hal_t* hal, exo_shm_t* shm,
                ExoMotorCtrl* ctrl, ExoMockSensor* mock_sensor);
    ~ExoRtWorker();

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
    exo_state_t GetPendingState();

private:
    void Run();
    void ProcessMailbox();
    void PublishFeedback();
    void SafetyCheck();
    void MockSensorTick();
    void SetThreadRt();

    /* ── 双电机 torque=0 (PDO 路径, RT 安全) ── */
    void _safe_torque_zero_all();
    void _safe_disable_all();

    motor_hal_t*    m_hal;
    exo_shm_t*      m_shm;
    ExoMotorCtrl*   m_ctrl;
    ExoMockSensor*  m_mock_sensor;

    std::atomic<bool> m_running{false};
    std::thread       m_thread;

    SafetyConfig m_safety;
    RtConfig     m_rt;

    /* ── 延迟追踪 ── */
    uint64_t m_last_seq;
    uint64_t m_seq_stall_us;
    uint64_t m_can_last_frame_us;
    uint64_t m_latency_history[64];     /* 最近64次闭环延迟 (T8-T0) */
    uint32_t m_latency_idx;

    /* ── 周期控制 ── */
    int      m_report_divider;
    uint64_t m_cycle_count;
    uint64_t m_overrun_count;

    /* ── 停滞检测 ── */
    int16_t  m_last_position[EXO_MOTOR_COUNT];
    uint64_t m_pos_stall_us[EXO_MOTOR_COUNT];

    /* ── 一次性日志 ── */
    bool     m_sensor_notified[EXO_MOTOR_COUNT];
    bool     m_imu_notified;

    /* ── RT→主线程 状态切换请求 (atomic, RT 写, 主线程读后清零) ── */
    uint32_t m_pending_state = STATE_INIT;
};

}  /* namespace stark_periph_manager_node */
