/*
 * @file exo_rt_worker.h
 * @brief RT 工作线程 — 控制下发 + 反馈上报 + 安全监控 (三合一)
 *
 * 线程属性:
 *   调度策略: SCHED_FIFO
 *   优先级:   90
 *   周期:     1KHz (1000μs)
 *   核心:     CPU affinity 可配 (默认 core 2/3)
 *
 * 单周期流程 (约束 < 50μs, 1ms 预算的 5%):
 *   ① ProcessMailbox()   → 读 SHM mailbox → motor_hal_multi_ctrl
 *   ② PublishFeedback()  → 读 fb_cache → 每 5 周期写 SHM double buffer (200Hz)
 *   ③ SafetyCheck()      → seq 停滞 / 编码器 / 过温 / CAN 断线
 *
 * 依赖:
 *   motor_hal_t*  — HAL 反馈缓存 + PDO 控制
 *   exo_shm_t*    — 共享内存 (mailbox 读, fb_buffer 写, 状态写)
 */
#pragma once

#include <atomic>
#include <thread>
#include <cstdint>
#include <string>

extern "C" {
#include "motor_hal.h"
#include "exo_shm.h"
}

namespace stark_periph_manager_node
{

/* ── 安全监控配置 (可覆盖默认值) ── */
struct SafetyConfig {
    uint32_t algo_timeout_ms   = 200;    /* seq 停滞 → WARN      */
    uint32_t algo_shutdown_ms  = 500;    /* seq 停滞 → FAULT     */
    int32_t  overtemp_celsius  = 80;     /* 过温阈值, °C         */
    uint32_t can_offline_ms    = 2000;   /* CAN 断线 → FAULT     */
    uint32_t encoder_stall_s   = 3;      /* 编码器停滞 → FAULT   */
};

/* ── RT 工作线程配置 ── */
struct RtConfig {
    int      priority         = 90;
    uint32_t period_us        = 1000;
    int      report_divider   = 5;       /* 5 周期 → 200Hz 上报  */
    int      cpu_affinity[2]  = {2, 3};  /* core 亲和性           */
};

class ExoRtWorker {
public:
    ExoRtWorker(motor_hal_t* hal, exo_shm_t* shm);
    ~ExoRtWorker();

    /* ── 生命周期 ── */
    void Start();                       /* 创建 RT 线程 */
    void Stop();                        /* 停止 + join */
    bool IsRunning() const { return m_running.load(std::memory_order_acquire); }

    /* ── 配置 ── */
    void SetSafetyConfig(const SafetyConfig& cfg) { m_safety = cfg; }
    void SetRtConfig(const RtConfig& cfg)         { m_rt = cfg; }

    /* ── 诊断 ── */
    uint64_t GetCycleCount() const { return m_cycle_count; }
    uint64_t GetOverrunCount() const { return m_overrun_count; }

private:
    /* ── RT 线程主循环 ── */
    void Run();

    /* ── 子步骤 ── */
    bool ProcessMailbox();              /* 读 SHM mailbox → multi_ctrl */
    void PublishFeedback();             /* 读 fb_cache → 写 SHM double buffer */
    void SafetyCheck();                 /* 安全监控: seq/编码器/过温/CAN */

    /* ── 辅助 ── */
    void SetThreadRt();                 /* 设置调度策略 + 优先级 + 名称 */
    void HandleTimeout(uint32_t ms);    /* 超时动作: WARN/FAULT */

    /* ── 外部依赖 ── */
    motor_hal_t*    m_hal;              /* HAL 实例 */
    exo_shm_t*      m_shm;              /* 共享内存映射 */

    /* ── 线程控制 ── */
    std::atomic<bool> m_running{false};
    std::thread       m_thread;

    /* ── 配置 ── */
    SafetyConfig m_safety;
    RtConfig     m_rt;

    /* ── Safety 状态跟踪 ── */
    uint64_t m_last_seq;                /* 上次读到的 seq_begin 值 */
    uint64_t m_seq_stall_us;            /* seq 停滞起始时间戳 (μs) */
    uint64_t m_can_last_frame_us;       /* 最后收到 CAN 帧的时间 */

    /* ── 周期控制 ── */
    int      m_report_divider;          /* 上报分频计数器 (倒计数) */
    uint64_t m_cycle_count;             /* 总周期计数 */
    uint64_t m_overrun_count;           /* 超时计数 (调试) */

    /* ── 编码器停滞检测 ── */
    int16_t  m_last_position[4];        /* 上次位置快照, 按 motor index */
    uint64_t m_pos_stall_us[4];         /* 位置停滞起始时间 */
};

}  /* namespace stark_periph_manager_node */
