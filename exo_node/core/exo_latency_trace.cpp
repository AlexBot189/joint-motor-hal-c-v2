/*
 * exo_latency_trace.cpp — 耗时追踪实现 (dump_stats / reset_stats)
 */
#include "exo_latency_trace.h"
#include <cstdio>
#include <cmath>

namespace stark_periph_manager_node {

#if EXO_LATENCY_TRACE

void ExoLatencyTracer::reset_stats()
{
    m_fb_read_min  = UINT32_MAX; m_fb_read_max  = 0; m_fb_read_acc  = 0; m_fb_read_sq  = 0;
    m_mock_snsr_min = UINT32_MAX; m_mock_snsr_max = 0; m_mock_snsr_acc = 0; m_mock_snsr_sq = 0;
    m_shm_write_min = UINT32_MAX; m_shm_write_max = 0; m_shm_write_acc = 0; m_shm_write_sq = 0;
    m_fb_total_min  = UINT32_MAX; m_fb_total_max  = 0; m_fb_total_acc  = 0; m_fb_total_sq  = 0;
    m_ctrl_total_min = UINT32_MAX; m_ctrl_total_max = 0; m_ctrl_total_acc = 0; m_ctrl_total_sq = 0;
}

void ExoLatencyTracer::dump_stats(uint32_t n)
{
    if (n == 0) return;

    auto calc_std = [](uint64_t sq, uint64_t acc, uint32_t cnt) -> uint32_t {
        double mean = (double)acc / cnt;
        double var = (double)sq / cnt - mean * mean;
        return (var > 0) ? (uint32_t)sqrt(var) : 0;
    };

    printf("\n┌─ [LatencyTrace] 1000-cycle stats ─────────────────────────┐\n");
    printf("│ frames: %u                                                    │\n", n);

    printf("├─ 反馈路径 (PublishFeedback)                                ─┤\n");
    printf("│   fb_cache读  : min=%3uus  avg=%3uus  max=%3uus  σ=%3uus   │\n",
           m_fb_read_min, (uint32_t)(m_fb_read_acc / n), m_fb_read_max,
           calc_std(m_fb_read_sq, m_fb_read_acc, n));
    printf("│   mock传感器  : min=%3uus  avg=%3uus  max=%3uus  σ=%3uus   │\n",
           m_mock_snsr_min, (uint32_t)(m_mock_snsr_acc / n), m_mock_snsr_max,
           calc_std(m_mock_snsr_sq, m_mock_snsr_acc, n));
    printf("│   SHM写入     : min=%3uus  avg=%3uus  max=%3uus  σ=%3uus   │\n",
           m_shm_write_min, (uint32_t)(m_shm_write_acc / n), m_shm_write_max,
           calc_std(m_shm_write_sq, m_shm_write_acc, n));
    printf("│   反馈总延迟  : min=%3uus  avg=%3uus  max=%3uus  σ=%3uus   │\n",
           m_fb_total_min, (uint32_t)(m_fb_total_acc / n), m_fb_total_max,
           calc_std(m_fb_total_sq, m_fb_total_acc, n));

    printf("├─ 控制路径 (ProcessMailbox → PDO)                           ─┤\n");
    printf("│   控制总延迟  : min=%3uus  avg=%3uus  max=%3uus  σ=%3uus   │\n",
           m_ctrl_total_min, (uint32_t)(m_ctrl_total_acc / n), m_ctrl_total_max,
           calc_std(m_ctrl_total_sq, m_ctrl_total_acc, n));

    printf("├─ 单周期合计   : 反馈+控制 avg=%3uus  max=%3uus              │\n",
           (uint32_t)((m_fb_total_acc + m_ctrl_total_acc) / n),
           m_fb_total_max + m_ctrl_total_max);
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

#endif  /* EXO_LATENCY_TRACE */

}  /* namespace stark_periph_manager_node */
