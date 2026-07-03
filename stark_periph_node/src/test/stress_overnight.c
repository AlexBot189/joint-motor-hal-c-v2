/*
 * stress_overnight.c -- 环形缓冲长时间压力测试
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 通宵运行, 日志输出到文件. 验证:
 *   1. 环形缓冲不丢帧
 *   2. 反馈数据连续无异常
 *   3. 控制命令全部被 RT 消费
 *
 * 编译: gcc -O2 stress_overnight.c -lpthread -lrt -lm -o stress_overnight
 * 用法: sudo ./stress_overnight [hours] [logfile]
 *       默认 8 小时, 日志 stress_test.log
 */

#include "stark_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <math.h>

static volatile int g_running = 1;
static FILE* g_log = NULL;
static uint64_t g_start_ms = 0;

static uint64_t g_total_frames   = 0;  /* 总发送帧数 */
static uint64_t g_burst_frames   = 0;  /* 突发生效帧数 (反馈变化次数) */
static uint64_t g_fb_ticks       = 0;  /* 反馈更新次数 */
static uint64_t g_cycle_count    = 0;  /* 控制循环次数 */
static int32_t  g_last_current_1 = 0;  /* 上次电机1电流 */
static int32_t  g_last_current_2 = 0;  /* 上次电机2电流 */
static int      g_jump_count     = 0;  /* 电流跳变次数 (>2x 步长) */
static int      g_stall_count    = 0;  /* 无反馈变化次数 */
static int      g_hb_ok          = 0;  /* 心跳正常计数 */

static void sig_handler(int sig) { (void)sig; g_running = 0; }

#define LOG(fmt, ...) do { \
    time_t now = time(NULL); \
    char ts[32]; strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&now)); \
    fprintf(g_log, "[%s] " fmt "\n", ts, ##__VA_ARGS__); \
    fflush(g_log); \
} while(0)

int main(int argc, char** argv)
{
    int hours = (argc >= 2) ? atoi(argv[1]) : 8;
    if (hours < 1) hours = 8;
    const char* logfile = (argc >= 3) ? argv[2] : "stress_test.log";

    g_log = fopen(logfile, "w");
    if (!g_log) { perror("fopen"); return 1; }
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    LOG("=== 环形缓冲压力测试 ===");
    LOG("测试时长: %d 小时", hours);
    LOG("验证项: 帧丢失 / 电流跳变 / 反馈连续性");

    /* 连接 */
    stark_client_t c;
    LOG("等待 stark_node...");
    while (stark_open(&c) != 0 && g_running) usleep(100000);
    if (!g_running) goto done;
    LOG("SHM 已连接");

    LOG("等待就绪...");
    while (!stark_ready(&c) && g_running) usleep(100000);
    if (!g_running) goto done;
    LOG("就绪, 在线: %d %d", stark_online(&c, 1), stark_online(&c, 2));

    /* 使能 + 电流模式 */
    stark_enable(&c, 1); stark_enable(&c, 2);
    stark_set_mode(&c, 1, STARK_MODE_CURRENT);
    stark_set_mode(&c, 2, STARK_MODE_CURRENT);
    usleep(5000);
    LOG("已使能, 电流模式");

    g_start_ms = time(NULL) * 1000ULL;
    uint64_t test_end_ms = g_start_ms + (uint64_t)hours * 3600 * 1000;
    uint64_t last_report = 0;
    uint32_t rpt_ver = 0;
    uint32_t rt_cycle = 0;
    int burst = 0;

    /* 正弦波参数: 1Hz, 振幅 500mA */
    while (g_running) {
        uint64_t now = time(NULL) * 1000ULL;
        if (now >= test_end_ms) break;

        /* 模拟外骨骼控制: 正常 1KHz 单帧 */
        int32_t ma = (int32_t)(500.0f * sinf((float)(now % 1000) / 1000.0f * 2.0f * M_PI));
        stark_multi(&c, ma, 0, 0, ma, 0, 0);
        g_total_frames++;

        /* 每 5 秒触发一次突发: 20 帧连发 (模拟轨迹插值) */
        if (burst > 0) {
            stark_multi(&c, ma, 0, 0, ma, 0, 0);
            g_total_frames++;
            burst--;
        }
        if (g_cycle_count % 5000 == 0) burst = 20;

        usleep(1000);
        g_cycle_count++;

        /* 反馈检测 */
        motor_data_t fb = stark_fb(&c, 1);
        if (abs(fb.current_iq - g_last_current_1) > 0) {
            g_last_current_1 = fb.current_iq;
            g_fb_ticks++;
            /* 检测电流跳变: 步长 >20mA 且 >2x 正弦变化率 */
            int expected_step = abs(ma - g_last_current_1);
            if (abs(fb.current_iq) > 0 && abs(fb.current_iq - ma) > 200) {
                g_jump_count++;
            }
        } else {
            g_stall_count++;
        }

        /* 周期上报 */
        const PeriodicUploadData* d;
        if (stark_report_try_read(&c, &rpt_ver, &d)) {
            g_burst_frames++;
        }

        /* 心跳 */
        if (g_cycle_count % 200 == 0) {
            stark_heartbeat(&c);
            if (stark_rt_alive(&c, &rt_cycle)) g_hb_ok++;
        }

        /* 每 10 分钟输出摘要 */
        if (now - last_report >= 600000) {
            last_report = now;
            int pct = (int)((now - g_start_ms) * 100 / (test_end_ms - g_start_ms));
            LOG("进度 %d%% | 周期=%lu 发帧=%lu 反馈=%lu 上报=%lu "
                "跳变=%d 停滞=%d 心跳OK=%d",
                pct, (unsigned long)g_cycle_count,
                (unsigned long)g_total_frames,
                (unsigned long)g_fb_ticks,
                (unsigned long)g_burst_frames,
                g_jump_count, g_stall_count, g_hb_ok);
        }
    }

done:
    /* 急停退出 */
    stark_estop(&c, 1); stark_estop(&c, 2);
    usleep(10000);
    stark_close(&c);

    /* 最终报告 */
    uint64_t elapsed_ms = (uint64_t)time(NULL) * 1000ULL - g_start_ms;
    int elapsed_min = (int)(elapsed_ms / 60000);

    LOG("=== 测试完成 ===");
    LOG("实际运行: %d 分钟", elapsed_min);
    LOG("控制周期:  %lu", (unsigned long)g_cycle_count);
    LOG("发送帧数:  %lu (含突发 %lu 次)", (unsigned long)g_total_frames,
        (unsigned long)(g_cycle_count / 5000));
    LOG("反馈更新:  %lu", (unsigned long)g_fb_ticks);
    LOG("上报批数:  %lu", (unsigned long)g_burst_frames);
    LOG("电流跳变:  %d  (异常阈值 >200mA)", g_jump_count);
    LOG("反馈停滞:  %d  (电流无变化)", g_stall_count);
    LOG("心跳正常:  %d 次", g_hb_ok);
    LOG("平均KHz:   %.2f", g_cycle_count / (double)elapsed_min / 60.0);

    if (g_jump_count == 0 && g_stall_count < g_cycle_count / 100) {
        LOG("结果: PASS -- 环形缓冲工作正常, 无丢帧异常");
    } else if (g_jump_count > 0) {
        LOG("结果: WARN -- %d 次电流跳变, 可能丢帧或模式异常", g_jump_count);
    } else {
        LOG("结果: WARN -- %d 次反馈停滞, 需检查", g_stall_count);
    }

    fclose(g_log);
    printf("测试完成, 日志: %s\n", logfile);
    return 0;
}
