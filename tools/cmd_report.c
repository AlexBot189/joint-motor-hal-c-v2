/**
 * @file cmd_report.c
 * @brief CA 数据上报: motor_tool report [period_ms]
 *
 * 等效 GD32 的 @CA 命令。
 * motor_tool report 5  → 每 5ms 输出反馈+传感器数据
 * motor_tool report 0  → 停止
 */

#include "command_registry.h"
#include "tool_hal.h"
#include "daemon.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

static volatile int g_report_running = 0;

static void _sigint_handler(int sig) { (void)sig; g_report_running = 0; }

static uint64_t _us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
}

int cmd_do_report(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }

    int period_ms = (argc >= 3) ? atoi(argv[2]) : 5;
    if (period_ms <= 0) {
        printf("Report stopped.\n");
        g_report_running = 0;
        return 0;
    }

    int motor_count = tool_motor_count();
    if (motor_count == 0) {
        fprintf(stderr, "ERROR: no motors registered. Use 'startup' first.\n");
        return -1;
    }

    printf("=== Data Report started (period=%dms) ===\n", period_ms);
    printf("  Motors: ");
    for (int i = 0; i < motor_count; i++) printf("%d ", tool_motor_id(i));
    printf("\n  Press Ctrl+C to stop.\n\n");

    signal(SIGINT, _sigint_handler);
    g_report_running = 1;
    uint64_t t0 = _us();
    uint64_t last = 0;

    while (g_report_running) {
        uint64_t now = _us();
        if (now - last >= (uint64_t)period_ms * 1000UL) {
            last = now;
            double ts = (double)(now - t0) / 1000000.0;

            /* 逐电机输出 */
            for (int i = 0; i < motor_count; i++) {
                int id = tool_motor_id(i);

                motor_feedback_t fb;
                int fb_ok = motor_hal_get_feedback(g_hal, (uint8_t)id, &fb);

                motor_sensor_t s;
                int s_ok  = motor_hal_get_sensor(g_hal, (uint8_t)id, &s);

                if (fb_ok == 0) {
                    printf("[%.3f] [%d] angle=%d speed=%d cur=%d temp=%d err=0x%04X state=0x%02X",
                           ts, id,
                           fb.position, fb.velocity, fb.current_iq,
                           fb.temperature, fb.error_code, fb.status_byte);
                } else {
                    printf("[%.3f] [%d] feedback=N/A", ts, id);
                }

                if (s_ok == 0) {
                    printf(" hall=%u/%u/%u force=%u knee=%u sw=%d valid=%d\n",
                           s.hall_adc0, s.hall_adc1, s.hall_adc2,
                           s.force_raw, s.knee_adc,
                           s.hw_sw_pc9, s.data_valid);
                } else {
                    printf(" sensor=N/A\n");
                }
            }

            fflush(stdout);
        }
    }

    printf("\n=== Report stopped ===\n");
    return 0;
}
