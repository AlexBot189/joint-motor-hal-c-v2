/**
 * @file cmd_report.c
 * @brief CA 数据上报: motor_tool report [period_ms]
 *
 * 等效的数据上报命令。在 daemon 中启动独立线程，不阻塞主循环。
 * motor_tool report 5  ,  每 5ms 输出 feedback+sensor
 * motor_tool report 0  ,  停止
 */

#include "command_registry.h"
#include "tool_hal.h"
#include "daemon.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

static pthread_t       g_report_thread;
static volatile bool   g_report_running = false;
static int             g_report_period_ms = 5;
static bool            g_report_active = false;  /* 区别于 running, 表示线程已启动 */

static uint64_t _us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
}

static void* _report_thread_fn(void *arg)
{
    (void)arg;
    uint64_t t0 = _us();
    uint64_t last = 0;
    int motor_count = tool_motor_count();

    while (g_report_running) {
        uint64_t now = _us();
        if (now - last >= (uint64_t)g_report_period_ms * 1000UL) {
            last = now;
            double ts = (double)(now - t0) / 1000000.0;

            for (int i = 0; i < motor_count; i++) {
                int id = tool_motor_id(i);
                motor_feedback_t fb;
                motor_sensor_t s;
                int fb_ok = motor_hal_get_feedback(g_hal, (uint8_t)id, &fb);
                int s_ok  = motor_hal_get_sensor(g_hal, (uint8_t)id, &s);

                if (fb_ok == 0) {
                    printf("[%.3f] [%d] angle=%d speed=%d cur=%d temp=%d err=0x%04X st=0x%02X",
                           ts, id, fb.position, fb.velocity, fb.current_iq,
                           fb.temperature, fb.error_code, fb.status_byte);
                } else {
                    printf("[%.3f] [%d] fb=N/A", ts, id);
                }
                if (s_ok == 0) {
                    printf(" hall=%u/%u/%u force=%u knee=%u sw=%d valid=%d\n",
                           s.hall_adc0, s.hall_adc1, s.hall_adc2,
                           s.force_raw, s.knee_adc, s.hw_sw_pc9, s.data_valid);
                } else {
                    printf(" sn=N/A\n");
                }
            }
            fflush(stdout);
        }
    }
    return NULL;
}

int cmd_do_report(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }

    int period_ms = (argc >= 3) ? atoi(argv[2]) : 5;

    if (period_ms <= 0) {
        /* 停止 */
        if (g_report_active) {
            g_report_running = false;
            pthread_join(g_report_thread, NULL);
            g_report_active = false;
            printf("✓ Report stopped.\n");
        } else {
            printf("Report not running.\n");
        }
        return 0;
    }

    if (g_report_active) {
        /* 已运行, 先停再重启 */
        g_report_running = false;
        pthread_join(g_report_thread, NULL);
        g_report_active = false;
    }

    int motor_count = tool_motor_count();
    if (motor_count == 0) {
        fprintf(stderr, "ERROR: no motors registered\n");
        return -1;
    }

    g_report_period_ms = period_ms;
    g_report_running = true;

    if (pthread_create(&g_report_thread, NULL, _report_thread_fn, NULL) != 0) {
        fprintf(stderr, "ERROR: thread create failed\n");
        g_report_running = false;
        return -1;
    }
    g_report_active = true;

    printf("✓ Report started (period=%dms, motors=", period_ms);
    for (int i = 0; i < motor_count; i++) printf("%d ", tool_motor_id(i));
    printf("). Use 'motor_tool report 0' to stop.\n");
    return 0;
}
