/**
 * @file cmd_calib.c
 * @brief 电机校准: motor_tool calib <start|status|exit> [args]
 */

#include "command_registry.h"
#include "tool_hal.h"
#include "daemon.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

/* 引用 motor_calib 模块 */
#include "../src/motor_calib.h"

static motor_calib_t *g_calib = NULL;
static volatile int g_calib_running = 0;

static void _sigint_handler(int sig) { (void)sig; g_calib_running = 0; }

int cmd_do_calib(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }

    if (argc < 3) {
        fprintf(stderr, "Usage: motor_tool calib <start|status|exit> [id_r] [id_l] [timeout_ms]\n");
        return -1;
    }

    const char *sub = argv[2];

    if (strcmp(sub, "status") == 0) {
        if (!g_calib) {
            printf("Calib: not started\n");
            return 0;
        }
        motor_calib_state_t st = motor_calib_get_state(g_calib);
        const char *names[] = {"IDLE","ZEROING","CHECKING","DONE"};
        const char *name = (st >= 0 && st <= 3) ? names[st] : "TIMEOUT";
        printf("Calib state: %s (%d)\n", name, st);
        return (st == MOTOR_CALIB_DONE) ? 0 : 1;
    }

    if (strcmp(sub, "exit") == 0) {
        if (!g_calib) {
            printf("Calib: not started\n");
            return 0;
        }
        motor_calib_exit(g_calib);
        motor_calib_destroy(g_calib);
        g_calib = NULL;
        printf("Calib: exited\n");
        return 0;
    }

    if (strcmp(sub, "start") == 0) {
        /* 单电机: calib start <id> [timeout_ms]
         *   双电机: calib start <id_r> <id_l> [timeout_ms]
         *   id=0 表示跳过该电机 */
        if (argc < 4) {
            fprintf(stderr, "Usage: motor_tool calib start <id> [id_l] [timeout_ms]\n");
            fprintf(stderr, "  Single motor: motor_tool calib start 1\n");
            fprintf(stderr, "  Dual motor:   motor_tool calib start 1 2\n");
            fprintf(stderr, "  Skip one:     motor_tool calib start 1 0  (skip left)\n");
            return -1;
        }

        int id_r = atoi(argv[3]);
        int id_l = (argc >= 5) ? atoi(argv[4]) : 0;
        int timeout_ms = (argc >= 6) ? atoi(argv[5]) : 10000;

        if (g_calib) motor_calib_destroy(g_calib);
        g_calib = motor_calib_create(g_hal);
        if (!g_calib) {
            fprintf(stderr, "ERROR: calib create failed\n");
            return -1;
        }

        motor_calib_config_t cfg = {
            .motor_id_r          = (uint8_t)id_r,
            .motor_id_l          = (uint8_t)id_l,
            .timeout_ms          = timeout_ms,
            .angle_threshold_deg = 1.0f,
            .ctrl_mode           = MOTOR_MODE_CURRENT,
            .enable_after_done   = true,  /* motor_tool 校准后立即可用 */
        };

        if (motor_calib_start(g_calib, &cfg) != 0) {
            fprintf(stderr, "ERROR: calib start failed\n");
            return -1;
        }

        if (id_l > 0)
            printf("Calib started: R=%d L=%d timeout=%dms\n", id_r, id_l, timeout_ms);
        else
            printf("Calib started: motor=%d timeout=%dms\n", id_r, timeout_ms);
        printf("Polling... Press Ctrl+C to abort.\n\n");

        signal(SIGINT, _sigint_handler);
        g_calib_running = 1;

        while (g_calib_running) {
            motor_calib_state_t st = motor_calib_poll(g_calib);
            if (st == MOTOR_CALIB_DONE) {
                printf("\n✓ Calibration SUCCESS\n");
                g_calib_running = 0;
                return 0;
            }
            if (st == MOTOR_CALIB_TIMEOUT) {
                printf("\n✗ Calibration TIMEOUT\n");
                g_calib_running = 0;
                return 1;
            }
            usleep(20000);
        }

        printf("\nCalib aborted by user.\n");
        motor_calib_exit(g_calib);
        return 0;
    }

    fprintf(stderr, "Unknown calib command: %s (use: start/status/exit)\n", sub);
    return -1;
}
