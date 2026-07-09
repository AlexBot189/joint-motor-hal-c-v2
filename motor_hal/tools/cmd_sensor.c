/**
 * @file cmd_sensor.c
 * @brief 传感器透传命令: sensor config / sensor stop / sensor read
 */

#include "command_registry.h"
#include "tool_hal.h"
#include "daemon.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ================================================================
 * sensor config <id> <period_ms> [bus_fmt]
 *
 * period_div = period_ms * 4  (因为 250us tick)
 *   period_ms=0 ,  period_div=0 ,  关闭
 *   period_ms=1 ,  period_div=4 ,  1KHz
 *   period_ms=10 ,  period_div=40 ,  100Hz
 *
 * bus_fmt: 0=Classic CAN, 3=CANFD BRS (默认)
 * ================================================================ */

int cmd_do_sensor(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 3) {
        fprintf(stderr, "Usage: motor_tool sensor <config|stop|read> <id> [period_ms] [bus_fmt]\n");
        return -1;
    }

    const char *sub = argv[2];
    int id = (argc >= 4) ? atoi(argv[3]) : 1;

    if (strcmp(sub, "config") == 0 || strcmp(sub, "start") == 0) {
        if (argc < 5) {
            fprintf(stderr, "Usage: motor_tool sensor config <id> <period_ms> [bus_fmt]\n");
            fprintf(stderr, "  period_ms=0, 关闭, 1, 1ms/1KHz, 10, 10ms/100Hz\n");
            fprintf(stderr, "  bus_fmt: 0=Classic CAN, 3=CANFD BRS (默认)\n");
            return -1;
        }
        int period_ms = atoi(argv[4]);
        uint8_t bus_fmt = (argc >= 6) ? (uint8_t)atoi(argv[5]) : 3;  /* 默认 CANFD BRS */

        if (period_ms == 0) {
            printf("Stopping sensor passthrough for motor %d...\n", id);
            int ret = motor_hal_sensor_stop(g_hal, (uint8_t)id);
            if (ret == 0) printf("✓ Motor %d: sensor passthrough stopped\n", id);
            else fprintf(stderr, "✗ Motor %d: stop failed (ret=%d)\n", id, ret);
            return ret;
        }

        uint16_t period_div = (uint16_t)(period_ms * 4);  /* 250us tick */
        printf("Configuring sensor passthrough: motor=%d period=%dms(%dHz) bus=%s\n",
               id, period_ms, 1000 / period_ms,
               bus_fmt == 3 ? "CANFD BRS" : "Classic CAN");

        int ret = motor_hal_sensor_config(g_hal, (uint8_t)id, period_div, bus_fmt);
        if (ret == 0) printf("✓ Motor %d: sensor passthrough enabled\n", id);
        else fprintf(stderr, "✗ Motor %d: config failed (ret=%d)\n", id, ret);
        return ret;
    }

    if (strcmp(sub, "stop") == 0) {
        /* 先停止传感器看板 (如果有) */
        tool_sensor_watch_stop();
        printf("Stopping sensor passthrough for motor %d...\n", id);
        int ret = motor_hal_sensor_stop(g_hal, (uint8_t)id);
        if (ret == 0) printf("✓ Motor %d: sensor passthrough stopped\n", id);
        else fprintf(stderr, "✗ Motor %d: stop failed (ret=%d)\n", id, ret);
        return ret;
    }

    if (strcmp(sub, "read") == 0) {
        motor_sensor_t s;
        int ret = motor_hal_get_sensor(g_hal, (uint8_t)id, &s);
        if (ret != 0) {
            fprintf(stderr, "✗ Motor %d: sensor read failed (ret=%d)\n", id, ret);
            return ret;
        }
        printf("[%d] Hall: %4d %4d %4d | Force: %5d%s | Knee: %4d | SW: %d\n",
               id,
               s.hall_adc0, s.hall_adc1, s.hall_adc2,
               s.force_raw, s.data_valid ? "" : "(INVALID)",
               s.knee_hall, s.hw_sw_pc9);
        return 0;
    }

    if (strcmp(sub, "watch") == 0) {
        int out_fd = daemon_get_client_fd();
        int ret = tool_sensor_watch_start(id, out_fd);
        if (ret < 0) {
            fprintf(stderr, "✗ sensor watch already running or start failed\n");
            return ret;
        }
        printf("✓ Sensor watch started for motor %d (use 'sensor stop 1' to stop)\n", id);
        return 0;
    }

    fprintf(stderr, "Unknown sensor command: %s (use: config/stop/read/watch)\n", sub);
    return -1;
}
