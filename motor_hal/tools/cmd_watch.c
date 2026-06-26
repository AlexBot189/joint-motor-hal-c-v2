/**
 * @file cmd_watch.c
 * @brief watch 命令: 持续轮询显示反馈
 */

#include "command_registry.h"
#include "tool_hal.h"
#include "daemon.h"
#include <stdio.h>
#include <stdlib.h>

int cmd_do_watch(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;

    int period_ms = atoi(argv[2]);
    if (period_ms <= 0) {
        fprintf(stderr, "Invalid period: %d (must be >0 ms)\n", period_ms);
        return -1;
    }

    /* watch 输出走 daemon 的 client socket */
    int out_fd = daemon_get_client_fd();
    return tool_watch_start(period_ms, out_fd);
}
