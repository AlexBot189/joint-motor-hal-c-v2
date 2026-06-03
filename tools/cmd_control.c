/**
 * @file cmd_control.c
 * @brief 控制命令: speed / accel / abs / rel / maxv / stop / mode
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 * 通用: 解析 id + value_x100, 反查电机是否存在
 * ================================================================ */

static int _parse_id_and_val(int argc, char **argv, int *id, int *val_x100)
{
    *id       = atoi(argv[2]);
    *val_x100 = atoi(argv[3]);
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    return 0;
}

/* ================================================================
 * speed <id> <rpm*100>
 * ================================================================ */

int cmd_do_speed(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    int id, val;
    if (_parse_id_and_val(argc, argv, &id, &val) < 0) return -1;
    return tool_set_speed(id, val);
}

/* ================================================================
 * accel <id> <acc*100>
 * ================================================================ */

int cmd_do_accel(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    int id, val;
    if (_parse_id_and_val(argc, argv, &id, &val) < 0) return -1;
    return tool_set_accel(id, val);
}

/* ================================================================
 * abs <id> <deg*100>
 * ================================================================ */

int cmd_do_abs(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    int id, val;
    if (_parse_id_and_val(argc, argv, &id, &val) < 0) return -1;
    return tool_set_abs_pos(id, val);
}

/* ================================================================
 * rel <id> <delta_deg*100>
 * ================================================================ */

int cmd_do_rel(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    int id, val;
    if (_parse_id_and_val(argc, argv, &id, &val) < 0) return -1;
    return tool_set_rel_pos(id, val);
}

/* ================================================================
 * maxv <id> <rpm*100>
 * ================================================================ */

int cmd_do_maxv(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    int id, val;
    if (_parse_id_and_val(argc, argv, &id, &val) < 0) return -1;
    return tool_set_max_vel(id, val);
}

/* ================================================================
 * stop [id]
 * ================================================================ */

int cmd_do_stop(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }

    int id = (argc >= 3) ? atoi(argv[2]) : 0;  /* 默认 id=0 */
    return tool_stop(id);
}

/* ================================================================
 * mode <id> <pp|pv|csp|csv|cur>
 * ================================================================ */

int cmd_do_mode(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }

    int id = atoi(argv[2]);
    return tool_set_mode(id, argv[3]);
}
