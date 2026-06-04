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

/* ================================================================
 * torque <id> <mA> — 电流/力矩控制
 * ================================================================ */

int cmd_do_torque(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int id = atoi(argv[2]);
    int ma = atoi(argv[3]);
    return tool_set_torque(id, ma);
}

/* ================================================================
 * csp <id> <deg*100> — CSP 同步位置控制
 * ================================================================ */

int cmd_do_csp(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int id = atoi(argv[2]);
    int deg = atoi(argv[3]);
    return tool_set_csp(id, deg);
}

/* ================================================================
 * mit <id> <pos> <vel> <kp> <kd> <torque> — MIT 阻抗控制
 * ================================================================ */

int cmd_do_mit(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int  id     = atoi(argv[2]);
    float pos   = atof(argv[3]);
    float vel   = atof(argv[4]);
    float kp    = atof(argv[5]);
    float kd    = atof(argv[6]);
    float torque = atof(argv[7]);
    return tool_set_mit(id, pos, vel, kp, kd, torque);
}

/* ================================================================
 * brake <id> <release|lock> — 抱闸控制
 * ================================================================ */

int cmd_do_brake(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int  id      = atoi(argv[2]);
    bool release = (strcmp(argv[3], "release") == 0 || strcmp(argv[3], "1") == 0);
    return tool_set_brake(id, release);
}

/* ================================================================
 * quickstop <id> — 急停
 * ================================================================ */

int cmd_do_quickstop(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int id = atoi(argv[2]);
    return tool_quickstop(id);
}

/* ================================================================
 * save <id> — 保存到 Flash
 * ================================================================ */

int cmd_do_save(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int id = atoi(argv[2]);
    return tool_save_flash(id);
}

/* ================================================================
 * setzero <id> — 零位标定
 * ================================================================ */

int cmd_do_setzero(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int id = atoi(argv[2]);
    return tool_set_zero(id);
}

/* ================================================================
 * pid <id> <cp> <ci> <vp> <vi> <pp> <pi> — PID 设置
 * ================================================================ */

int cmd_do_pid(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int      id = atoi(argv[2]);
    uint16_t cp = (uint16_t)atoi(argv[3]);
    uint16_t ci = (uint16_t)atoi(argv[4]);
    uint16_t vp = (uint16_t)atoi(argv[5]);
    uint16_t vi = (uint16_t)atoi(argv[6]);
    uint16_t pp = (uint16_t)atoi(argv[7]);
    uint16_t pi = (uint16_t)atoi(argv[8]);
    return tool_set_pid(id, cp, ci, vp, vi, pp, pi);
}

/* ================================================================
 * sdoread <id> <index_hex> [subidx] — 通用 SDO 读
 * ================================================================ */

int cmd_do_sdo_read(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int      id     = atoi(argv[2]);
    uint16_t index  = (uint16_t)strtol(argv[3], NULL, 0);  /* 0x 前缀自动识别 */
    uint8_t  subidx = (argc >= 5) ? (uint8_t)atoi(argv[4]) : 0x00;
    return tool_sdo_read(id, index, subidx);
}

/* ================================================================
 * sdowrite <id> <index_hex> <subidx> <value> <size> — 通用 SDO 写
 * ================================================================ */

int cmd_do_sdo_write(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    int      id     = atoi(argv[2]);
    uint16_t index  = (uint16_t)strtol(argv[3], NULL, 0);
    uint8_t  subidx = (uint8_t)atoi(argv[4]);
    uint32_t value  = (uint32_t)strtoul(argv[5], NULL, 0);
    uint8_t  size   = (uint8_t)atoi(argv[6]);
    return tool_sdo_write(id, index, subidx, value, size);
}
