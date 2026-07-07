/**
 * @file cmd_control.c
 * @brief SDO 控制命令: torque / speed / abs / abs_stop / limit / pid / setzero / sdoread/write
 *
 * 位置/速度/电流: tool 内部走完整 SDO 时序
 * 限位/零位: tool 内部自动失能
 * 其他: 单条 SDO
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * limit_pos <id> <deg*100> — 正限位 (自动失能, 写, save_flash)
 * ================================================================ */

int cmd_do_limit_pos(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id      = atoi(argv[2]);
    float deg = (float)atof(argv[3]);
    return tool_limit_pos_set(id, deg);
}

/* ================================================================
 * limit_neg <id> <deg*100> — 负限位 (自动失能, 写, save_flash)
 * ================================================================ */

int cmd_do_limit_neg(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id      = atoi(argv[2]);
    float deg = (float)atof(argv[3]);
    return tool_limit_neg_set(id, deg);
}

/* ================================================================
 * setzero <id> — 零位标定 (自动失能, 写0x2531)
 * ================================================================ */

int cmd_do_setzero(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id = atoi(argv[2]);
    return tool_set_zero_auto(id);
}

/* ================================================================
 * pid <id> <cp> <ci> <vp> <vi> <pp> <pi> — PID 设置
 * ================================================================ */

int cmd_do_pid(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
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
 * save <id> — 保存到 Flash
 * ================================================================ */

int cmd_do_save(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id = atoi(argv[2]);
    return tool_save_flash(id);
}

/* ================================================================
 * sdoread <id> <0xIndex> [subidx] — 通用 SDO 读
 * ================================================================ */

int cmd_do_sdo_read(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int      id     = atoi(argv[2]);
    uint16_t index  = (uint16_t)strtol(argv[3], NULL, 0);
    uint8_t  subidx = (argc >= 5) ? (uint8_t)atoi(argv[4]) : 0x00;
    return tool_sdo_read(id, index, subidx);
}

/* ================================================================
 * sdowrite <id> <0xIndex> <subidx> <value> <size> — 通用 SDO 写
 * ================================================================ */

int cmd_do_sdo_write(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int      id     = atoi(argv[2]);
    uint16_t index  = (uint16_t)strtol(argv[3], NULL, 0);
    uint8_t  subidx = (uint8_t)atoi(argv[4]);
    uint32_t value  = (uint32_t)strtoul(argv[5], NULL, 0);
    uint8_t  size   = (uint8_t)atoi(argv[6]);
    return tool_sdo_write(id, index, subidx, value, size);
}

/* ================================================================
 * stop [id] — 停止 daemon
 * ================================================================ */

int cmd_do_stop(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    int id = (argc >= 3) ? atoi(argv[2]) : 0;
    /* daemon stop: 全局清理, id 无用 */
    (void)id;
    printf("motor_tool daemon stopping...\n");
    return 0;
}
