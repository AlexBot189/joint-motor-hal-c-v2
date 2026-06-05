/**
 * @file command_registry.h
 * @brief motor_tool 命令注册表
 */

#ifndef COMMAND_REGISTRY_H
#define COMMAND_REGISTRY_H

#include "motor_hal.h"

/* ================================================================
 * 命令编号 (用于 switch 分发)
 * ================================================================ */

enum {
    CMD_INIT, CMD_STARTUP, CMD_ENABLE, CMD_DISABLE, CMD_RESET,
    CMD_SPEED, CMD_ACCEL, CMD_ABS, CMD_REL, CMD_MAXV, CMD_STOP, CMD_MODE,
    CMD_TORQUE, CMD_CSP, CMD_MIT, CMD_BRAKE, CMD_QUICKSTOP,
    CMD_SAVE, CMD_SETZERO, CMD_PID,
    CMD_SDO_READ, CMD_SDO_WRITE,
    CMD_READ, CMD_WATCH, CMD_SENSOR, CMD_REPORT, CMD_CALIB,
    CMD_FAULT_RESET, CMD_REBOOT,
    CMD_HELP,
    CMD_COUNT
};

/* ================================================================
 * 命令条目
 * ================================================================ */

typedef struct {
    int         id;         /* CMD_xxx 枚举值 */
    const char *name;       /* 命令名, 如 "speed" */
    const char *usage;      /* 用法, 如 "speed <id> <rpm*100>" */
    const char *help;       /* 帮助文本 */
    int         min_args;   /* 最小参数 (含命令名自身) */
    int         max_args;   /* 最大参数 (-1=不限) */
} command_entry_t;

/* ================================================================
 * 全局注册表
 * ================================================================ */

extern const command_entry_t g_cmd_table[];
extern const int             g_cmd_count;

/* ================================================================
 * 命令处理函数
 * ================================================================ */

struct motor_hal;
typedef int (*cmd_handler_t)(motor_hal_t *hal, int cmd_id, int argc, char **argv);

int cmd_do_init(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_startup(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_enable(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_disable(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_speed(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_accel(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_abs(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_rel(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_maxv(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_stop(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_mode(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_torque(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_csp(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_mit(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_brake(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_quickstop(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_save(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_setzero(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_pid(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_sdo_read(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_sdo_write(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_read(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_watch(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_help(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_sensor(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_report(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_calib(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_fault_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_reboot(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* ================================================================
 * 命令分发: 根据命令名查找并执行回调
 * ================================================================ */

int cmd_dispatch(motor_hal_t *hal, int argc, char **argv);

#endif /* COMMAND_REGISTRY_H */
