/**
 * @file command_registry.h
 * @brief motor_tool 命令注册表
 */

#ifndef COMMAND_REGISTRY_H
#define COMMAND_REGISTRY_H

#include "motor_hal.h"

enum {
    /* 系统命令 */
    CMD_INIT, CMD_STARTUP, CMD_ENABLE, CMD_DISABLE, CMD_RESET,

    /* SDO 控制 (完整时序) */
    CMD_TORQUE, CMD_SPEED, CMD_ABS, CMD_ABS_STOP,
    CMD_ABS_ACCEL, CMD_ABS_SPEED,

    /* SDO 单控 */
    CMD_SETZERO, CMD_LIMIT_POS, CMD_LIMIT_NEG, CMD_LIMIT_POS_RD, CMD_LIMIT_NEG_RD,
    CMD_SAVE, CMD_PID,

    /* 调试 */
    CMD_SDO_READ, CMD_SDO_WRITE,

    /* 读取 */
    CMD_READ,

    /* 持续监控 */
    CMD_WATCH,

    /* 传感器 */
    CMD_SENSOR,

    /* 数据上报 */
    CMD_REPORT,

    /* 校准 */
    CMD_CALIB,

    /* PDO 映射 */
    CMD_TPDO_MAP, CMD_RPDO_MAP,

    /* 其他系统 */
    CMD_FAULT_RESET, CMD_REBOOT,
    CMD_STOP, CMD_HELP,

    CMD_COUNT
};

typedef struct {
    int         id;
    const char *name;
    const char *usage;
    const char *help;
    int         min_args;
    int         max_args;
} command_entry_t;

extern const command_entry_t g_cmd_table[];
extern const int             g_cmd_count;

typedef int (*cmd_handler_t)(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* 系统 */
int cmd_do_init(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_startup(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_enable(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_disable(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* SDO 控制 */
int cmd_do_torque(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_speed(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_abs(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_abs_stop(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_abs_accel(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_abs_speed(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* SDO 单控 */
int cmd_do_setzero(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_limit_pos(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_limit_neg(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_save(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_pid(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* 调试 */
int cmd_do_sdo_read(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_sdo_write(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* 读取 */
int cmd_do_read(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* 监控/传感器/上报/校准 */
int cmd_do_watch(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_sensor(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_report(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_calib(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* PDO 映射 */
int cmd_do_tpdo_map(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_rpdo_map(motor_hal_t *hal, int cmd_id, int argc, char **argv);

/* 其他 */
int cmd_do_help(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_fault_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_reboot(motor_hal_t *hal, int cmd_id, int argc, char **argv);
int cmd_do_stop(motor_hal_t *hal, int cmd_id, int argc, char **argv);

int cmd_dispatch(motor_hal_t *hal, int argc, char **argv);

#endif /* COMMAND_REGISTRY_H */
