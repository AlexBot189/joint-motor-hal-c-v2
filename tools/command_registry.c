/**
 * @file command_registry.c
 * @brief 命令注册表 + 分发逻辑
 */

#include "command_registry.h"
#include <string.h>
#include <stdio.h>

/* ================================================================
 * 命令注册表 — 加命令只需加一行
 * ================================================================ */

const command_entry_t g_cmd_table[] = {
    /* { id,        name,       usage,                          help,                           min, max } */

    /* 系统命令 */
    { CMD_INIT,     "init",    "init <can_iface>",              "初始化 CANFD 接口 (can0)",     2,   2 },
    { CMD_STARTUP,  "startup", "startup <id>",                  "上电启动 (Bootup→关狗→使能)",   2,   2 },
    { CMD_ENABLE,   "enable",  "enable <id>",                   "使能电机 (DS402)",             2,   2 },
    { CMD_DISABLE,  "disable", "disable <id>",                  "脱使能电机",                   2,   2 },
    { CMD_RESET,    "reset",   "reset <id>",                    "故障复位",                     2,   2 },

    /* 控制命令 */
    { CMD_SPEED,    "speed",   "speed <id> <rpm*100>",         "设置速度 (RPM×100)",           3,   3 },
    { CMD_ACCEL,    "accel",   "accel <id> <acc*100>",         "设置加减速度 (RPM/s×100)",     3,   3 },
    { CMD_ABS,      "abs",     "abs <id> <deg*100>",           "绝对位置 (度×100)",            3,   3 },
    { CMD_REL,      "rel",     "rel <id> <delta_deg*100>",     "相对位置 (度×100)",            3,   3 },
    { CMD_MAXV,     "maxv",    "maxv <id> <rpm*100>",          "位置模式最大轨迹速度",         3,   3 },
    { CMD_STOP,     "stop",    "stop [id]",                     "停止电机 (默认 id=0)",         1,   2 },
    { CMD_MODE,     "mode",    "mode <id> <pp|pv|csp|csv|cur>","切换控制模式",                 3,   3 },

    /* 读取命令 */
    { CMD_READ,     "read",    "read <item> <id>",              "读取: angle/speed/current/temp/state/error/version/all", 3, 3 },

    /* 持续监控 */
    { CMD_WATCH,    "watch",   "watch <period_ms>",             "持续轮询显示反馈 (Ctrl+C 退出)", 2, 2 },

    /* 帮助 */
    { CMD_HELP,     "help",    "help",                          "显示此帮助",                   1,   1 },
};

const int g_cmd_count = sizeof(g_cmd_table) / sizeof(g_cmd_table[0]);

/* ================================================================
 * 命令名 → ID 查找
 * ================================================================ */

static const command_entry_t* _find_cmd(const char *name)
{
    for (int i = 0; i < g_cmd_count; i++) {
        if (strcmp(g_cmd_table[i].name, name) == 0)
            return &g_cmd_table[i];
    }
    return NULL;
}

/* ================================================================
 * 分发
 * ================================================================ */

int cmd_dispatch(motor_hal_t *hal, int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: motor_tool <command> [args...]\n");
        fprintf(stderr, "       motor_tool help\n");
        return 1;
    }

    const command_entry_t *cmd = _find_cmd(argv[1]);
    if (!cmd) {
        fprintf(stderr, "Unknown command: %s\nTry 'motor_tool help'\n", argv[1]);
        return 1;
    }

    if (argc - 1 < cmd->min_args || (cmd->max_args > 0 && argc - 1 > cmd->max_args)) {
        fprintf(stderr, "Usage: motor_tool %s\n", cmd->usage);
        return 1;
    }

    /* 静态函数指针表 — 用 switch 编译期常量有更好的 inlining */
    switch (cmd->id) {
        case CMD_INIT:    return cmd_do_init(hal, cmd->id, argc, argv);
        case CMD_STARTUP: return cmd_do_startup(hal, cmd->id, argc, argv);
        case CMD_ENABLE:  return cmd_do_enable(hal, cmd->id, argc, argv);
        case CMD_DISABLE: return cmd_do_disable(hal, cmd->id, argc, argv);
        case CMD_RESET:   return cmd_do_reset(hal, cmd->id, argc, argv);
        case CMD_SPEED:   return cmd_do_speed(hal, cmd->id, argc, argv);
        case CMD_ACCEL:   return cmd_do_accel(hal, cmd->id, argc, argv);
        case CMD_ABS:     return cmd_do_abs(hal, cmd->id, argc, argv);
        case CMD_REL:     return cmd_do_rel(hal, cmd->id, argc, argv);
        case CMD_MAXV:    return cmd_do_maxv(hal, cmd->id, argc, argv);
        case CMD_STOP:    return cmd_do_stop(hal, cmd->id, argc, argv);
        case CMD_MODE:    return cmd_do_mode(hal, cmd->id, argc, argv);
        case CMD_READ:    return cmd_do_read(hal, cmd->id, argc, argv);
        case CMD_WATCH:   return cmd_do_watch(hal, cmd->id, argc, argv);
        case CMD_HELP:    return cmd_do_help(hal, cmd->id, argc, argv);
        default:          return 1;
    }
}
