/**
 * @file command_registry.c
 * @brief 命令注册表 + 分发逻辑
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <string.h>
#include <stdio.h>

/* ================================================================
 * 命令注册表
 * ================================================================ */

const command_entry_t g_cmd_table[] = {

    /* 系统命令 */
    { CMD_INIT,     "init",     "init <can_iface>",              "初始化 CANFD 接口",            2, 2 },
    { CMD_STARTUP,  "startup",  "startup <id>",                  "上电启动 (Bootup→NMT→DS402)",  2, 2 },
    { CMD_ENABLE,   "enable",   "enable <id>",                   "使能电机",                     2, 2 },
    { CMD_DISABLE,  "disable",  "disable <id>",                  "脱使能电机",                   2, 2 },
    { CMD_RESET,    "reset",    "reset <id>",                    "故障复位",                     2, 2 },

    /* SDO 控制 — 完整时序 */
    { CMD_TORQUE,   "torque",   "torque <id> <mA>",              "SDO电流控制(0~20000mA) 使能→电流模式→写0x6071", 3, 3 },
    { CMD_SPEED,    "speed",    "speed <id> <rpm*100> [acc*100] [dec*100]", "SDO速度控制 使能→PV→加减速→写0x60FF", 3, 5 },
    { CMD_ABS,      "abs",      "abs <id> <deg*100>",           "SDO位置控制 使能→PP→设参→目标→启动", 3, 3 },
    { CMD_ABS_STOP, "abs_stop", "abs_stop <id>",                "停止位置运动 (CW=0x0F)",       2, 2 },
    { CMD_ABS_ACCEL,"abs_accel","abs_accel <acc*100>",          "位置加减速 RPM/s×100 (默认2000)", 2, 2 },
    { CMD_ABS_SPEED,"abs_speed","abs_speed <rpm*100>",          "位置轨迹速度 RPM×100 输出端(默认10)", 2, 2 },

    /* SDO 单控 */
    { CMD_SETZERO,  "setzero",  "setzero <id>",                  "零位标定 (自动失能)",          2, 2 },
    { CMD_LIMIT_POS,"limit_pos","limit_pos <id> <deg*100>",     "正限位 (失能→写→Flash)",      3, 3 },
    { CMD_LIMIT_NEG,"limit_neg","limit_neg <id> <deg*100>",     "负限位 (失能→写→Flash)",      3, 3 },
    { CMD_LIMIT_POS_RD,"read_limit_pos","read_limit_pos <id>",  "读正限位 0x607D/02",           2, 2 },
    { CMD_LIMIT_NEG_RD,"read_limit_neg","read_limit_neg <id>",  "读负限位 0x607D/01",           2, 2 },
    { CMD_SAVE,     "save",     "save <id>",                     "保存参数到 Flash",             2, 2 },
    { CMD_PID,      "pid",      "pid <id> <cp> <ci> <vp> <vi> <pp> <pi>", "设置 PID 参数",   8, 8 },

    /* 调试 */
    { CMD_SDO_READ, "sdoread",  "sdoread <id> <0xIndex> [sub]", "通用 SDO 读",                 3, 4 },
    { CMD_SDO_WRITE,"sdowrite", "sdowrite <id> <0xIndex> <sub> <value> <size>", "通用 SDO 写", 6, 6 },

    /* 读取 */
    { CMD_READ,     "read",     "read <item> <id>",              "读: angle/speed/current/temp/state/error/version/pid/all", 3, 3 },

    /* 监控/传感器/上报/校准 */
    { CMD_WATCH,    "watch",    "watch <period_ms>",             "持续轮询反馈",                 2, 2 },
    { CMD_SENSOR,   "sensor",   "sensor <config|stop|read|watch> ...", "传感器透传",        -1, -1 },
    { CMD_REPORT,   "report",   "report [period_ms]",             "数据上报 (0=停止)",           2, 2 },
    { CMD_CALIB,    "calib",    "calib <start|status|exit> ...", "电机零位校准",                 -1, -1 },
    { CMD_TPDO_MAP, "tpdo_map", "tpdo_map <id> <cob> <ttype> <idx> <sub> <bits> ...", "TPDO映射", -1, -1 },
    { CMD_RPDO_MAP, "rpdo_map", "rpdo_map <id> <cob> <ttype> <idx> <sub> <bits> ...", "RPDO映射", -1, -1 },
    { CMD_RPDO_SEND, "rpdo_send", "rpdo_send <id> <hex_bytes...>", "RPDO发送", 3, -1 },
    { CMD_TPDO,  "tpdo",  "tpdo <id> <sync> <item> ...",  "TPDO快捷映射", 4, -1 },
    { CMD_RPDO,  "rpdo",  "rpdo <id> <ttype> <item> ...", "RPDO快捷映射", 4, -1 },
    { CMD_PDO,   "pdo",   "pdo <id> <pos|vel|cur|csp> <val>", "PDO单轴控制", 4, -1 },
    { CMD_MULTI,"multi","multi <pos|vel|cur|csp> <id:val> ...", "PDO多轴广播", 3, -1 },
    { CMD_MIT,  "mit",  "mit <id> <pos> <vel> <kp> <kd> <torque>", "MIT阻抗控制", 7, 7 },

    /* PDO Byte0 控制 */
    { CMD_PDO_ENABLE,  "pdo_enable",  "pdo_enable <id>",     "PDO使能 (Byte0 bit7=1)", 2, 2 },
    { CMD_PDO_DISABLE, "pdo_disable", "pdo_disable <id>",     "PDO失能 (Byte0 bit7=0)", 2, 2 },
    { CMD_BUS_ON,      "bus_on",       "bus_on <id>",         "母线接通 (Byte0 bit6=1, 预留)", 2, 2 },
    { CMD_BUS_OFF,     "bus_off",      "bus_off <id>",        "母线断开 (Byte0 bit6=0, 预留)", 2, 2 },
    { CMD_ESTOP,       "estop",        "estop <id>",          "急停: enable=0+bus=OFF", 2, 2 },
    { CMD_ESTOP_NOW,   "estop_now",     "estop_now <id>",    "急停+立即发帧", 2, 2 },
    { CMD_RECOVER,     "recover",      "recover <id>",        "恢复: bus=ON+enable=1", 2, 2 },
    { CMD_RECOVER_NOW, "recover_now",   "recover_now <id>",  "恢复+立即发帧", 2, 2 },
    { CMD_PDO_ENABLE_NOW, "pdo_enable_now", "pdo_enable_now <id>", "PDO使能+立即发帧", 2, 2 },
    { CMD_PDO_DISABLE_NOW,"pdo_disable_now","pdo_disable_now <id>","PDO失能+立即发帧", 2, 2 },
    { CMD_CLEARCF,     "clearcf",      "clearcf <id>",        "清故障脉冲 (Byte0 bit5)", 2, 2 },
    { CMD_SETMODE,     "setmode",      "setmode <id> <1~6>",  "PDO切换模式", 3, 3 },
    { CMD_BYTE0,       "byte0",        "byte0 <id> [0xHH]",   "读/写原始 Byte0", 2, 3 },

    /* 其他 */
    { CMD_FAULT_RESET,"fault_reset","fault_reset <id>",          "清零故障",                     2, 2 },
    { CMD_REBOOT,   "reboot",   "reboot <id>",                   "电机重启",                     2, 2 },
    { CMD_STOP,     "stop",     "stop",                           "停止 daemon",                   1, 1 },
    { CMD_PROBE,    "probe",    "probe [id]",                     "主动探测电机在线",              1, 2 },
    { CMD_HELP,     "help",     "help",                          "帮助",                         1, 1 },
};

const int g_cmd_count = sizeof(g_cmd_table) / sizeof(g_cmd_table[0]);

/* ================================================================
 * 查找
 * ================================================================ */

static const command_entry_t* _find_cmd(const char *name)
{
    for (int i = 0; i < g_cmd_count; i++)
        if (strcmp(g_cmd_table[i].name, name) == 0)
            return &g_cmd_table[i];
    return NULL;
}

/* ================================================================
 * 分发
 * ================================================================ */

int cmd_dispatch(motor_hal_t *hal, int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: motor_tool <command> [args...]\n");
        return 1;
    }

    const command_entry_t *cmd = _find_cmd(argv[1]);
    if (!cmd) {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        return 1;
    }

    if (argc - 1 < cmd->min_args || (cmd->max_args > 0 && argc - 1 > cmd->max_args)) {
        fprintf(stderr, "Usage: motor_tool %s\n", cmd->usage);
        return 1;
    }

    /* 读取命令: read_* 项特殊分发 */
    if (cmd->id == CMD_READ) {
        /* 用 cmd_do_read 统一处理, 内部根据 argv[2] 再分发到 tool_read_* */
        return cmd_do_read(hal, CMD_READ, argc, argv);
    }
    if (cmd->id == CMD_LIMIT_POS_RD) {
        int id = atoi(argv[2]);
        return tool_limit_pos_read(id);
    }
    if (cmd->id == CMD_LIMIT_NEG_RD) {
        int id = atoi(argv[2]);
        return tool_limit_neg_read(id);
    }

    switch (cmd->id) {
        case CMD_INIT:     return cmd_do_init(hal, cmd->id, argc, argv);
        case CMD_STARTUP:  return cmd_do_startup(hal, cmd->id, argc, argv);
        case CMD_ENABLE:   return cmd_do_enable(hal, cmd->id, argc, argv);
        case CMD_DISABLE:  return cmd_do_disable(hal, cmd->id, argc, argv);
        case CMD_RESET:    return cmd_do_reset(hal, cmd->id, argc, argv);
        case CMD_TORQUE:   return cmd_do_torque(hal, cmd->id, argc, argv);
        case CMD_SPEED:    return cmd_do_speed(hal, cmd->id, argc, argv);
        case CMD_ABS:      return cmd_do_abs(hal, cmd->id, argc, argv);
        case CMD_ABS_STOP: return cmd_do_abs_stop(hal, cmd->id, argc, argv);
        case CMD_ABS_ACCEL:return cmd_do_abs_accel(hal, cmd->id, argc, argv);
        case CMD_ABS_SPEED:return cmd_do_abs_speed(hal, cmd->id, argc, argv);
        case CMD_SETZERO:  return cmd_do_setzero(hal, cmd->id, argc, argv);
        case CMD_LIMIT_POS:return cmd_do_limit_pos(hal, cmd->id, argc, argv);
        case CMD_LIMIT_NEG:return cmd_do_limit_neg(hal, cmd->id, argc, argv);
        case CMD_SAVE:     return cmd_do_save(hal, cmd->id, argc, argv);
        case CMD_PID:      return cmd_do_pid(hal, cmd->id, argc, argv);
        case CMD_SDO_READ: return cmd_do_sdo_read(hal, cmd->id, argc, argv);
        case CMD_SDO_WRITE:return cmd_do_sdo_write(hal, cmd->id, argc, argv);
        case CMD_WATCH:    return cmd_do_watch(hal, cmd->id, argc, argv);
        case CMD_SENSOR:   return cmd_do_sensor(hal, cmd->id, argc, argv);
        case CMD_REPORT:   return cmd_do_report(hal, cmd->id, argc, argv);
        case CMD_CALIB:    return cmd_do_calib(hal, cmd->id, argc, argv);
        case CMD_TPDO_MAP: return cmd_do_tpdo_map(hal, cmd->id, argc, argv);
        case CMD_RPDO_MAP: return cmd_do_rpdo_map(hal, cmd->id, argc, argv);
        case CMD_RPDO_SEND: return cmd_do_rpdo_send(hal, cmd->id, argc, argv);
        case CMD_TPDO:     return cmd_do_tpdo_quick(hal, cmd->id, argc, argv);
        case CMD_RPDO:     return cmd_do_rpdo_quick(hal, cmd->id, argc, argv);
        case CMD_PDO:      return cmd_do_pdo(hal, cmd->id, argc, argv);
        case CMD_MULTI:    return cmd_do_multi(hal, cmd->id, argc, argv);
        case CMD_MIT:      return cmd_do_mit(hal, cmd->id, argc, argv);
        /* PDO Byte0 */
        case CMD_PDO_ENABLE:  return cmd_do_pdo_enable(hal, cmd->id, argc, argv);
        case CMD_PDO_DISABLE: return cmd_do_pdo_disable(hal, cmd->id, argc, argv);
        case CMD_BUS_ON:      return cmd_do_bus_on(hal, cmd->id, argc, argv);
        case CMD_BUS_OFF:     return cmd_do_bus_off(hal, cmd->id, argc, argv);
        case CMD_ESTOP:       return cmd_do_estop(hal, cmd->id, argc, argv);
        case CMD_ESTOP_NOW:   return cmd_do_estop_now(hal, cmd->id, argc, argv);
        case CMD_RECOVER:     return cmd_do_recover(hal, cmd->id, argc, argv);
        case CMD_RECOVER_NOW: return cmd_do_recover_now(hal, cmd->id, argc, argv);
        case CMD_PDO_ENABLE_NOW:  return cmd_do_pdo_enable_now(hal, cmd->id, argc, argv);
        case CMD_PDO_DISABLE_NOW: return cmd_do_pdo_disable_now(hal, cmd->id, argc, argv);
        case CMD_CLEARCF:     return cmd_do_clearcf(hal, cmd->id, argc, argv);
        case CMD_SETMODE:     return cmd_do_setmode(hal, cmd->id, argc, argv);
        case CMD_BYTE0:       return cmd_do_byte0(hal, cmd->id, argc, argv);
        case CMD_FAULT_RESET: return cmd_do_fault_reset(hal, cmd->id, argc, argv);
        case CMD_REBOOT:   return cmd_do_reboot(hal, cmd->id, argc, argv);
        case CMD_HELP:     return cmd_do_help(hal, cmd->id, argc, argv);
        case CMD_PROBE:    return cmd_do_probe(hal, cmd->id, argc, argv);
        case CMD_STOP:     return cmd_do_stop(hal, cmd->id, argc, argv);
        default: return 1;
    }
}
