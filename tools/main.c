/**
 * @file main.c
 * @brief motor_tool — CANFD CANopen 电机控制工具入口
 *
 * 模式:
 *   motor_tool daemon can0   → 启动守护进程
 *   motor_tool stop           → 停止守护进程
 *   motor_tool help           → 显示帮助 (本地)
 *   motor_tool <cmd> [args]  → 通过 socket 向 daemon 发命令
 *   motor_tool                → 显示用法
 */

#include "daemon.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern void print_quick_usage(void);  /* from cmd_help.c */

static void print_usage(void)
{
    print_quick_usage();  /* 统一从 cmd_help.c 输出 */
}

static void print_help(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║  motor_tool — CANFD CANopen 电机控制工具        ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");
    printf("用法: motor_tool <command> [args...]\n\n");

    printf("── 系统命令 ──────────────────────────────────────\n");
    printf("  daemon <can>         启动守护进程\n");
    printf("  stop                 停止守护进程\n");
    printf("  startup <id>         上电启动 (0=双电机)\n");
    printf("  enable <id>          使能电机\n");
    printf("  disable <id>         脱使能\n");
    printf("  reset <id>           故障复位\n\n");

    printf("── 控制命令 (SDO, 自动时序) ×100 精度 ────────\n");
    printf("  torque <id> <mA>           电流控制 (0~20000mA) 使能→切模式→写目标\n");
    printf("  speed <id> <rpm*100> [acc*100] [dec*100]  速度控制 使能→PV→设加减速→写目标\n");
    printf("  abs <id> <deg*100>         位置控制 使能→PP→设参数→目标→启动\n");
    printf("  abs_stop <id>              停止位置运动\n");
    printf("  abs_accel <acc*100>        位置加减速 RPM/s×100 (默认2000)\n");
    printf("  abs_speed <rpm*100>        位置轨迹速度 RPM×100 输出端(默认10)\n\n");

    printf("── 配置命令 ──────────────────────────────────────\n");
    printf("  setzero <id>               零位标定 (自动失能)\n");
    printf("  limit_pos <id> <deg*100>   正限位 (失能→写→Flash)\n");
    printf("  limit_neg <id> <deg*100>   负限位 (失能→写→Flash)\n");
    printf("  save <id>                  保存参数到 Flash\n");
    printf("  pid <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置PID\n\n");

    printf("── 调试命令 ──────────────────────────────────────\n");
    printf("  sdoread <id> <0xIndex> [subidx]   通用 SDO 读\n");
    printf("  sdowrite <id> <0xIndex> <sub> <val> <size>  通用 SDO 写\n\n");

    printf("── 读取命令 ──────────────────────────────────────\n");
    printf("  read angle <id>      角度\n");
    printf("  read speed <id>      速度\n");
    printf("  read current <id>    电流 (mA)\n");
    printf("  read temp <id>       温度 (×0.1°C)\n");
    printf("  read state <id>      电机状态\n");
    printf("  read error <id>      故障码\n");
    printf("  read version <id>    固件版本\n");
    printf("  read mode <id>       运行模式\n");
    printf("  read pid <id>        PID参数\n");
    printf("  read all <id>        全部信息\n");
    printf("  read_limit_pos <id>  读正限位\n");
    printf("  read_limit_neg <id>  读负限位\n\n");

    printf("── 持续监控 ─────────────────────────────────────\n");
    printf("  watch <period_ms>    持续轮询反馈 (Ctrl+C 退出)\n");
    printf("  sensor watch <id>    传感器数据持续显示\n\n");

    printf("── 示例 ─────────────────────────────────────────\n");
    printf("  motor_tool daemon can0 &\n");
    printf("  motor_tool startup 1\n");
    printf("  motor_tool torque 1 500          # 电机1: 500mA\n");
    printf("  motor_tool speed 1 5000 100000   # 电机1: 50RPM 加速度1000RPM/s\n");
    printf("  motor_tool abs 1 4500            # 电机1: 45°\n");
    printf("  motor_tool abs_stop 1            # 电机1: 停止位置运动\n");
    printf("  motor_tool read all 0            # 读双电机全部信息\n");
    printf("  motor_tool watch 200             # 200ms持续监控\n");
    printf("  motor_tool stop\n\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        print_usage();
        return 0;
    }

    if (strcmp(argv[1], "daemon") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Usage: motor_tool daemon <can_iface>\n");
            return 1;
        }
        return daemon_start(argv[2]);
    }

    if (strcmp(argv[1], "help") == 0) {
        print_help();
        return 0;
    }

    if (strcmp(argv[1], "stop") == 0) {
        /* 停止 daemon (本地处理: 通过 socket 发 stop 命令) */
        return daemon_stop();
    }

    /* sensor watch → 持续读模式 (长连接, 不断开) */
    if (strcmp(argv[1], "sensor") == 0 && argc >= 4 && strcmp(argv[2], "watch") == 0) {
        return client_sensor_watch(argc, argv);
    }

    /* 其他命令 → 客户端模式 */
    return client_send(argc, argv);
}
