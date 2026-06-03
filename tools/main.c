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

static void print_usage(void)
{
    printf("motor_tool — CANFD CANopen 电机控制工具\n\n");
    printf("用法:\n");
    printf("  motor_tool daemon can0        启动守护进程\n");
    printf("  motor_tool stop               停止守护进程\n");
    printf("  motor_tool help               显示命令列表\n");
    printf("  motor_tool <command> [args]   发送控制/读取命令\n\n");
    printf("启动流程:\n");
    printf("  1. motor_tool daemon can0 &    # 后台启动\n");
    printf("  2. motor_tool speed 1 5000     # 控制电机1 (RPM×100)\n");
    printf("  3. motor_tool abs 2 4500       # 电机2 绝对位置45°\n");
    printf("  4. motor_tool read all 0       # 读双电机全部状态\n");
    printf("  5. motor_tool watch 200        # 持续监控\n");
    printf("  6. motor_tool disable 0        # 脱使能\n");
    printf("  7. motor_tool stop             # 停止 daemon\n");
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

    printf("── 控制命令 — ×100 精度 ─────────────────────────\n");
    printf("  speed <id> <rpm*100>       设置速度 (例: 5050→50.50RPM)\n");
    printf("  accel <id> <acc*100>       设置加减速 (例: 3000→30RPM/s)\n");
    printf("  abs <id> <deg*100>         绝对位置 (例: 2300→23.00°)\n");
    printf("  rel <id> <delta*100>       相对位置 (先读再设)\n");
    printf("  maxv <id> <rpm*100>        位置模式最大轨迹速度\n");
    printf("  stop [id]                  停止电机 (默认 id=0)\n");
    printf("  mode <id> <pp|pv|csp|csv|cur>  切换模式\n\n");

    printf("── 读取命令 — 输出原始值 ────────────────────────\n");
    printf("  read angle <id>      角度 (编码器counts)\n");
    printf("  read speed <id>      速度 (RPM)\n");
    printf("  read current <id>    Q轴电流 (mA)\n");
    printf("  read temp <id>       温度 (raw, ×0.1°C)\n");
    printf("  read state <id>      电机状态\n");
    printf("  read error <id>      故障码\n");
    printf("  read version <id>    固件版本\n");
    printf("  read all <id>        全部信息\n\n");

    printf("── 持续监控 ─────────────────────────────────────\n");
    printf("  watch <period_ms>    持续轮询显示 (Ctrl+C 退出)\n\n");

    printf("── 示例 ─────────────────────────────────────────\n");
    printf("  motor_tool daemon can0 &\n");
    printf("  motor_tool speed 1 5000         # 电机1: 50RPM\n");
    printf("  motor_tool speed 0 10000        # 双电机: 100RPM\n");
    printf("  motor_tool abs 2 4500           # 电机2: 45°\n");
    printf("  motor_tool rel 1 1000           # 电机1: 相对+10°\n");
    printf("  motor_tool read all 0           # 读双电机\n");
    printf("  motor_tool watch 200            # 200ms持续监控\n");
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

    /* 其他命令 → 客户端模式 */
    return client_send(argc, argv);
}
