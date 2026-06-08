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
    printf("  startup <id>         上电启动 (0=所有已注册电机)\n");
    printf("  enable <id>          使能电机\n");
    printf("  disable <id>         脱使能\n");
    printf("  reset <id>           故障复位\n");
    printf("  reboot <id>          电机重启\n");
    printf("  probe [id]           主动探测电机在线 (0=全部)\n\n");

    printf("── 校准 & 传感器透传 ★ 控制前必须完成 ★ ──────────\n");
    printf("  calib start <id_r> <id_l> [t]  校准流程 (设零位→验证)\n");
    printf("  calib status          查看校准进度\n");
    printf("  calib exit            退出校准\n");
    printf("  setzero <id>          设当前位置为零位\n");
    printf("  sensor config <id> <period>    开启传感器透传 (例: 250Hz)\n");
    printf("  sensor read <id>      读传感器缓存\n");
    printf("  sensor watch <id>     传感器数据持续显示\n");
    printf("  sensor stop <id>      停止透传\n\n");

    printf("── 控制命令 (SDO, 使能在 daemon 启动时完成) ────\n");
    printf("  torque <id> <mA>           电流控制 (0~20000mA)\n");
    printf("  speed <id> <rpm*100> [acc*100] [dec*100]  速度控制\n");
    printf("  abs <id> <deg*100>         位置控制\n");
    printf("  abs_stop <id>              停止位置运动\n");
    printf("  abs_accel <acc*100>        位置加减速 RPM/s×100 (默认2000)\n");
    printf("  abs_speed <rpm*100>        位置轨迹速度 RPM×100 输出端(默认10)\n\n");

    printf("── PDO 实时控制 (直接发帧, <100μs) ────────────\n");
    printf("  pdo <id> pos <deg*100>     单轴位置控制 PDO\n");
    printf("  pdo <id> vel <rpm*100>     单轴速度控制 PDO\n");
    printf("  pdo <id> cur <mA>           单轴电流控制 PDO\n");
    printf("  pdo <id> csp <cnt>          单轴 CSP 控制 PDO\n");
    printf("  multi pos 1:4500 2:-4500   多轴广播位置\n");
    printf("  multi vel 1:5000 2:-3000   多轴广播速度\n");
    printf("  multi cur 1:1000 2:500     多轴广播电流\n");
    printf("  mit <id> <pos> <vel> <kp> <kd> <torque>  MIT 阻抗\n\n");

    printf("── 配置命令 ──────────────────────────────────────\n");
    printf("  limit_pos <id> <deg*100>   正限位 (失能→写→Flash)\n");
    printf("  limit_neg <id> <deg*100>   负限位 (失能→写→Flash)\n");
    printf("  save <id>                  保存参数到 Flash\n");
    printf("  pid <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置PID\n\n");

    printf("── 调试命令 ──────────────────────────────────────\n");
    printf("  sdoread <id> <0xIndex> [subidx]   通用 SDO 读\n");
    printf("  sdowrite <id> <0xIndex> <sub> <val> <size>  通用 SDO 写\n");
    printf("  tpdo_map <id> <cob> <ttype> <idx> <sub> <bits> ...   TPDO 映射\n");
    printf("  rpdo_map <id> <cob> <ttype> <idx> <sub> <bits> ...   RPDO 映射\n");
    printf("  tpdo <id> <sync> <item> ...                TPDO 快捷映射\n");
    printf("  rpdo <id> <ttype> <item> ...               RPDO 快捷映射\n");
    printf("  rpdo_send <id> <hex_bytes...>             RPDO 发送\n\n");

    printf("── 读取命令 ──────────────────────────────────────\n");
    printf("  read angle <id>      角度\n");
    printf("  read speed <id>      速度 RPM\n");
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

    printf("── 框架工作流 ────────────────────────────────────\n");
    printf("  ① motor_tool daemon can0 &        # 启动守护进程\n");
    printf("  ② [电机上电 → 发 0x701 bootup]     # 物理操作\n");
    printf("  ③ # daemon 自动收到 bootup → auto-startup → 使能\n");
    printf("  ④ motor_tool calib start 1 2      # 校准\n");
    printf("     motor_tool sensor config 1 250  # 开启透传\n");
    printf("  ⑤ motor_tool torque 1 500         # 控制!\n");
    printf("     motor_tool state 1              # 查看状态\n");
    printf("     motor_tool stop                 # 停止\n\n");
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
