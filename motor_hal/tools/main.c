/**
 * @file main.c
 * @brief motor_tool — CANFD CANopen 电机控制工具入口
 *
 * 模式:
 *   motor_tool daemon can0   ,  启动守护进程
 *   motor_tool stop           ,  停止守护进程
 *   motor_tool help           ,  显示帮助 (本地)
 *   motor_tool <cmd> [args]  ,  通过 socket 向 daemon 发命令
 *   motor_tool                ,  显示用法
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

    printf("── 校准 & 传感器透传 控制前必须完成 ──────────\n");
    printf("  calib start <id> [id_l] [t]  单/双电机校准 (0=跳过)\n");
    printf("  calib status          查看校准进度\n");
    printf("  calib exit            退出校准\n");
    printf("  setzero <id>          设当前位置为零位\n");
    printf("  sensor config <id> <period>    开启传感器透传 (例: 250Hz)\n");
    printf("  sensor read <id>      读传感器缓存\n");
    printf("  sensor watch <id>     传感器数据持续显示\n");
    printf("  sensor stop <id>      停止透传\n\n");

    printf("── 控制命令 (自动使能) ────────────────────\n");
    printf("  sdo cur <N> <mA>              SDO 电流控制\n");
    printf("  sdo cur <N1> <N2> <mA>        双电机电流(同值)\n");
    printf("  sdo cur <N1> <N2> <mA1> <mA2> 双电机电流(异值)\n");
    printf("  sdo pos <N> <deg>             SDO 位置控制 (PP)\n");
    printf("  sdo csp <N> <deg>             SDO 位置控制 (CSP)\n");
    printf("  sdo vel <N> <rpm>             SDO 速度控制 (PV)\n");
    printf("  sdo csv <N> <rpm>             SDO 速度控制 (CSV)\n");
    printf("  双电机: sdo pos <N1> <N2> <deg>      同角度\n");
    printf("  双电机: sdo pos <N1> <N2> <deg1> <deg2> 不同角度\n");
    printf("  pdo cur|pos|csp|vel|csv        参数同 SDO (多轴广播, 不发 SDO)\n\n");

    printf("── 配置命令 ──────────────────────────────────────\n");
    printf("  limit_pos <id> <deg>       正限位 (失能, 写, Flash)\n");
    printf("  limit_neg <id> <deg>       负限位 (失能, 写, Flash)\n");
    printf("  save <id>                  保存参数到 Flash\n");
    printf("  pid <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置PID\n\n");

    printf("── 调试命令 ──────────────────────────────────────\n");
    printf("  sdoread <id> <0xIndex> [subidx]   通用 SDO 读\n");
    printf("  sdowrite <id> <0xIndex> <sub> <val> <size>  通用 SDO 写\n\n");

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
    printf("  read all <id>        全部信息\n\n");

    printf("── 持续监控 ─────────────────────────────────────\n");
    printf("  watch <period_ms>    持续轮询反馈 (Ctrl+C 退出)\n");
    printf("  sensor watch <id>    传感器数据持续显示\n\n");

    printf("── 框架工作流 ────────────────────────────────────\n");
    printf("  ① motor_tool daemon can0 &        # 启动守护进程 (auto-startup + 零位校准)\n");
    printf("  ② motor_tool sdo cur 1 500        # SDO 电流控制电机1 (自动使能)\n");
    printf("  ③ motor_tool sdo pos 1 45         # SDO 位置控制 (PP, 转到45°)\n");
    printf("  ④ motor_tool pdo cur 1 2 500      # 双电机 PDO 电流控制\n");
    printf("  ⑤ motor_tool pdo vel 1 2 30       # 双电机 PDO 速度控制 (PV)\n");
    printf("  ⑥ motor_tool disable 1            # 脱使能电机1\n");
    printf("     motor_tool stop                 # 停止 daemon\n\n");
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

    /* sensor watch ,  持续读模式 (长连接, 不断开) */
    if (strcmp(argv[1], "sensor") == 0 && argc >= 4 && strcmp(argv[2], "watch") == 0) {
        return client_sensor_watch(argc, argv);
    }

    /* 其他命令 ,  客户端模式 */
    return client_send(argc, argv);
}
