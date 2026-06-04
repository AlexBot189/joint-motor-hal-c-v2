/**
 * @file cmd_help.c
 * @brief help 命令
 */

#include "command_registry.h"
#include <stdio.h>

int cmd_do_help(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc; (void)argv;

    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║  motor_tool — CANFD CANopen 电机控制工具        ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");
    printf("用法: motor_tool <command> [args...]\n\n");

    printf("── 系统命令 ──────────────────────────────────────\n");
    printf("  init <can>          初始化 CANFD 接口\n");
    printf("  startup <id>        上电启动 (0=双电机)\n");
    printf("  enable <id>         使能电机\n");
    printf("  disable <id>        脱使能\n");
    printf("  reset <id>          故障复位\n\n");

    printf("── 控制命令 — ×100 精度 ─────────────────────────\n");
    printf("  speed <id> <rpm*100>      设置速度 (例: 5050→50.50RPM)\n");
    printf("  accel <id> <acc*100>      设置加减速 (例: 3000→30RPM/s)\n");
    printf("  abs <id> <deg*100>        绝对位置 (例: 2300→23.00°)\n");
    printf("  rel <id> <delta*100>      相对位置 (先读再设)\n");
    printf("  maxv <id> <rpm*100>       位置模式最大轨迹速度\n");
    printf("  stop [id]                 停止 (默认 id=0)\n");
    printf("  mode <id> <pp|pv|csp|csv|cur>  切换模式\n");
    printf("  torque <id> <mA>          电流/力矩控制 (例: 2000→2A)\n");
    printf("  csp <id> <deg*100>        CSP同步位置控制\n");
    printf("  mit <id> <pos> <vel> <kp> <kd> <t>  MIT阻抗控制\n");
    printf("  brake <id> <release|lock> 抱闸控制\n");
    printf("  quickstop <id>            急停 (DS402 Quick Stop)\n\n");

    printf("── 配置命令 ──────────────────────────────────────\n");
    printf("  save <id>                 保存参数到 Flash (掉电不丢失)\n");
    printf("  setzero <id>              零位标定 (当前位置=0)\n");
    printf("  pid <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置PID\n\n");

    printf("── 调试命令 ──────────────────────────────────────\n");
    printf("  sdoread <id> <0xIndex> [subidx]   通用 SDO 读\n");
    printf("  sdowrite <id> <0xIndex> <sub> <val> <size>  通用 SDO 写\n\n");

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
    printf("  motor_tool init can0\n");
    printf("  motor_tool startup 0              # 双电机上电\n");
    printf("  motor_tool speed 1 5000           # 电机1: 50RPM\n");
    printf("  motor_tool speed 0 10000          # 双电机: 100RPM\n");
    printf("  motor_tool abs 2 4500             # 电机2: 绝对45°\n");
    printf("  motor_tool rel 1 1000             # 电机1: 相对+10°\n");
    printf("  motor_tool torque 1 2000          # 电机1: 2A力矩\n");
    printf("  motor_tool csp 1 3000             # CSP: 30°\n");
    printf("  motor_tool brake 1 release        # 松开抱闸\n");
    printf("  motor_tool quickstop 1            # 急停\n");
    printf("  motor_tool sdoread 1 0x100A       # 读固件版本\n");
    printf("  motor_tool save 1                 # 保存到Flash\n");
    printf("  motor_tool read all 0             # 读双电机全部\n");
    printf("  motor_tool watch 200              # 200ms持续监控\n");
    printf("  motor_tool stop\n\n");

    return 0;
}
