/**
 * @file cmd_help.c
 * @brief help 命令 + 裸运行显示用法
 */

#include "command_registry.h"
#include <stdio.h>

int cmd_do_help(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc; (void)argv;

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║          motor_tool — CANFD 电机控制工具 v2              ║\n");
    printf("║          对照 GD32 ODS 协议 (V-prefix/C-prefix)          ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");
    printf("用法: motor_tool <command> [args...]\n");
    printf("      id=0 → 广播到全部已注册电机\n\n");

    printf("┌─ 生命周期 ──────────────────────────────────────────────┐\n");
    printf("│ init <can>            初始化 CANFD 接口                   │\n");
    printf("│ startup <id>          上电启动 (Bootup→关狗→使能)         │\n");
    printf("│ enable <id>           使能 (DS402: 0x06→0x07→0x0F)       │\n");
    printf("│ disable <id>          脱使能 [V-F]                        │\n");
    printf("│ fault_reset <id>      清零故障 [V-A]                      │\n");
    printf("│ reboot <id>           电机重启 [V-E]                      │\n");
    printf("│ setzero <id>          设当前位置为零位 [V-C]              │\n");
    printf("│ save <id>             参数保存到 Flash                    │\n");
    printf("│ calib start <id_r> <id_l> [t]  校准流程 [V-D]            │\n");
    printf("│ calib status          查看校准状态                        │\n");
    printf("│ calib exit            退出校准                            │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 控制 (×100精度, id=0=广播) ────────────────────────────┐\n");
    printf("│ torque  <id> <mA>         Q轴电流 [V-G]   例: 2000→2A    │\n");
    printf("│ speed   <id> <rpm*100>    目标速度 [V-I]  例: 5050→50.50 │\n");
    printf("│ accel   <id> <acc*100>    加减速度 [V-J]                 │\n");
    printf("│ abs     <id> <deg*100>    绝对位置 [V-K]  例: 4500→45°   │\n");
    printf("│ rel     <id> <delta*100>  相对位置 [V-L]                 │\n");
    printf("│ maxv    <id> <rpm*100>    最大速度 [V-U]                 │\n");
    printf("│ stop    [id]              停止                            │\n");
    printf("│ mode    <id> <pp|pv|csp|csv|cur>  切换模式                │\n");
    printf("│ csp     <id> <deg*100>    CSP 同步位置                    │\n");
    printf("│ mit  <id> <p> <v> <kp> <kd> <t>  MIT 阻抗控制            │\n");
    printf("│ brake   <id> <rel|lock>  抱闸控制                         │\n");
    printf("│ quickstop <id>           急停 (DS402)                     │\n");
    printf("│ pid <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置 PID          │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 读取 ──────────────────────────────────────────────────┐\n");
    printf("│ read angle <id>        角度 (V-a)                        │\n");
    printf("│ read speed <id>        速度 RPM (V-b)                    │\n");
    printf("│ read current <id>      Q轴电流 mA (V-c)                  │\n");
    printf("│ read voltage <id>      母线电压 (V-d)                    │\n");
    printf("│ read bus_current <id>  母线电流 (V-e)                    │\n");
    printf("│ read temp <id>         电机温度 °C (V-f)                 │\n");
    printf("│ read mode <id>         运行模式 (V-g)                    │\n");
    printf("│ read state <id>        电机状态 (V-h)                    │\n");
    printf("│ read error <id>        故障码 (V-I)                      │\n");
    printf("│ read version <id>      固件版本 (V-j)                    │\n");
    printf("│ read all <id>          全部信息                           │\n");
    printf("│ sensor read <id>       霍尔/力矩/膝关节 (V-n)            │\n");
    printf("│ sdoread  <id> <idx> [s] 通用 SDO 读                      │\n");
    printf("│ sdowrite <id> <idx> <s> <v> <sz>  通用 SDO 写            │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 数据流 ────────────────────────────────────────────────┐\n");
    printf("│ report [period_ms]     CA 数据上报  例: report 5          │\n");
    printf("│   → 每 period_ms 输出 feedback + sensor (Ctrl+C 停止)     │\n");
    printf("│   → report 0 停止上报                                    │\n");
    printf("│                                                          │\n");
    printf("│ watch [period_ms]      持续监控 (精简版)                  │\n");
    printf("│                                                          │\n");
    printf("│ sensor config <id> <p> <fmt>  传感器透传配置              │\n");
    printf("│ sensor stop <id>              停止透传                    │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 典型工作流 ────────────────────────────────────────────┐\n");
    printf("│ $ motor_tool init can0                                    │\n");
    printf("│ $ motor_tool startup 1                                    │\n");
    printf("│ $ motor_tool enable 1                                     │\n");
    printf("│ $ motor_tool calib start 1 2   # 校准左右电机             │\n");
    printf("│ $ motor_tool torque 1 2000     # 2A 力矩                  │\n");
    printf("│ $ motor_tool report 5          # 5ms 数据流查看           │\n");
    printf("│ $ motor_tool disable 1                                    │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    return 0;
}

/* 裸运行 motor_tool 时显示快捷用法 */
void print_quick_usage(void)
{
    printf("\n╔══════════════════════════════════════════════════════════╗\n");
    printf("║          motor_tool — CANFD 电机控制工具 v2              ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");
    printf("快捷命令对照 (完整列表: motor_tool help)\n");
    printf("──────────────────────────────────────────────────────────\n");
    printf("  生命周期: init startup enable disable fault_reset reboot\n");
    printf("            setzero save calib\n");
    printf("  控制:     torque speed abs rel accel maxv stop mode\n");
    printf("            csp mit brake quickstop pid\n");
    printf("  读取:     read <item> sensor read sdoread\n");
    printf("  数据流:   report watch\n");
    printf("──────────────────────────────────────────────────────────\n");
    printf("  示例:\n");
    printf("    motor_tool init can0\n");
    printf("    motor_tool startup 1 && motor_tool enable 1\n");
    printf("    motor_tool torque 1 2000     # 2A 电流控制\n");
    printf("    motor_tool speed 1 5000      # 50RPM 速度控制\n");
    printf("    motor_tool abs 1 4500        # 45° 绝对位置\n");
    printf("    motor_tool read temp 1       # 读电机温度\n");
    printf("    motor_tool report 5          # 每5ms数据上报 (Ctrl+C停)\n");
    printf("    motor_tool watch 200         # 200ms持续监控\n\n");
}
