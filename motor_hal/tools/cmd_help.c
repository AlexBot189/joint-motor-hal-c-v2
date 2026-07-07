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
    printf("╚══════════════════════════════════════════════════════════╝\n\n");
    printf("用法: motor_tool <command> [args...]\n");
    printf("      id=0 ,  广播到全部已注册电机\n\n");

    printf("┌─ 生命周期 (daemon 模式) ────────────────────────────────┐\n");
    printf("│ daemon <can>          启动守护进程 (自动检测+零位校准)       │\n");
    printf("│ probe [id]            探测电机在线 (ID列表)               │\n");
    printf("│ startup <id>          上电启动 (0=所有已注册电机)          │\n");
    printf("│ enable <id>           使能 (DS402: Shutdown→SwitchOn→EnableOp) │\n");
    printf("│ disable <id>          脱使能                              │\n");
    printf("│ fault_reset <id>      清零故障                            │\n");
    printf("│ reboot <id>           电机重启                            │\n");
    printf("│ setzero <id>          设当前位置为零位                     │\n");
    printf("│ save <id>             参数保存到 Flash                    │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 控制命令 (自动使能, 无需手动 enable) ──────────────────┐\n");
    printf("│ SDO 控制 (CiA 402 标准):                                  │\n");
    printf("│   sdo cur <N> <mA>              单电机电流                  │\n");
    printf("│   sdo cur <N1> <N2> <mA>        双电机电流(同值)            │\n");
    printf("│   sdo cur <N1> <N2> <mA1> <mA2> 双电机电流(异值)            │\n");
    printf("│   sdo pos <N> <deg>             单电机位置(PP, 轮廓位置)      │\n");
    printf("│   sdo pos <N1> <N2> <deg>       双电机位置(PP, 同角度)        │\n");
    printf("│   sdo pos <N1> <N2> <deg1> <deg2> 双电机位置(PP, 不同角度)    │\n");
    printf("│   sdo csp <N> <deg>             单电机位置(CSP)             │\n");
    printf("│   sdo csp <N1> <N2> <deg>       双电机位置(CSP)             │\n");
    printf("│   sdo vel <N> <rpm>             单电机速度(PV)              │\n");
    printf("│   sdo vel <N1> <N2> <rpm>       双电机速度(PV)              │\n");
    printf("│   sdo csv <N> <rpm>             单电机速度(CSV)              │\n");
    printf("│ PDO 控制 (多轴广播 COB 0x200, 不触发 SDO):                  │\n");
    printf("│   pdo cur/pos/csp/vel/csv 参数同 SDO 用法                    │\n");
    printf("│ PDO 多轴广播 (id:val 格式):                                │\n");
    printf("│   multi <mode> <id:val> ...     例: multi cur 1:500 2:300 │\n");
    printf("│ MIT 阻抗控制:                                              │\n");
    printf("│   mit <id> <pos> <vel> <kp> <kd> <torque>                  │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 读取 ──────────────────────────────────────────────────┐\n");
    printf("│ read angle <id>        角度 (°)                          │\n");
    printf("│ read speed <id>        速度 RPM                          │\n");
    printf("│ read current <id>      Q轴电流 mA                        │\n");
    printf("│ read temp <id>         电机温度 °C                       │\n");
    printf("│ read mode <id>         运行模式                          │\n");
    printf("│ read state <id>        电机状态                          │\n");
    printf("│ read error <id>        故障码                            │\n");
    printf("│ read version <id>      固件版本                          │\n");
    printf("│ read pid <id>          PID参数                           │\n");
    printf("│ read all <id>          全部信息                           │\n");
    printf("│ sensor read <id>       霍尔/力矩/膝关节                   │\n");
    printf("│ sdoread  <id> <idx> [sub]  通用 SDO 读                   │\n");
    printf("│ sdowrite <id> <idx> <sub> <val> <size>  通用 SDO 写      │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 数据流 & 校准 & 传感器 ───────────────────────────────┐\n");
    printf("│ 持续监控:                                                │\n");
    printf("│   watch [period_ms]      持续轮询反馈 (Ctrl+C 退出)       │\n");
    printf("│   report [period_ms]     数据上报  例: report 5           │\n");
    printf("│ 校准:                                                    │\n");
    printf("│   calib start <id_r> <id_l> [t]  校准流程                │\n");
    printf("│   calib status                  查看校准状态              │\n");
    printf("│   calib exit                    退出校准                  │\n");
    printf("│ 传感器透传:                                              │\n");
    printf("│   sensor config <id> <period>    开启 (例: sensor config 1 250) │\n");
    printf("│   sensor read <id>               读传感器缓存             │\n");
    printf("│   sensor watch <id>              实时看板 (Ctrl+C 停)     │\n");
    printf("│   sensor stop <id>               停止透传                 │\n");
    printf("│ 配置:                                                    │\n");
    printf("│   limit_pos <id> <deg>          正限位                    │\n");
    printf("│   limit_neg <id> <deg>          负限位                    │\n");
    printf("│   pid <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置PID        │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    printf("┌─ 框架工作流 ────────────────────────────────────────────┐\n");
    printf("│  ① motor_tool daemon can0 &        # 启动守护进程          │\n");
    printf("│      ,  自动检测电机, startup, 零位校准, 电机就绪           │\n");
    printf("│  ② motor_tool sdo cur 1 500        # SDO 电流控制 (自动使能)│\n");
    printf("│  ③ motor_tool sdo pos 1 45         # SDO 位置控制 (PP)       │\n");
    printf("│  ④ motor_tool pdo cur 1 2 500      # 双电机 PDO 电流(同值)    │\n");
    printf("│  ⑤ motor_tool pdo pos 1 2 40       # 双电机 PDO 位置(同角度)   │\n");
    printf("│     motor_tool pdo vel 1 2 50 80    # 双电机 PDO 速度(异值)     │\n");
    printf("│  ⑥ motor_tool disable 1            # 脱使能电机1              │\n");
    printf("│     motor_tool stop                 # 停止 daemon           │\n");
    printf("└──────────────────────────────────────────────────────────┘\n\n");

    return 0;
}

/* 裸运行 motor_tool 时显示快捷用法 */
void print_quick_usage(void)
{
    printf("\n╔══════════════════════════════════════════════════════════╗\n");
    printf("║          motor_tool — CANFD 电机控制工具 v2              ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    printf("── SDO 控制 (自动使能) ──────────────────────────────\n");
    printf("  sdo cur <N> <mA>              例: sdo cur 1 500\n");
    printf("  sdo cur <N1> <N2> <mA>        例: sdo cur 1 2 500\n");
    printf("  sdo cur <N1> <N2> <mA1> <mA2> 例: sdo cur 1 2 500 300\n");
    printf("  sdo pos <N> <deg>             例: sdo pos 1 45\n");
    printf("  sdo pos <N1> <N2> <deg>       例: sdo pos 1 2 45\n");
    printf("  sdo pos <N1> <N2> <deg1> <deg2> 例: sdo pos 1 2 45 90\n");
    printf("  sdo csp <N> <deg>             例: sdo csp 1 30\n");
    printf("  sdo csp <N1> <N2> <deg>       例: sdo csp 1 2 30\n");
    printf("  sdo csp <N1> <N2> <deg1> <deg2> 例: sdo csp 1 2 30 60\n");
    printf("  sdo vel <N> <rpm>             例: sdo vel 1 100\n");
    printf("  sdo vel <N1> <N2> <rpm>       例: sdo vel 1 2 100\n");
    printf("  sdo vel <N1> <N2> <rpm1> <rpm2> 例: sdo vel 1 2 100 50\n");
    printf("  sdo csv <N> <rpm>             例: sdo csv 1 100\n");
    printf("  sdo csv <N1> <N2> <rpm>       例: sdo csv 1 2 100\n");
    printf("  sdo csv <N1> <N2> <rpm1> <rpm2> 例: sdo csv 1 2 100 50\n\n");

    printf("── PDO 控制 (多轴广播, 不发 SDO) ──────────────────\n");
    printf("  pdo cur <N> <mA>              例: pdo cur 1 500\n");
    printf("  pdo cur <N1> <N2> <mA>        例: pdo cur 1 2 500\n");
    printf("  pdo cur <N1> <N2> <mA1> <mA2> 例: pdo cur 1 2 500 300\n");
    printf("  pdo pos <N> <deg>             例: pdo pos 1 45\n");
    printf("  pdo pos <N1> <N2> <deg>       例: pdo pos 1 2 45\n");
    printf("  pdo pos <N1> <N2> <deg1> <deg2> 例: pdo pos 1 2 45 90\n");
    printf("  pdo csp <N> <deg>             例: pdo csp 1 30\n");
    printf("  pdo csp <N1> <N2> <deg>       例: pdo csp 1 2 30\n");
    printf("  pdo csp <N1> <N2> <deg1> <deg2> 例: pdo csp 1 2 30 60\n");
    printf("  pdo vel <N> <rpm>             例: pdo vel 1 100\n");
    printf("  pdo vel <N1> <N2> <rpm>       例: pdo vel 1 2 100\n");
    printf("  pdo vel <N1> <N2> <rpm1> <rpm2> 例: pdo vel 1 2 100 50\n");
    printf("  pdo csv <N> <rpm>             例: pdo csv 1 100\n");
    printf("  pdo csv <N1> <N2> <rpm>       例: pdo csv 1 2 100\n");
    printf("  pdo csv <N1> <N2> <rpm1> <rpm2> 例: pdo csv 1 2 100 50\n\n");

    printf("── 其他命令 ─────────────────────────────────────\n");
    printf("  生命周期: daemon probe startup enable disable fault_reset reboot\n");
    printf("            setzero save\n");
    printf("  校准透传: calib sensor\n");
    printf("  读取:     read <item> sensor read sdoread sdowrite\n");
    printf("  数据流:   watch report\n");
    printf("  配置:     limit_pos limit_neg pid\n");
    printf("  多轴:     multi mit\n");
    printf("──────────────────────────────────────────────────\n");
    printf("  快速启动:\n");
    printf("    motor_tool daemon can0 &        # 启动守护进程\n");
    printf("    motor_tool sdo cur 1 500        # SDO 电流控制\n");
    printf("    motor_tool sdo pos 1 45         # SDO 位置控制 (PP)\n");
    printf("    motor_tool pdo cur 1 2 500      # 双电机 PDO 电流(同值)\n");
    printf("    motor_tool pdo pos 1 2 40       # 双电机 PDO 位置(同角度)\n");
    printf("  完整帮助: motor_tool help\n\n");
}
