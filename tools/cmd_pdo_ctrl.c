/**
 * @file cmd_pdo_ctrl.c
 * @brief PDO 实时控制命令: pdo / multi / mit
 *
 * 这些命令通过自定义 PDO 帧直接控制电机, 不经过 SDO, <100μs 延迟。
 * 不需要映射, 直接发帧。
 *
 * 用法:
 *   motor_tool pdo <id> pos <deg> [acc]            # 单轴位置
 *   motor_tool pdo <id> vel <rpm> [acc]            # 单轴速度
 *   motor_tool pdo <id> cur <mA>                       # 单轴电流
 *   motor_tool pdo <id> csp <cnt>                      # 单轴 CSP
 *
 *   motor_tool multi pos 1:45 2:-45                 # 多轴位置
 *   motor_tool multi vel 1:50 2:-30                 # 多轴速度
 *   motor_tool multi cur 1:1000 2:500                  # 多轴电流 (mA)
 *   motor_tool multi csp 1:16384 2:-16384             # 多轴 CSP
 *
 *   motor_tool mit 1 <pos> <vel> <kp> <kd> <torque>   # MIT 阻抗控制
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ================================================================
 * 解析 "id:value" 格式, 如 "1:45"
 * ================================================================ */

static int _parse_id_val(const char *arg, int *id, int *val)
{
    char copy[32];
    strncpy(copy, arg, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *colon = strchr(copy, ':');
    if (!colon) return -1;
    *colon = '\0';
    *id  = atoi(copy);
    *val = atoi(colon + 1);
    return 0;
}

/* ================================================================
 * pdo <id> <mode> <target> [acc]
 *   pos=PP, vel=PV, cur=电流, csp=CSP
 * ================================================================ */

int cmd_do_pdo(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 4) {
        fprintf(stderr, "Usage: motor_tool pdo <id> <mode> <target> [acc_or_feedfwd]\n");
        fprintf(stderr, "  mode: pos / vel / cur / csp\n");
        fprintf(stderr, "  pos: <deg>   vel: <rpm>   cur: <mA>   csp: <cnt>\n");
        fprintf(stderr, "  [acc]: profile accel RPM/s (pos/vel mode only)\n");
        fprintf(stderr, "Examples:\n");
        fprintf(stderr, "  motor_tool pdo 1 pos 45           # 电机1: 45°\n");
        fprintf(stderr, "  motor_tool pdo 1 vel 50 1000      # 电机1: 50RPM acc=1000RPM/s\n");
        fprintf(stderr, "  motor_tool pdo 1 cur 1000         # 电机1: 1000mA\n");
        fprintf(stderr, "  motor_tool pdo 1 csp 16384        # 电机1: CSP 16384cnt\n");
        return -1;
    }

    int id = atoi(argv[2]);
    const char *mode_str = argv[3];

    if (strcmp(mode_str, "cur") == 0) {
        int ma = atoi(argv[4]);
        int ret = motor_hal_set_torque(g_hal, (uint8_t)id, (int16_t)ma);
        if (ret == 0) printf("✓ Motor %d: PDO current=%d mA\n", id, ma);
        else fprintf(stderr, "✗ Motor %d: PDO current failed (ret=%d)\n", id, ret);
        return ret;

    } else if (strcmp(mode_str, "vel") == 0) {
        float rpm = (float)atof(argv[4]);
        int ret = motor_hal_set_velocity(g_hal, (uint8_t)id, rpm);
        if (ret == 0) printf("✓ Motor %d: PDO vel=%.2f RPM\n", id, rpm);
        else fprintf(stderr, "✗ Motor %d: PDO vel failed (ret=%d)\n", id, ret);
        return ret;

    } else if (strcmp(mode_str, "pos") == 0) {
        float deg = (float)atof(argv[4]);
        int ret = motor_hal_set_position(g_hal, (uint8_t)id, deg);
        if (ret == 0) printf("✓ Motor %d: PDO pos=%.2f°\n", id, deg);
        else fprintf(stderr, "✗ Motor %d: PDO pos failed (ret=%d)\n", id, ret);
        return ret;

    } else if (strcmp(mode_str, "csp") == 0) {
        int16_t cnt = (int16_t)atoi(argv[4]);
        int ret = motor_hal_ctrl_raw(g_hal, (uint8_t)id, MOTOR_MODE_CSP, cnt, 0, 0);
        if (ret == 0) printf("✓ Motor %d: PDO CSP=%d cnt\n", id, cnt);
        else fprintf(stderr, "✗ Motor %d: PDO CSP failed (ret=%d)\n", id, ret);
        return ret;
    }

    fprintf(stderr, "Unknown mode: %s (use: pos/vel/cur/csp)\n", mode_str);
    return -1;
}

/* ================================================================
 * multi <mode> <id1:val1> [id2:val2] ...
 *   一次下发最多8个电机, 用多轴广播帧 (0x200, 64B)
 * ================================================================ */

int cmd_do_multi(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 4) {
        fprintf(stderr, "Usage: motor_tool multi <mode> <id1:val1> [id2:val2] ...\n");
        fprintf(stderr, "  mode: pos / vel / cur / csp\n");
        fprintf(stderr, "  pos: <deg>   vel: <rpm>   cur: <mA>   csp: <cnt>\n");
        fprintf(stderr, "  Max 8 motors per command.\n");
        fprintf(stderr, "Examples:\n");
        fprintf(stderr, "  motor_tool multi pos 1:45 2:-45            # 双关节位置\n");
        fprintf(stderr, "  motor_tool multi vel 1:50 2:-30 3:20       # 三关节速度\n");
        fprintf(stderr, "  motor_tool multi cur 1:1000 2:500            # 双关节电流\n");
        return -1;
    }

    const char *mode_str = argv[2];
    int n = argc - 3;  /* 电机数量 */
    if (n > 8) { fprintf(stderr, "ERROR: max 8 motors (got %d)\n", n); return -1; }

    multi_axis_cmd_t cmds[8];
    memset(cmds, 0, sizeof(cmds));

    motor_mode_t pdo_mode = MOTOR_MODE_PROFILE_POS;
    if (strcmp(mode_str, "vel") == 0)      pdo_mode = MOTOR_MODE_PROFILE_VEL;
    else if (strcmp(mode_str, "cur") == 0) pdo_mode = MOTOR_MODE_CURRENT;
    else if (strcmp(mode_str, "csp") == 0) pdo_mode = MOTOR_MODE_CSP;

    for (int i = 0; i < n; i++) {
        int id, val;
        if (_parse_id_val(argv[3 + i], &id, &val) != 0) {
            fprintf(stderr, "ERROR: invalid arg '%s' (use id:val)\n", argv[3 + i]);
            return -1;
        }
        if (id < 1 || id > 8) {
            fprintf(stderr, "ERROR: id %d out of range (1~8)\n", id);
            return -1;
        }

        cmds[i].node_id       = (uint8_t)id;
        cmds[i].mode          = pdo_mode;
        cmds[i].enable        = true;
        cmds[i].release_brake = true;
        cmds[i].clear_error   = false;

        if (strcmp(mode_str, "pos") == 0) {
            int16_t cnt = motor_deg_to_counts((float)val);
            cmds[i].target1 = cnt;
        } else if (strcmp(mode_str, "vel") == 0) {
            cmds[i].target1 = (int16_t)val;
        } else if (strcmp(mode_str, "cur") == 0) {
            cmds[i].target1 = (int16_t)val;
        } else {
            cmds[i].target1 = (int16_t)val;  /* csp: 直接用 cnt */
        }
    }

    /* 打印 */
    printf("Multi-ctrl (%s): ", mode_str);
    for (int i = 0; i < n; i++) {
        printf("[id=%d val=%d] ", cmds[i].node_id, cmds[i].target1);
    }
    printf("\n");

    motor_hal_multi_ctrl(g_hal, cmds, (uint8_t)n);

    /* 读反馈显示 (从缓存, 不触 SDO) */
    usleep(20000);  /* 等 20ms 让反馈帧到达并被接收线程缓存 */
    for (int i = 0; i < n; i++) {
        motor_feedback_t fb;
        int ret = motor_hal_get_feedback(g_hal, cmds[i].node_id, &fb);
        if (ret == 0) {
            float angle = motor_counts_to_deg(fb.position);
            printf("  [id=%d] pos=%.1f° vel=%d RPM cur=%d mA temp=%.1f°C st=0x%02X\n",
                   cmds[i].node_id, angle, fb.velocity, fb.current_iq,
                   motor_temp_to_c(fb.temperature), fb.status_byte);
        }
    }

    printf("✓ Multi-ctrl: %d motors\n", n);
    return 0;
}

/* ================================================================
 * mit <id> <pos> <vel> <kp> <kd> <torque>
 *   位置和力矩参数×100精度
 * ================================================================ */

int cmd_do_mit(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 8) {
        fprintf(stderr, "Usage: motor_tool mit <id> <pos*100> <vel> <kp*100> <kd*100> <torque*100>\n");
        fprintf(stderr, "  pos:  position ×100 (°)       vel:  velocity (RPM)\n");
        fprintf(stderr, "  kp:   stiffness ×100           kd:  damping ×100\n");
        fprintf(stderr, "  torque: feedforward torque ×100\n");
        fprintf(stderr, "Examples:\n");
        fprintf(stderr, "  motor_tool mit 1 0 0 30 5 0        # 柔顺模式 (低刚度)\n");
        fprintf(stderr, "  motor_tool mit 1 3000 0 200 30 0   # 刚性位置 30°\n");
        return -1;
    }

    int   id      = atoi(argv[2]);
    float pos     = (float)atoi(argv[3]) / 100.0f;
    float vel     = (float)atoi(argv[4]);
    float kp      = (float)atoi(argv[5]) / 100.0f;
    float kd      = (float)atoi(argv[6]) / 100.0f;
    float torque  = (float)atoi(argv[7]) / 100.0f;

    int ret = motor_hal_mit_control(g_hal, (uint8_t)id, pos, vel, kp, kd, torque);
    if (ret == 0) printf("✓ Motor %d: MIT pos=%.2f° vel=%.0f kp=%.2f kd=%.2f tor=%.2f\n",
                          id, pos, vel, kp, kd, torque);
    else fprintf(stderr, "✗ Motor %d: MIT failed (ret=%d)\n", id, ret);
    return ret;
}
