/**
 * @file cmd_byte0.c
 * @brief PDO Byte0 CLI 控制命令
 *
 * 用法:
 *   motor_tool pdo_enable <id>           PDO使能 (Byte0 bit7=1)
 *   motor_tool pdo_disable <id>          PDO失能 (Byte0 bit7=0)
 *   motor_tool bus_on <id>              母线接通 (Byte0 bit6=1, 预留)
 *   motor_tool bus_off <id>             母线断开 (Byte0 bit6=0, 预留)
 *   motor_tool estop <id>               急停: enable=0 + bus=OFF
 *   motor_tool recover <id>             恢复: enable=1 + bus=ON
 *   motor_tool clearcf <id>             清故障脉冲 (Byte0 bit5, 下一帧自动清0)
 *   motor_tool setmode <id> <1~6>       切换 PDO 控制模式
 *   motor_tool byte0 <id> [0xHH]        读/写原始 Byte0 值 (调试)
 *
 *   注意: SDO enable/disable 命令不走 Byte0, 走 DS402 状态机。
 *         PDO enable/disable 仅控制 PDO 帧 bit7, 不改 DS402。
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>

/* broadcast helper */
static int _broadcast_op(int id, int (*fn)(motor_hal_t*, uint8_t), const char *label)
{
    if (id == 0) {
        int count = tool_motor_count();
        if (count == 0) { fprintf(stderr, "No motors registered\n"); return -1; }
        for (int i = 0; i < count; i++) {
            int mid = tool_motor_id(i);
            int ret = fn(g_hal, (uint8_t)mid);
            if (ret == 0) printf("✓ Motor %d: %s\n", mid, label);
            else fprintf(stderr, "✗ Motor %d: %s failed (ret=%d)\n", mid, label, ret);
        }
        return 0;
    }
    int ret = fn(g_hal, (uint8_t)id);
    if (ret == 0) printf("✓ Motor %d: %s\n", id, label);
    else fprintf(stderr, "✗ Motor %d: %s failed (ret=%d)\n", id, label, ret);
    return ret;
}

#define BCAST(fn) ((int(*)(motor_hal_t*,uint8_t))(fn))

int cmd_do_pdo_enable(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_enable), "PDO enable (bit7=1)");
}

int cmd_do_pdo_disable(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_disable), "PDO disable (bit7=0)");
}

int cmd_do_bus_on(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_bus_on), "bus ON (bit6=1)");
}

int cmd_do_bus_off(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_bus_off), "bus OFF (bit6=0)");
}

int cmd_do_estop(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_estop), "ESTOP (Byte0, 0x00)");
}

int cmd_do_recover(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_recover), "recover (Byte0, 0xC0)");
}

int cmd_do_clearcf(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), BCAST(motor_hal_pdo_clear_fault), "clear fault (bit5 pulse)");
}

int cmd_do_setmode(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 4) {
        fprintf(stderr, "Usage: motor_tool setmode <id> <1~6>\n");
        fprintf(stderr, "  1=PP 2=PV 3=CSP 4=CSV 5=Current 6=MIT\n");
        return -1;
    }
    int id   = atoi(argv[2]);
    int mode = atoi(argv[3]);
    if (mode < 1 || mode > 6) { fprintf(stderr, "mode %d out of range\n", mode); return -1; }

    const char *names[] = {"","PP","PV","CSP","CSV","Current","MIT"};
    int ret = motor_hal_pdo_set_mode(g_hal, (uint8_t)id, (motor_mode_t)mode);
    if (ret == 0) printf("✓ Motor %d: PDO mode ,  %s\n", id, names[mode]);
    else fprintf(stderr, "✗ Motor %d: setmode failed\n", id);
    return ret;
}

int cmd_do_byte0(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    int id = atoi(argv[2]);

    if (argc >= 4) {
        uint8_t val = (uint8_t)strtol(argv[3], NULL, 0);
        int ret = motor_hal_pdo_set_byte0(g_hal, (uint8_t)id, val);
        if (ret == 0) {
            printf("✓ Motor %d: Byte0 set to 0x%02X (en=%d bus=%d clr=%d mode=%d)\n",
                   id, val, pdo_byte0_get_enable(val),
                   pdo_byte0_get_bus_on(val),
                   pdo_byte0_get_clr_err(val),
                   pdo_byte0_get_mode(val));
        }
        return ret;
    }

    uint8_t b0;
    int ret = motor_hal_pdo_get_byte0(g_hal, (uint8_t)id, &b0);
    if (ret == 0) {
        printf("Motor %d Byte0 = 0x%02X\n", id, b0);
        printf("  bit7 enable=%d  bit6 bus_ON=%d  bit5 clr_err=%d  mode=%d\n",
               pdo_byte0_get_enable(b0), pdo_byte0_get_bus_on(b0),
               pdo_byte0_get_clr_err(b0), pdo_byte0_get_mode(b0));
    }
    return ret;
}

/* ================================================================
 * "Now" 命令: 改 Byte0 后立即发 PDO 帧, 不走 SDO
 *
 * disable_now / estop_now 需要先发包再断 enabled:
 *   motor_hal_pdo_set_byte0 ,  发包(此时 enabled 还是 true)
 *   ,  motor_hal_pdo_disable/estop ,  设 enabled=false
 * ================================================================ */

static int _now_op(int id,
                   void (*set_byte0_fn)(motor_hal_t*, uint8_t, uint8_t), uint8_t byte0_val,
                   void (*final_fn)(motor_hal_t*, uint8_t),
                   const char *label)
{
    if (id == 0) {
        int count = tool_motor_count();
        if (count == 0) { fprintf(stderr, "No motors registered\n"); return -1; }
        for (int i = 0; i < count; i++) {
            int mid = tool_motor_id(i);
            set_byte0_fn(g_hal, (uint8_t)mid, byte0_val);
            motor_hal_set_torque(g_hal, (uint8_t)mid, 0);
            final_fn(g_hal, (uint8_t)mid);
            printf("✓ Motor %d: %s + PDO sent\n", mid, label);
        }
        return 0;
    }
    set_byte0_fn(g_hal, (uint8_t)id, byte0_val);
    motor_hal_set_torque(g_hal, (uint8_t)id, 0);
    final_fn(g_hal, (uint8_t)id);
    printf("✓ Motor %d: %s + PDO sent\n", id, label);
    return 0;
}

int cmd_do_pdo_enable_now(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id = atoi(argv[2]);
    motor_hal_pdo_enable(g_hal, (uint8_t)id);
    motor_hal_set_torque(g_hal, (uint8_t)id, 0);
    printf("✓ Motor %d: pdo_enable_now + PDO sent\n", id);
    return 0;
}

int cmd_do_pdo_disable_now(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id = atoi(argv[2]);
    /* 先清 bit7 但保持 enabled=true, 发包后再正式 disable */
    uint8_t b0;
    if (motor_hal_pdo_get_byte0(g_hal, (uint8_t)id, &b0) == 0) {
        motor_hal_pdo_set_byte0(g_hal, (uint8_t)id, b0 & ~PDO_BYTE0_ENABLE);
    }
    motor_hal_set_torque(g_hal, (uint8_t)id, 0);
    motor_hal_pdo_disable(g_hal, (uint8_t)id);
    printf("✓ Motor %d: pdo_disable_now + PDO sent\n", id);
    return 0;
}

int cmd_do_estop_now(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _now_op(atoi(argv[2]),
                   motor_hal_pdo_set_byte0, 0x00,                          /* Byte0=0x00 */
                   motor_hal_pdo_estop, "estop_now");
}

int cmd_do_recover_now(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    int id = atoi(argv[2]);
    motor_hal_pdo_recover(g_hal, (uint8_t)id);
    motor_hal_set_torque(g_hal, (uint8_t)id, 0);
    printf("✓ Motor %d: recover_now + PDO sent\n", id);
    return 0;
}
