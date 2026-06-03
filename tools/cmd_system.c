/**
 * @file cmd_system.c
 * @brief 系统命令: init / startup / enable / disable / reset
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 * init <can_iface>
 * ================================================================ */

int cmd_do_init(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    return tool_init(argv[2]);
}

/* ================================================================
 * startup <id> — 注册 + 启动
 * ================================================================ */

int cmd_do_startup(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }

    int id = atoi(argv[2]);
    if (id < 0 || id > 127) { fprintf(stderr, "Invalid ID: %d\n", id); return -1; }

    /* id=0 广播: 启动电机1和2 */
    if (id == 0) {
        int num_motors = 2;  /* 方案A: 硬编码双电机 */
        int errors = 0;

        for (int i = 1; i <= num_motors; i++) {
            motor_config_t cfg = {0};
            cfg.node_id           = (uint8_t)i;
            cfg.heartbeat_ms      = 2000;
            cfg.profile_accel     = 5000;
            cfg.profile_decel     = 5000;
            cfg.profile_velocity  = 20;
            cfg.disable_watchdog  = true;
            cfg.auto_enable       = true;
            cfg.bootup_timeout_ms = 5000;

            motor_hal_add_motor(g_hal, &cfg);
            tool_register_motor(i);

            printf("Starting motor %d...\n", i);
            int ret = motor_hal_startup(g_hal, (uint8_t)i, 5000);
            if (ret == 0) {
                printf("  ✓ Motor %d: OPERATION_ENABLED\n", i);
            } else {
                fprintf(stderr, "  ✗ Motor %d startup failed (ret=%d)\n", i, ret);
                errors++;
            }
        }
        return errors > 0 ? -1 : 0;
    }

    /* 单电机启动 */
    motor_config_t cfg = {0};
    cfg.node_id           = (uint8_t)id;
    cfg.heartbeat_ms      = 2000;
    cfg.profile_accel     = 5000;
    cfg.profile_decel     = 5000;
    cfg.profile_velocity  = 20;
    cfg.disable_watchdog  = true;
    cfg.auto_enable       = true;
    cfg.bootup_timeout_ms = 5000;

    motor_hal_add_motor(g_hal, &cfg);
    tool_register_motor(id);

    printf("Starting motor %d...\n", id);
    int ret = motor_hal_startup(g_hal, (uint8_t)id, 5000);
    if (ret != 0) {
        fprintf(stderr, "✗ Motor %d startup failed (ret=%d)\n", id, ret);
        return -1;
    }
    printf("✓ Motor %d: OPERATION_ENABLED\n", id);
    return 0;
}

/* ================================================================
 * enable / disable / reset
 * ================================================================ */

static int _single_or_broadcast(int id,
                                int (*fn_single)(motor_hal_t*, uint8_t),
                                const char *action)
{
    if (id == 0) {
        int errors = 0;
        for (int i = 1; i <= 2; i++) {
            int ret = fn_single(g_hal, (uint8_t)i);
            if (ret == 0)
                printf("✓ Motor %d: %s OK\n", i, action);
            else {
                fprintf(stderr, "✗ Motor %d: %s failed (ret=%d)\n", i, action, ret);
                errors++;
            }
        }
        return errors > 0 ? -1 : 0;
    }
    int ret = fn_single(g_hal, (uint8_t)id);
    if (ret != 0) {
        fprintf(stderr, "✗ Motor %d: %s failed (ret=%d)\n", id, action, ret);
        return -1;
    }
    printf("✓ Motor %d: %s OK\n", id, action);
    return 0;
}

static int _enable_fn(motor_hal_t *h, uint8_t id)  { return motor_hal_enable(h, id); }
static int _disable_fn(motor_hal_t *h, uint8_t id) { return motor_hal_disable(h, id); }
static int _reset_fn(motor_hal_t *h, uint8_t id)   { return motor_hal_fault_reset(h, id); }

int cmd_do_enable(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    return _single_or_broadcast(atoi(argv[2]), _enable_fn, "enable");
}

int cmd_do_disable(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    return _single_or_broadcast(atoi(argv[2]), _disable_fn, "disable");
}

int cmd_do_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: call 'init' first\n"); return -1; }
    return _single_or_broadcast(atoi(argv[2]), _reset_fn, "fault_reset");
}
