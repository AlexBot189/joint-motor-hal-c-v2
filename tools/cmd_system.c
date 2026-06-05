/**
 * @file cmd_system.c
 * @brief 系统命令: init / startup / enable / disable / reset (v2: 广播使用已注册电机列表)
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

/* ================================================================
 * init <can_iface> — daemon 内部调用, client 端不应手动调
 * ================================================================ */

int cmd_do_init(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    return tool_init(argv[2]);
}

/* ================================================================
 * startup <id> — 手动启动电机 (daemon 已自动扫描, 此命令用于追加)
 *   如果电机已注册, 只做 startup, 不重复 add_motor
 * ================================================================ */

static int _startup_single(int id)
{
    /* 尝试注册 (已注册则忽略 -EEXIST) */
    motor_config_t cfg = {0};
    cfg.node_id           = (uint8_t)id;
    cfg.heartbeat_ms      = 2000;
    cfg.profile_accel     = 5000;
    cfg.profile_decel     = 5000;
    cfg.profile_velocity  = 20;
    cfg.disable_watchdog  = true;
    cfg.auto_enable       = true;
    cfg.bootup_timeout_ms = 5000;

    int ret = motor_hal_add_motor(g_hal, &cfg);
    if (ret != 0 && ret != -EEXIST) {
        fprintf(stderr, "✗ Motor %d: add failed (ret=%d)\n", id, ret);
        return -1;
    }

    printf("Starting motor %d...\n", id);
    ret = motor_hal_startup(g_hal, (uint8_t)id, 5000);
    if (ret != 0) {
        fprintf(stderr, "✗ Motor %d startup failed (ret=%d)\n", id, ret);
        return -1;
    }

    tool_register_motor(id);  /* 确保在广播列表中 */
    printf("✓ Motor %d: OPERATION_ENABLED\n", id);
    return 0;
}

int cmd_do_startup(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    int id = atoi(argv[2]);
    if (id < 0 || id > 127) { fprintf(stderr, "Invalid ID: %d\n", id); return -1; }

    if (id == 0) {
        /* 广播: 尝试 ID 1~4 */
        int errors = 0;
        for (int i = 1; i <= 4; i++) {
            if (_startup_single(i) < 0) errors++;
        }
        return errors > 0 ? -1 : 0;
    }

    return _startup_single(id);
}

/* ================================================================
 * enable / disable / reset — 广播时使用已注册电机列表
 * ================================================================ */

static int _broadcast_op(int id,
                          int (*fn_single)(motor_hal_t*, uint8_t),
                          const char *action)
{
    if (id == 0) {
        int count = tool_motor_count();
        if (count == 0) {
            fprintf(stderr, "No motors registered\n");
            return -1;
        }
        int errors = 0;
        for (int i = 0; i < count; i++) {
            int mid = tool_motor_id(i);
            int ret = fn_single(g_hal, (uint8_t)mid);
            if (ret == 0)
                printf("✓ Motor %d: %s OK\n", mid, action);
            else {
                fprintf(stderr, "✗ Motor %d: %s failed (ret=%d)\n", mid, action, ret);
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
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), _enable_fn, "enable");
}

int cmd_do_disable(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), _disable_fn, "disable");
}

int cmd_do_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id; (void)argc;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return _broadcast_op(atoi(argv[2]), _reset_fn, "fault_reset");
}

/* ---- 新增: fault_reset / reboot ---- */

int cmd_do_fault_reset(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return tool_fault_reset(atoi(argv[2]));
}

int cmd_do_reboot(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }
    return tool_reboot(atoi(argv[2]));
}
