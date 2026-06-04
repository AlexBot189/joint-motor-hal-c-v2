/**
 * @file motor_hal_startup.c
 * @brief 电机启动流程 (v4: Bootup 快速试探 + SDO 同步验证 + 固件读非阻塞)
 *
 * 完整启动序列:
 *   1. 快速试探 Bootup (500ms, 不论结果都继续)
 *   2. SDO 同步配置心跳 ★ 真正的电机在线验证
 *   3. 关看门狗 (推荐)
 *   4. 读固件版本 (best-effort, 失败不阻塞后续)
 *   5. 使能 (DS402 状态机: Shutdown→SwitchOn→EnableOp)
 *   6. 延时 120ms (抱闸释放)
 *
 * 改动:
 *   - 步骤1: bootup 等不到不报错, 继续往下走
 *   - 步骤2: fire-and-forget → 改同步 SDO, 真正的"电机在线"验证
 *   - 步骤4: 固件读失败不阻塞, 打印日志继续
 */

#include "motor_hal_types.h"
#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include <errno.h>
#include <time.h>
#include <stdio.h>

/* ---------- 快速试探 Bootup (v4: 非硬门槛, 不论结果都返回 0) ---------- */

int motor_startup_wait_bootup(can_driver_t *drv __attribute__((unused)),
                              uint8_t node_id __attribute__((unused)),
                              int timeout_ms,
                              const volatile bool *bootup_flag)
{
    int elapsed = 0;
    while (!*bootup_flag && elapsed < timeout_ms) {
        usleep(10000);
        elapsed += 10;
    }
    /* Bootup 是一次性事件, 错过了就错过了。
       不论有没有收到, 都返回 0 继续 — 真正的验证走步骤 2 的 SDO。 */
    return 0;
}

/* ---------- 使能 (DS402 状态机) ---------- */

int motor_startup_enable(can_driver_t *drv, uint8_t node_id)
{
    int ret;

    /* Step A: Shutdown → Ready to Switch On */
    ret = sdo_write_simple(drv, node_id, OD_CONTROLWORD, 0x00, CW_SHUTDOWN, 2);
    if (ret != 0) return ret;
    usleep(20000);

    /* Step B: Switch On → Switched On */
    ret = sdo_write_simple(drv, node_id, OD_CONTROLWORD, 0x00, CW_SWITCH_ON, 2);
    if (ret != 0) return ret;
    usleep(20000);

    /* Step C: Enable Operation → Operation Enabled */
    ret = sdo_write_simple(drv, node_id, OD_CONTROLWORD, 0x00, CW_ENABLE_OP, 2);
    if (ret != 0) return ret;

    /* 等待抱闸释放 (100ms + 余量) */
    usleep(120000);

    return 0;
}

/* ---------- 完整启动 (v4) ---------- */

int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg,
                       const volatile bool *bootup_flag)
{
    int ret;

    /* 1. 快速试探 Bootup (500ms, 不论结果都继续) */
    motor_startup_wait_bootup(drv, cfg->node_id, 500, bootup_flag);

    /* 2. SDO 同步配置心跳 ★ 用同步 SDO 等响应:
         有响应 → 电机确认在线 + 心跳周期生效
         超时   → 电机不在线, 返回错误 */
    ret = sdo_write_simple(drv, cfg->node_id, OD_HEARTBEAT, 0x00,
                           cfg->heartbeat_ms, 2);
    if (ret != 0) return ret;

    /* 3. 关看门狗 (推荐) */
    if (cfg->disable_watchdog) {
        sdo_write_simple(drv, cfg->node_id, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
    }

    /* 4. 读固件版本 (best-effort: 成功记录, 失败跳过不阻塞) */
    {
        uint32_t ver = 0;
        ret = sdo_read_simple(drv, cfg->node_id, OD_FIRMWARE_VER, 0x00, &ver);
        if (ret == 0) {
            fprintf(stderr, "[startup] motor %d firmware=0x%08X\n",
                    cfg->node_id, ver);
        } else {
            fprintf(stderr, "[startup] motor %d firmware read skipped (ret=%d)\n",
                    cfg->node_id, ret);
        }
    }

    /* 5. 使能 */
    if (cfg->auto_enable) {
        ret = motor_startup_enable(drv, cfg->node_id);
        if (ret != 0) return ret;
    }

    return 0;
}
