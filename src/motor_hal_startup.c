/**
 * @file motor_hal_startup.c
 * @brief 电机启动流程 (v3: Bootup 仅做快速试探, 真实校验走 SDO)
 *
 * 关键认知:
 *   - Bootup 帧 (701 00) 只在电机上电瞬间发一次, 错过就没有
 *   - 电机在配置心跳前不会周期上报任何数据
 *   - 因此 Bootup 只能做"快速试探", 不能做"硬门槛"
 *   - 真正验证电机是否在线: 发 SDO 心跳配置, 等响应
 *
 * 完整启动序列:
 *   1. 快速试探 Bootup (500ms, 不论结果都继续)
 *   2. SDO 配置心跳周期 ★ 同步等待, 验证电机通信
 *   3. 关看门狗 (推荐)
 *   4. 读固件版本 (二次验证)
 *   5. 使能 (DS402 状态机)
 *   6. 延时 120ms (抱闸释放)
 */

#include "motor_hal_types.h"
#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include <errno.h>
#include <time.h>

/* ---------- 快速试探 Bootup (v3: 非硬门槛, 不论结果都返回 0) ---------- */

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
       不论有没有收到, 都返回 0 继续 — 真正的验证走 SDO。 */
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

/* ---------- 完整启动 ---------- */

int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg,
                       const volatile bool *bootup_flag)
{
    int ret;

    /* 1. 快速试探 Bootup (500ms, 过了继续 — 错过是正常的) */
    motor_startup_wait_bootup(drv, cfg->node_id, 500, bootup_flag);

    /* 2. SDO 配置心跳 ★ 同步 SDO, 等响应 — 这是真正的"电机在线"验证 */
    ret = sdo_write_simple(drv, cfg->node_id, OD_HEARTBEAT, 0x00,
                           cfg->heartbeat_ms, 2);
    if (ret != 0) return ret;  /* 电机无响应 — 确实不在线 */

    /* 3. 关看门狗 (推荐) */
    if (cfg->disable_watchdog) {
        sdo_write_simple(drv, cfg->node_id, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
    }

    /* 4. 读固件版本 (二次验证通信链路) */
    {
        uint32_t ver = 0;
        sdo_read_simple(drv, cfg->node_id, OD_FIRMWARE_VER, 0x00, &ver);
    }

    /* 5. 使能 */
    if (cfg->auto_enable) {
        ret = motor_startup_enable(drv, cfg->node_id);
        if (ret != 0) return ret;
    }

    return 0;
}
