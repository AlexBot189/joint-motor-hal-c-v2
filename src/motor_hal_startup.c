/**
 * @file motor_hal_startup.c
 * @brief 电机启动流程 (v2: bootup 通过 recv 线程标志位检测, 不自己 recv)
 *
 * 完整启动序列:
 *   1. 等待 Bootup 帧 (轮询 bootup_received 标志位, 由 recv 线程设置)
 *   2. SDO 配置心跳周期
 *   3. 关看门狗 (推荐)
 *   4. 读固件版本 (验证通信, 通过 SDO 队列)
 *   5. 使能 (DS402 状态机: Shutdown→SwitchOn→EnableOp)
 *   6. 延时 120ms (抱闸释放)
 */

#include "motor_hal_types.h"
#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include <errno.h>
#include <time.h>

/* ---------- 等待 Bootup (v2: 轮询标志位, 不自己 recv) ---------- */

int motor_startup_wait_bootup(can_driver_t *drv __attribute__((unused)),
                              uint8_t node_id, int timeout_ms,
                              const volatile bool *bootup_flag)
{
    int elapsed = 0;
    while (!*bootup_flag && elapsed < timeout_ms) {
        usleep(10000);   /* 10ms 轮询间隔 */
        elapsed += 10;
    }
    return *bootup_flag ? 0 : -ETIMEDOUT;
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

    /* 1. 等待 Bootup (标志位由 recv 线程设置) */
    ret = motor_startup_wait_bootup(drv, cfg->node_id, cfg->bootup_timeout_ms, bootup_flag);
    if (ret != 0) return ret;

    /* 2. 配置心跳 */
    {
        canfd_frame_t f;
        canopen_sdo_write_build(cfg->node_id, OD_HEARTBEAT, 0x00,
                                cfg->heartbeat_ms, 2, &f);
        can_driver_send(drv, &f);
    }

    /* 3. 关看门狗 (推荐) */
    if (cfg->disable_watchdog) {
        sdo_write_simple(drv, cfg->node_id, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
    }

    /* 4. 读固件版本 (验证) */
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
