/**
 * @file motor_hal_startup.c
 * @brief 电机启动流程
 *
 * 完整启动序列:
 *   1. 等待 Bootup 帧 (0x700+ID, data[0]==0)
 *   2. SDO 配置心跳周期
 *   3. 关看门狗 (推荐)
 *   4. 读固件版本 (验证通信)
 *   5. 使能 (DS402 状态机: Shutdown→SwitchOn→EnableOp)
 *   6. 延时 100ms (抱闸释放)
 */

#include "motor_hal_types.h"
#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include <errno.h>
#include <time.h>

/* ---------- 等待 Bootup ---------- */

int motor_startup_wait_bootup(can_driver_t *drv, uint8_t node_id, int timeout_ms)
{
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }

    while (1) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        long remaining_ms = (deadline.tv_sec - now.tv_sec) * 1000
                          + (deadline.tv_nsec - now.tv_nsec) / 1000000L;
        if (remaining_ms <= 0) return -ETIMEDOUT;

        canfd_frame_t f;
        int ret = can_driver_recv(drv, &f,
                    (int)(remaining_ms > 50 ? 50 : remaining_ms));
        if (ret <= 0) continue;

        if (canopen_is_bootup(f.id, f.data[0])) {
            uint8_t n = canopen_extract_node(f.id, COB_BOOTUP_BASE);
            if (n == node_id) {
                return 0;
            }
        }
    }
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

int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg)
{
    int ret;

    /* 1. 等待 Bootup */
    ret = motor_startup_wait_bootup(drv, cfg->node_id, cfg->bootup_timeout_ms);
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
