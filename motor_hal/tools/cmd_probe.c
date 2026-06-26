/**
 * @file cmd_probe.c
 * @brief 电机在线探测: motor_tool probe [id]
 *
 * 探测策略:
 *   1. 读反馈缓存 (最快, 如果已收到过反馈帧)
 *   2. 主动 SDO 读 0x100A (固件版本) 验证通信
 *   3. 都不通则标记离线
 *
 * 用法:
 *   motor_tool probe 1      # 探测电机 1
 *   motor_tool probe 0      # 探测所有已注册电机
 *   motor_tool probe        # 同上
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int _probe_single(int id)
{
    int online = 0;

    /* 1. 检查反馈缓存 (接收线程已收到过帧) */
    motor_feedback_t fb;
    if (motor_hal_get_feedback(g_hal, (uint8_t)id, &fb) == 0 && fb.status_byte != 0) {
        printf("Motor %d: ONLINE  (feedback OK, pos=%d vel=%d cur=%d)\n",
               id, fb.position, fb.velocity, fb.current_iq);
        return 1;
    }

    /* 2. 主动 SDO 读固件版本 (Ping) */
    uint32_t ver = 0;
    int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)id, 0x100A, 0x00, &ver);
    if (ret == 0) {
        printf("Motor %d: ONLINE  (SDO ping OK, fw=0x%08X)\n", id, ver);
        /* 顺便注册 */
        motor_config_t cfg = {0};
        cfg.node_id = (uint8_t)id; cfg.heartbeat_ms = 2000;
        cfg.disable_watchdog = true; cfg.auto_enable = false;
        motor_hal_add_motor(g_hal, &cfg);
        tool_register_motor(id);
        return 1;
    }

    /* 3. 离线 */
    const char *reason = motor_utils_sdo_abort_str((uint32_t)(-ret));
    printf("Motor %d: OFFLINE (no feedback, SDO failed: %s)\n",
           id, reason ? reason : "no response");
    return 0;
}

int cmd_do_probe(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    int id = (argc >= 3) ? atoi(argv[2]) : 0;

    if (id == 0) {
        /* 探测已注册电机列表; 如果还没注册, 尝试 1,2 */
        int count = tool_motor_count();
        if (count == 0) {
            printf("No motors registered. Trying IDs 1,2...\n");
            int r = _probe_single(1);
            int l = _probe_single(2);
            printf("\nResult: R=%s L=%s\n",
                   r ? "ONLINE" : "OFFLINE", l ? "ONLINE" : "OFFLINE");
            return (r && l) ? 0 : 1;
        }
        int online = 0, total = 0;
        for (int i = 0; i < count; i++) {
            int mid = tool_motor_id(i);
            if (_probe_single(mid)) online++;
            total++;
        }
        printf("\nResult: %d/%d online\n", online, total);
        return (online == total) ? 0 : 1;
    }

    return _probe_single(id) ? 0 : 1;
}
