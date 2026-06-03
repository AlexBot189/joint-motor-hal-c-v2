/**
 * @file sdo_client.c
 * @brief SDO 客户端 - 对象字典同步读写
 *
 * 支持: 1/2/4 字节 Expedited Transfer, 自动重试。
 */

#include "canopen_frames.h"
#include "can_driver_internal.h"
#include <errno.h>
#include <time.h>

/* =====================================================
 * 内部函数: 等待 SDO 响应
 * ===================================================== */

static int _sdo_wait_response(can_driver_t *drv, uint8_t node,
                              uint16_t expected_index __attribute__((unused)),
                              uint32_t *value, uint32_t *abort_code,
                              int timeout_ms)
{
    uint32_t expect_rx_id = COB_SDO_RX_BASE + node;

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

        if (f.id != expect_rx_id) continue;  /* 非目标节点的 SDO */

        uint16_t idx = 0;
        bool ok = canopen_sdo_parse_response(&f, value, &idx, NULL, abort_code);
        if (!ok) return -ECANCELED;  /* SDO Abort */
        return 0;
    }
}

/* =====================================================
 * 公共接口
 * ===================================================== */

int sdo_write(can_driver_t *drv, uint8_t node,
              uint16_t index, uint8_t subidx,
              uint32_t value, uint8_t size_bytes,
              int retry_count, int timeout_ms)
{
    int last_err = 0;

    for (int attempt = 0; attempt <= retry_count; attempt++) {
        canfd_frame_t f;
        canopen_sdo_write_build(node, index, subidx, value, size_bytes, &f);

        if (can_driver_send(drv, &f) < 0) {
            last_err = -errno;
            if (attempt < retry_count) usleep(5000);
            continue;
        }

        uint32_t resp = 0, abort_code = 0;
        int ret = _sdo_wait_response(drv, node, index, &resp, &abort_code, timeout_ms);
        if (ret == 0) return 0;
        last_err = ret;

        if (attempt < retry_count) usleep(10000);
    }
    return last_err;
}

int sdo_read(can_driver_t *drv, uint8_t node,
             uint16_t index, uint8_t subidx,
             uint32_t *value,
             int retry_count, int timeout_ms)
{
    int last_err = 0;

    for (int attempt = 0; attempt <= retry_count; attempt++) {
        canfd_frame_t f;
        canopen_sdo_read_build(node, index, subidx, &f);

        if (can_driver_send(drv, &f) < 0) {
            last_err = -errno;
            if (attempt < retry_count) usleep(5000);
            continue;
        }

        uint32_t abort_code = 0;
        int ret = _sdo_wait_response(drv, node, index, value, &abort_code, timeout_ms);
        if (ret == 0) return 0;
        last_err = ret;

        if (attempt < retry_count) usleep(10000);
    }
    return last_err;
}

int sdo_write_simple(can_driver_t *drv, uint8_t node,
                     uint16_t index, uint8_t subidx,
                     uint32_t value, uint8_t size_bytes)
{
    return sdo_write(drv, node, index, subidx, value, size_bytes, 2, 200);
}

int sdo_read_simple(can_driver_t *drv, uint8_t node,
                    uint16_t index, uint8_t subidx,
                    uint32_t *value)
{
    return sdo_read(drv, node, index, subidx, value, 2, 200);
}
