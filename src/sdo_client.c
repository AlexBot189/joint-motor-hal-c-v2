/**
 * @file sdo_client.c
 * @brief SDO 客户端 - 对象字典同步读写 (队列模式)
 *
 * v2 改动:
 *   - 不再自己 recv CAN 帧
 *   - 改为等内部队列 + 条件变量
 *   - 接收线程通过 sdo_push_response() 喂帧
 */

#define _GNU_SOURCE
#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include <errno.h>
#include <time.h>
#include <pthread.h>

/* =====================================================
 * SDO 响应队列 (全局, 接收线程写 → SDO客户端读)
 * ===================================================== */

#define SDO_QUEUE_SIZE 32

typedef struct {
    canfd_frame_t frames[SDO_QUEUE_SIZE];
    int           head;     /* 写入位置 */
    int           tail;     /* 读取位置 */
    int           count;    /* 当前数量 */
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
} sdo_queue_t;

static sdo_queue_t g_sdo_queue = { .head = 0, .tail = 0, .count = 0 };

/* =====================================================
 * 初始化 (在 motor_hal_create 调用)
 * ===================================================== */

void sdo_queue_init(void)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&g_sdo_queue.mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    pthread_cond_init(&g_sdo_queue.cond, NULL);
    g_sdo_queue.head  = 0;
    g_sdo_queue.tail  = 0;
    g_sdo_queue.count = 0;
}

void sdo_queue_destroy(void)
{
    pthread_mutex_destroy(&g_sdo_queue.mutex);
    pthread_cond_destroy(&g_sdo_queue.cond);
}

/* =====================================================
 * 接收线程调用: SDO 响应帧入队
 * ===================================================== */

void sdo_push_response(const canfd_frame_t *f)
{
    if (!f) return;

    pthread_mutex_lock(&g_sdo_queue.mutex);

    if (g_sdo_queue.count >= SDO_QUEUE_SIZE) {
        /* 队列满 → 丢弃最旧的 (不应该发生, 但防御) */
        g_sdo_queue.tail = (g_sdo_queue.tail + 1) % SDO_QUEUE_SIZE;
        g_sdo_queue.count--;
    }

    memcpy(&g_sdo_queue.frames[g_sdo_queue.head], f, sizeof(canfd_frame_t));
    g_sdo_queue.head = (g_sdo_queue.head + 1) % SDO_QUEUE_SIZE;
    g_sdo_queue.count++;

    pthread_cond_signal(&g_sdo_queue.cond);
    pthread_mutex_unlock(&g_sdo_queue.mutex);
}

/* =====================================================
 * SDO 客户端调用: 阻塞等匹配的响应
 * ===================================================== */

static int _sdo_wait_response(can_driver_t *drv __attribute__((unused)),
                              uint8_t node,
                              uint16_t expected_index __attribute__((unused)),
                              uint32_t *value, uint32_t *abort_code,
                              int timeout_ms)
{
    uint32_t expect_rx_id = COB_SDO_RX_BASE + node;

    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&g_sdo_queue.mutex);

    while (1) {
        /* 扫描队列, 找匹配节点 ID 的 SDO 响应 */
        /* 注意: 可能有多帧在队列里, 需要匹配 COB-ID */
        int found = -1;
        for (int i = 0; i < g_sdo_queue.count; i++) {
            int idx = (g_sdo_queue.tail + i) % SDO_QUEUE_SIZE;
            if (g_sdo_queue.frames[idx].id == expect_rx_id) {
                found = i;
                break;
            }
        }

        if (found >= 0) {
            /* 找到 → 解析 → 从队列移除 */
            int idx = (g_sdo_queue.tail + found) % SDO_QUEUE_SIZE;
            canfd_frame_t f = g_sdo_queue.frames[idx];

            /* 紧凑移除: 移动后续帧 */
            for (int j = found; j < g_sdo_queue.count - 1; j++) {
                int src = (g_sdo_queue.tail + j + 1) % SDO_QUEUE_SIZE;
                int dst = (g_sdo_queue.tail + j) % SDO_QUEUE_SIZE;
                memcpy(&g_sdo_queue.frames[dst], &g_sdo_queue.frames[src], sizeof(canfd_frame_t));
            }
            g_sdo_queue.head = (g_sdo_queue.head + SDO_QUEUE_SIZE - 1) % SDO_QUEUE_SIZE;
            g_sdo_queue.count--;

            pthread_mutex_unlock(&g_sdo_queue.mutex);

            /* 解析响应 */
            uint16_t idx_resp = 0;
            bool ok = canopen_sdo_parse_response(&f, value, &idx_resp, NULL, abort_code);
            if (!ok) return -ECANCELED;
            return 0;
        }

        /* 没找到 → 条件变量等 */
        int ret = pthread_cond_timedwait(&g_sdo_queue.cond,
                                         &g_sdo_queue.mutex, &deadline);
        if (ret == ETIMEDOUT) {
            pthread_mutex_unlock(&g_sdo_queue.mutex);
            return -ETIMEDOUT;
        }
        /* spurious wakeup → 重新扫描 */
    }
}

/* =====================================================
 * 公共接口 (不变)
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
