/*
 * @file perf_test.cpp
 * @brief 共享内存性能测试
 *
 * 测试项:
 *   1. SHM atomic 读延迟
 *   2. SHM atomic 写延迟
 *   3. SHM feedback_frame_t 帧拷贝延迟
 *   4. LIVE STATUS — 当前 SHM 状态快照
 *
 * 编译: g++ -O2 -o perf_test perf_test.cpp -I.. -lrt -lpthread
 * 运行: ./perf_test
 * 前置: stark_periph_manager_node 必须先启动并创建 SHM
 */

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "exo_shm.h"

static long timespec_diff_ns(const struct timespec *start,
                              const struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1000000000L +
           (end->tv_nsec - start->tv_nsec);
}

int main()
{
    /* ── 打开 SHM ── */
    int fd = shm_open(EXO_SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        perror("[perf_test] shm_open: start stark_periph_manager_node first");
        return 1;
    }

    exo_shm_t *shm = (exo_shm_t *)mmap(NULL, EXO_SHM_SIZE,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, 0);
    if (shm == MAP_FAILED) {
        perror("[perf_test] mmap");
        close(fd);
        return 1;
    }

    printf("[perf_test] SHM at %p\n", (void *)shm);
    printf("[perf_test] SHM size: %zu bytes (struct size: %zu bytes)\n",
           (size_t)EXO_SHM_SIZE, sizeof(exo_shm_t));
    printf("[perf_test] feedback_frame_t size: %zu bytes\n\n",
           sizeof(feedback_frame_t));

    /* ════════════════════════════════════════════════════════════════
     * 测试1: SHM atomic 读延迟 (__atomic_load_n)
     * ════════════════════════════════════════════════════════════════ */
    {
        struct timespec t1, t2;
        const int N = 100000;
        volatile uint32_t dummy;

        clock_gettime(CLOCK_MONOTONIC, &t1);
        for (int i = 0; i < N; i++) {
            dummy = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        }
        clock_gettime(CLOCK_MONOTONIC, &t2);
        (void)dummy;

        long ns = timespec_diff_ns(&t1, &t2);
        printf("├─ 测试1: SHM atomic read (acquire)\n");
        printf("│    %ld ns/op  (avg over %d ops)\n", ns / N, N);
        printf("│    total: %ld ns (%.1f us)\n\n", ns, ns / 1000.0);
    }

    /* ════════════════════════════════════════════════════════════════
     * 测试2: SHM atomic 写延迟 (__atomic_store_n)
     * ════════════════════════════════════════════════════════════════ */
    {
        struct timespec t1, t2;
        const int N = 100000;

        clock_gettime(CLOCK_MONOTONIC, &t1);
        for (int i = 0; i < N; i++) {
            __atomic_store_n(&shm->mailbox.seq_begin,
                             (uint64_t)i, __ATOMIC_RELEASE);
        }
        clock_gettime(CLOCK_MONOTONIC, &t2);

        long ns = timespec_diff_ns(&t1, &t2);
        printf("├─ 测试2: SHM atomic write (release)\n");
        printf("│    %ld ns/op  (avg over %d ops)\n", ns / N, N);
        printf("│    total: %ld ns (%.1f us)\n\n", ns, ns / 1000.0);
    }

    /* ════════════════════════════════════════════════════════════════
     * 测试3: feedback_frame_t 帧拷贝延迟 (memcpy)
     * ════════════════════════════════════════════════════════════════ */
    {
        struct timespec t1, t2;
        const int N = 100000;
        feedback_frame_t tmp;

        clock_gettime(CLOCK_MONOTONIC, &t1);
        for (int i = 0; i < N; i++) {
            __builtin_memcpy(&tmp, &shm->fb_buffer[0],
                             sizeof(feedback_frame_t));
        }
        clock_gettime(CLOCK_MONOTONIC, &t2);

        long ns = timespec_diff_ns(&t1, &t2);
        printf("├─ 测试3: SHM frame copy (memcpy %zu bytes)\n",
               sizeof(feedback_frame_t));
        printf("│    %ld ns/op  (avg over %d ops)\n", ns / N, N);
        printf("│    total: %ld ns (%.1f us)\n", ns, ns / 1000.0);

        /* 带宽估算 */
        double bytes_total = (double)N * sizeof(feedback_frame_t);
        double sec = ns / 1e9;
        printf("│    bandwidth: %.1f MB/s\n\n",
               (bytes_total / sec) / (1024.0 * 1024.0));
    }

    /* ════════════════════════════════════════════════════════════════
     * 测试4: RT 抖动估算 (1000 次 1ms sleep 实际耗时统计)
     * ════════════════════════════════════════════════════════════════ */
    {
        const int N = 1000;
        long min_ns = 0x7FFFFFFFFFFFFFFFL;
        long max_ns = 0;
        long sum_ns = 0;

        for (int i = 0; i < N; i++) {
            struct timespec t1, t2;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            usleep(1000);
            clock_gettime(CLOCK_MONOTONIC, &t2);

            long dt = timespec_diff_ns(&t1, &t2);
            if (dt < min_ns) min_ns = dt;
            if (dt > max_ns) max_ns = dt;
            sum_ns += dt;
        }

        long avg_ns = sum_ns / N;
        long jitter = max_ns - min_ns;

        printf("├─ 测试4: usleep(1000) jitter 估算\n");
        printf("│    min: %ld ns (%.1f us)\n", min_ns, min_ns / 1000.0);
        printf("│    avg: %ld ns (%.1f us)\n", avg_ns, avg_ns / 1000.0);
        printf("│    max: %ld ns (%.1f us)\n", max_ns, max_ns / 1000.0);
        printf("│    jitter (max-min): %ld ns (%.1f us)\n\n",
               jitter, jitter / 1000.0);
    }

    /* ════════════════════════════════════════════════════════════════
     * LIVE STATUS
     * ════════════════════════════════════════════════════════════════ */
    {
        printf("┌─────────────────────────────────────────────┐\n");
        printf("│           LIVE SHM STATUS                   │\n");
        printf("├─────────────────────────────────────────────┤\n");
        printf("│ active_idx       : %-4u                      │\n",
               shm->active_idx);
        printf("│ node_state       : %-4u                      │\n",
               shm->node_state);
        printf("│ motor_online     : 0x%02X                     │\n",
               shm->motor_online);
        printf("│ motor_enabled    : 0x%02X                     │\n",
               shm->motor_enabled);
        printf("│ motor_severity   : %-4u                      │\n",
               shm->motor_severity);
        printf("│ fault_reason     : %-4u                      │\n",
               shm->fault_reason);
        printf("│ calib_state      : %-4u                      │\n",
               shm->calib_state);
        printf("│ mailbox.seq_begin: %-10lu              │\n",
               (unsigned long)shm->mailbox.seq_begin);
        printf("│ mailbox.seq_end  : %-10lu              │\n",
               (unsigned long)shm->mailbox.seq_end);
        printf("│ mailbox.cmd.motor: %-4u                      │\n",
               shm->mailbox.cmd.motor_id);
        printf("│ mailbox.cmd.cmd  : %-4u                      │\n",
               shm->mailbox.cmd.cmd);
        printf("│ mailbox.cmd.value: %-6d                  │\n",
               shm->mailbox.cmd.value);
        printf("└─────────────────────────────────────────────┘\n");
    }

    /* ── 清理 ── */
    munmap(shm, EXO_SHM_SIZE);
    close(fd);

    return 0;
}
