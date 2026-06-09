/*
 * perf_test.cpp — 共享内存 + 延迟追踪性能测试
 *
 * 测试项:
 *   1. SHM atomic 读/写延迟
 *   2. feedback_frame_t memcpy 延迟
 *   3. RT 抖动估算 (SCHED_OTHER)
 *   4. LIVE STATUS — 含延迟追踪字段
 *
 * 编译: g++ -O2 -o perf_test perf_test.cpp -I.. -lrt -lpthread
 * 运行: ./perf_test
 */

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "exo_shm.h"

static long timespec_diff_ns(const struct timespec* start,
                              const struct timespec* end)
{
    return (end->tv_sec - start->tv_sec) * 1000000000L +
           (end->tv_nsec - start->tv_nsec);
}

void print_latency_section(const char* title, uint64_t t0, uint64_t t1,
                            uint64_t t2, uint64_t t3, const feedback_frame_t* fb)
{
    printf("├─ %s\n", title);
    if (t0 > 0 && fb) {
        printf("│   T0 (CAN RX)    : %lu us\n", (unsigned long)fb->ts_can_rx);
        printf("│   T1 (cache wr)  : %lu us\n", (unsigned long)fb->ts_cache_write);
    }
    printf("│   T2 (fb read)   : %lu us\n", (unsigned long)t2);
    printf("│   T3 (SHM write) : %lu us\n", (unsigned long)t3);
    if (fb) {
        printf("│   T4 (algo read) : %lu us\n", (unsigned long)fb->ts_algo_read);
        printf("│   T5 (algo done) : %lu us\n", (unsigned long)fb->ts_algo_done);
        printf("│   latency T2→T3  : %ld us\n", (long)(t3 - t2));
        if (fb->ts_can_rx > 0) {
            printf("│   latency T0→T3  : %ld us\n", (long)(t3 - fb->ts_can_rx));
        }
    }
}

int main()
{
    int fd = shm_open(EXO_SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        perror("[perf_test] shm_open: start stark_periph_manager_node first");
        return 1;
    }

    exo_shm_t* shm = (exo_shm_t*)mmap(NULL, EXO_SHM_SIZE,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED, fd, 0);
    if (shm == MAP_FAILED) {
        perror("[perf_test] mmap");
        close(fd);
        return 1;
    }

    printf("[perf_test] SHM at %p (%zu KB)\n", (void*)shm, EXO_SHM_SIZE / 1024);
    printf("[perf_test] feedback_frame_t: %zu bytes\n", sizeof(feedback_frame_t));
    printf("[perf_test] exo_shm_t: %zu bytes\n\n", sizeof(exo_shm_t));

    /* ── 测试 1: atomic read ── */
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
        printf("├─ SHM atomic read (acquire)\n");
        printf("│    %ld ns/op (avg over %d ops)\n\n", ns / N, N);
    }

    /* ── 测试 2: atomic write ── */
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
        printf("├─ SHM atomic write (release)\n");
        printf("│    %ld ns/op (avg over %d ops)\n\n", ns / N, N);
    }

    /* ── 测试 3: feedback_frame_t memcpy ── */
    {
        struct timespec t1, t2;
        const int N = 100000;
        feedback_frame_t tmp;

        clock_gettime(CLOCK_MONOTONIC, &t1);
        for (int i = 0; i < N; i++) {
            memcpy(&tmp, &shm->fb_buffer[0], sizeof(feedback_frame_t));
        }
        clock_gettime(CLOCK_MONOTONIC, &t2);

        long ns = timespec_diff_ns(&t1, &t2);
        double bytes_total = (double)N * sizeof(feedback_frame_t);
        double sec = ns / 1e9;
        printf("├─ SHM feedback_frame_t memcpy (%zu bytes)\n", sizeof(feedback_frame_t));
        printf("│    %ld ns/op (avg over %d ops)\n", ns / N, N);
        printf("│    bandwidth: %.1f MB/s\n\n",
               (bytes_total / sec) / (1024.0 * 1024.0));
    }

    /* ── 测试 4: RT 抖动 (SCHED_OTHER) ── */
    {
        const int N = 1000;
        long min_ns = 0x7FFFFFFFFFFFFFFFL;
        long max_ns = 0, sum_ns = 0;

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

        printf("├─ usleep(1000) jitter (SCHED_OTHER)\n");
        printf("│    min: %.1f us  avg: %.1f us  max: %.1f us\n",
               min_ns / 1000.0, (sum_ns / N) / 1000.0, max_ns / 1000.0);
        printf("│    jitter: %.1f us\n\n", (max_ns - min_ns) / 1000.0);
    }

    /* ── LIVE STATUS ── */
    {
        uint32_t active = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        const feedback_frame_t* fb = &shm->fb_buffer[active];

        printf("┌─────────────────────────────────────────────┐\n");
        printf("│              LIVE SHM STATUS                │\n");
        printf("├─────────────────────────────────────────────┤\n");
        printf("│ active_idx       : %-4u                      │\n", active);
        printf("│ node_state       : %-4u (%s)\n",
               shm->node_state,
               shm->node_state == 0 ? "INIT" :
               shm->node_state == 2 ? "READY" :
               shm->node_state == 5 ? "RUNNING" :
               shm->node_state == 6 ? "FAULT" : "?");
        printf("│ motor_online     : 0x%02X                     │\n", shm->motor_online);
        printf("│ motor_enabled    : 0x%02X                     │\n", shm->motor_enabled);
        printf("│ motor_severity   : %d                          │\n", shm->motor_severity);
        printf("│ fault_reason     : %d                          │\n", shm->fault_reason);
        printf("│ mailbox.seq      : %lu                      │\n",
               (unsigned long)shm->mailbox.seq_begin);
        printf("├─────────────────────────────────────────────┤\n");
        printf("│ 延迟追踪                                   │\n");
        printf("│   avg total latency : %u us                  │\n",
               shm->avg_total_latency_us);
        printf("│   max total latency : %u us                  │\n",
               shm->max_total_latency_us);
        printf("│   cycle overruns    : %u                      │\n",
               shm->cycle_overrun_count);
        printf("├─────────────────────────────────────────────┤\n");

        /* 电机反馈 */
        for (int i = 0; i < EXO_MOTOR_COUNT; i++) {
            printf("│ Motor %d: pos=%6d vel=%4d cur=%4dmA temp=%3d  │\n",
                   i + 1,
                   fb->motor[i].position,
                   fb->motor[i].velocity,
                   fb->motor[i].current_iq,
                   fb->motor[i].temperature);
        }

        printf("│ IMU:   roll=%.2f pitch=%.2f yaw=%.2f         │\n",
               (double)fb->imu.roll,
               (double)fb->imu.pitch,
               (double)fb->imu.yaw);
        printf("│ Baro:  %.2f hPa, %.2f°C                      │\n",
               (double)fb->baro.pressure_hpa,
               (double)fb->baro.temperature_c);

        printf("├─────────────────────────────────────────────┤\n");
        printf("│ 延迟节点 (μs):                               │\n");
        printf("│   T0 can_rx     : %lu\n", (unsigned long)fb->ts_can_rx);
        printf("│   T1 cache_wr   : %lu\n", (unsigned long)fb->ts_cache_write);
        printf("│   T2 shm_read   : %lu\n", (unsigned long)fb->ts_shm_read);
        printf("│   T3 shm_write  : %lu\n", (unsigned long)fb->ts_shm_write);
        printf("│   T4 algo_read  : %lu\n", (unsigned long)fb->ts_algo_read);
        printf("│   T5 algo_done  : %lu\n", (unsigned long)fb->ts_algo_done);
        printf("│   frame asm     : %lu\n", (unsigned long)fb->ts_frame_assembly);
        printf("│   timestamp     : %lu\n", (unsigned long)fb->timestamp_us);
        printf("└─────────────────────────────────────────────┘\n");
    }

    munmap(shm, EXO_SHM_SIZE);
    close(fd);

    return 0;
}
