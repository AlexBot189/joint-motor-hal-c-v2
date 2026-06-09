/*
 * algo_sim.cpp — 模拟算法进程
 *
 * v3: 双电机同步控制, 延迟追踪.
 *
 * 行为:
 *   1. 打开 SHM /exo_shm
 *   2. 1KHz 循环: 读 feedback_frame_t → 简单控制 → 写 mailbox (双电机)
 *   3. 填充 T4/T5/T6 时间戳 (延迟追踪)
 *
 * 编译: g++ -O2 -o algo_sim algo_sim.cpp -I.. -lrt -lpthread
 * 运行: ./algo_sim
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include "exo_shm.h"

static volatile int running = 1;

static void sig_handler(int s)
{
    (void)s;
    running = 0;
    printf("\n[algo_sim] shutting down...\n");
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    int fd = shm_open(EXO_SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        perror("[algo_sim] shm_open: start stark_periph_manager_node first");
        return 1;
    }

    exo_shm_t* shm = (exo_shm_t*)mmap(NULL, EXO_SHM_SIZE,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED, fd, 0);
    if (shm == MAP_FAILED) {
        perror("[algo_sim] mmap");
        close(fd);
        return 1;
    }

    printf("[algo_sim] SHM at %p, starting 1KHz loop...\n", (void*)shm);

    uint64_t seq = 0;
    uint64_t loop_count = 0;

    while (running) {
        /* ════════════════════════════════════════════════════════
         * ① 读反馈帧 → T4 (算法读 SHM 完成)
         * ════════════════════════════════════════════════════════ */
        uint32_t active = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        feedback_frame_t* fb = &shm->fb_buffer[active];

        uint64_t t4_us = now_us();

        if (fb->timestamp_us > 0) {
            fb->ts_algo_read = t4_us;

            uint8_t online = __atomic_load_n(&shm->motor_online, __ATOMIC_ACQUIRE);

            /* ════════════════════════════════════════════════════
             * ② 步态计算 (模拟) → T5
             * ════════════════════════════════════════════════════ */

            /* 简单策略: 固定力矩 200mA 给所有在线电机 */
            if (online & 0x01) {
                shm->mailbox.cmd[0].motor_id     = 1;
                shm->mailbox.cmd[0].cmd          = EXO_CMD_TORQUE;
                shm->mailbox.cmd[0].value        = 200;
                shm->mailbox.cmd[0].timestamp_us = fb->timestamp_us;
            }

            if (online & 0x02) {
                shm->mailbox.cmd[1].motor_id     = 2;
                shm->mailbox.cmd[1].cmd          = EXO_CMD_TORQUE;
                shm->mailbox.cmd[1].value        = 200;
                shm->mailbox.cmd[1].timestamp_us = fb->timestamp_us;
            }

            uint64_t t5_us = now_us();
            fb->ts_algo_done = t5_us;

            /* ════════════════════════════════════════════════════
             * ③ 写 mailbox → T6
             * ════════════════════════════════════════════════════ */

            seq++;
            __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);

            uint64_t t6_us = now_us();
            shm->mailbox.seq_end = seq;
        }

        loop_count++;

        if ((loop_count % 1000) == 0) {
            printf("[algo_sim] loop=%lu | online=0x%02X | state=%u | seq=%lu | "
                   "latency: total=%uus max=%uus overruns=%u\n",
                   (unsigned long)loop_count,
                   shm->motor_online,
                   shm->node_state,
                   (unsigned long)seq,
                   shm->avg_total_latency_us,
                   shm->max_total_latency_us,
                   shm->cycle_overrun_count);
        }

        usleep(1000);
    }

    printf("[algo_sim] total loops: %lu\n", (unsigned long)loop_count);

    munmap(shm, EXO_SHM_SIZE);
    close(fd);

    return 0;
}
