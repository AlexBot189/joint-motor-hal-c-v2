/*
 * @file algo_sim.cpp
 * @brief 模拟算法进程 — 独立进程, 模拟真实步态算法的行为
 *
 * 行为:
 *   1. 打开已有 SHM /exo_shm (只读+写, 不创建)
 *   2. 1KHz 循环: 读 feedback_frame_t → 假装步态计算 → 写 mailbox
 *   3. 简单控制策略: 读到电机位置后, 对在线电机发固定力矩
 *
 * 编译: g++ -O2 -o algo_sim algo_sim.cpp -I.. -lrt -lpthread
 * 运行: ./algo_sim
 * 停止: Ctrl+C (SIGINT)
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "exo_shm.h"

static volatile int running = 1;

static void sig_handler(int s)
{
    (void)s;
    running = 0;
    printf("\n[algo_sim] caught signal, shutting down...\n");
}

int main()
{
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── 打开 SHM (不创建, 需要 stark_periph_manager_node 先运行) ── */
    int fd = shm_open(EXO_SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        perror("[algo_sim] shm_open: SHM not ready, start stark_periph_manager_node first");
        return 1;
    }

    exo_shm_t *shm = (exo_shm_t *)mmap(NULL, EXO_SHM_SIZE,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, 0);
    if (shm == MAP_FAILED) {
        perror("[algo_sim] mmap");
        close(fd);
        return 1;
    }

    printf("[algo_sim] SHM opened at %p, starting 1KHz loop...\n", (void *)shm);

    uint64_t seq = 0;
    uint64_t loop_count = 0;

    while (running) {
        /* ── 读反馈帧 (active_idx 用 acquire 语义) ── */
        uint32_t active = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        feedback_frame_t *fb = &shm->fb_buffer[active];

        if (fb->timestamp_us > 0) {
            /* ── 简单控制策略: 对在线电机发固定力矩 ── */
            uint8_t online = __atomic_load_n(&shm->motor_online, __ATOMIC_ACQUIRE);

            if (online & 0x01) {                    /* 电机1 (右髋) 在线 */
                shm->mailbox.cmd.motor_id     = 1;
                shm->mailbox.cmd.cmd          = EXO_CMD_TORQUE;
                shm->mailbox.cmd.value        = 200;          /* 200 mA */
                shm->mailbox.cmd.timestamp_us = fb->timestamp_us;
            }

            if (online & 0x02) {                    /* 电机2 (左髋) 在线 */
                shm->mailbox.cmd.motor_id     = 2;
                shm->mailbox.cmd.cmd          = EXO_CMD_TORQUE;
                shm->mailbox.cmd.value        = 200;
                shm->mailbox.cmd.timestamp_us = fb->timestamp_us;
            }

            /* ── 更新 seq → 写 mailbox 快照 ── */
            seq++;
            __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
            shm->mailbox.seq_end = seq;
        }

        loop_count++;

        /* 每秒打印一次统计 */
        if ((loop_count % 1000) == 0) {
            printf("[algo_sim] loop=%lu | active_idx=%u | motor_online=0x%02X | "
                   "node_state=%u | seq=%lu\n",
                   (unsigned long)loop_count,
                   shm->active_idx,
                   shm->motor_online,
                   shm->node_state,
                   (unsigned long)seq);
        }

        usleep(1000);   /* 1 KHz */
    }

    printf("[algo_sim] total loops: %lu\n", (unsigned long)loop_count);

    munmap(shm, EXO_SHM_SIZE);
    close(fd);

    return 0;
}
