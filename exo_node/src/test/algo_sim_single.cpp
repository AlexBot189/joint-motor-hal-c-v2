/*
 * algo_sim_single.cpp — 单电机模拟算法进程
 *
 * ★ TEMP: 仅用于单电机联调测试, 推生产前恢复双电机版 algo_sim.cpp
 *
 * 启动: ./algo_sim_single
 * 编译: g++ -O2 -o algo_sim_single algo_sim_single.cpp -I.. -lrt -lpthread
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include "exo_shm.h"

#define ALGO_MODE_CURRENT   5
#define ALGO_MODE_CSP       3
#define ALGO_MODE_MIT       6

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

static void cmd_set(exo_shm_t *shm, int idx, uint8_t motor_id,
                    exo_cmd_type_t cmd, int32_t value,
                    int32_t value2, int32_t feedforward, uint64_t ts)
{
    shm->mailbox.cmd[idx].motor_id     = motor_id;
    shm->mailbox.cmd[idx].cmd          = (uint8_t)cmd;
    shm->mailbox.cmd[idx].value        = value;
    shm->mailbox.cmd[idx].value2       = value2;
    shm->mailbox.cmd[idx].feedforward  = feedforward;
    shm->mailbox.cmd[idx].mit_pos      = 0;
    shm->mailbox.cmd[idx].mit_vel      = 0;
    shm->mailbox.cmd[idx].mit_kp       = 0;
    shm->mailbox.cmd[idx].mit_kd       = 0;
    shm->mailbox.cmd[idx].mit_torque   = 0;
    shm->mailbox.cmd[idx].timestamp_us = ts;
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

    printf("[algo_sim] SHM at %p, single motor mode\n", (void*)shm);

    uint64_t seq = 0;
    uint64_t loop_count = 0;
    uint64_t t0_us = now_us();

    printf("[algo_sim] waiting for motor startup...\n");

    while (running) {
        uint32_t active = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        feedback_frame_t* fb = &shm->fb_buffer[active];
        uint64_t t4_us = now_us();

        if (fb->timestamp_us > 0) {
            fb->ts_algo_read = t4_us;

            uint8_t online = __atomic_load_n(&shm->motor_online, __ATOMIC_ACQUIRE);
            uint8_t state  = __atomic_load_n(&shm->node_state, __ATOMIC_ACQUIRE);
            float t_sec    = (float)(t4_us - t0_us) / 1e6f;

            static bool initialized = false;
            /* FAULT 后状态恢复 → 重新初始化 */
            if (state == STATE_FAULT) initialized = false;
            /* READY 状态即可发初始化命令 (ENABLE + SET_MODE), 不等 RUNNING */
            if ((state >= STATE_READY) && !initialized) {
                printf("[algo_sim] state=%u → PDO enable + set mode\n", state);
                cmd_set(shm, 0, 1, EXO_CMD_ENABLE,   0, 0, 0, 0);
                seq++;
                __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                shm->mailbox.seq_end = seq;
                usleep(5000);

                cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, ALGO_MODE_CURRENT, 0, 0, 0);
                seq++;
                __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                shm->mailbox.seq_end = seq;
                usleep(5000);
                initialized = true;
            }

            if (!initialized) goto next_cycle;
            /* 等 RUNNING 后再发控制命令 */
            if (state != STATE_RUNNING) goto next_cycle;

            /* ── 单电机控制场景 ── */
            if (t_sec < 3.0f) {
                /* 力矩模式 800mA */
                cmd_set(shm, 0, 1, EXO_CMD_TORQUE, 800, 0, 0, fb->timestamp_us);

            } else if (t_sec < 6.0f) {
                /* 位置模式 正弦 ±10° */
                static bool pos_mode_set = false;
                if (!pos_mode_set) {
                    printf("[algo_sim] switching to POS mode\n");
                    cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, ALGO_MODE_CSP, 0, 0, 0);
                    seq++;
                    __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                    shm->mailbox.seq_end = seq;
                    usleep(5000);
                    pos_mode_set = true;
                }
                float angle = 10.0f * sinf((t_sec - 3.0f) * 2.0f * M_PI);
                cmd_set(shm, 0, 1, EXO_CMD_POS,
                        (int32_t)(angle * 100.0f), 0, 0, fb->timestamp_us);

            } else if (t_sec < 8.0f) {
                /* MIT 柔顺 */
                static bool mit_mode_set = false;
                if (!mit_mode_set) {
                    printf("[algo_sim] switching to MIT mode\n");
                    cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, ALGO_MODE_MIT, 0, 0, 0);
                    seq++;
                    __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                    shm->mailbox.seq_end = seq;
                    usleep(5000);
                    mit_mode_set = true;
                }
                shm->mailbox.cmd[0].motor_id     = 1;
                shm->mailbox.cmd[0].cmd          = EXO_CMD_MIT;
                shm->mailbox.cmd[0].mit_pos      = 32768;
                shm->mailbox.cmd[0].mit_vel      = 0;
                shm->mailbox.cmd[0].mit_kp       = 50;
                shm->mailbox.cmd[0].mit_kd       = 5;
                shm->mailbox.cmd[0].mit_torque   = 0;
                shm->mailbox.cmd[0].timestamp_us = fb->timestamp_us;

            } else {
                /* 错误检测 */
                uint8_t err = fb->motor[0].error_code;
                if ((loop_count % 1000) == 0) {
                    printf("[algo_sim] motor1 error=0x%02X temp=%d pos=%d cur=%dmA\n",
                           err, fb->motor[0].temperature,
                           fb->motor[0].position, fb->motor[0].current_iq);
                }
            }

            uint64_t t5_us = now_us();
            fb->ts_algo_done = t5_us;

            seq++;
            __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
            shm->mailbox.seq_end = seq;
        }

next_cycle:
        loop_count++;

        if ((loop_count % 1000) == 0) {
            printf("[algo_sim] loop=%lu | online=0x%02X | state=%u | seq=%lu | "
                   "latency: fb_avg=%uus fb_max=%uus ctrl_avg=%uus overruns=%u\n",
                   (unsigned long)loop_count,
                   shm->motor_online,
                   shm->node_state,
                   (unsigned long)seq,
                   shm->fb_total_avg_us,
                   shm->fb_total_max_us,
                   shm->ctrl_total_avg_us,
                   shm->cycle_overrun_count);
        }

        usleep(1000);
    }

    /* 退出急停 */
    printf("[algo_sim] ESTOP motor 1...\n");
    cmd_set(shm, 0, 1, EXO_CMD_ESTOP, 0, 0, 0, 0);
    seq++;
    __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
    shm->mailbox.seq_end = seq;
    usleep(20000);

    printf("[algo_sim] total loops: %lu\n", (unsigned long)loop_count);

    munmap(shm, EXO_SHM_SIZE);
    close(fd);

    return 0;
}
