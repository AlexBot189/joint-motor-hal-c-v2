/*
 * algo_sim.cpp — 模拟算法进程
 *
 * v4: Byte0控制 + 多控制模式示例.
 *
 * 控制流程:
 *   1. SDO startup (由 motor_node 完成) → DS402 in OP
 *   2. 算法发 EXO_CMD_ENABLE → pdo_byte0 bit7=1 → 电机开始响应PDO
 *   3. 算法发 EXO_CMD_SET_MODE → 切换控制模式
 *   4. 控制循环发 EXO_CMD_TORQUE/POS/SPEED/MIT → 只传target
 *   5. 急停: EXO_CMD_ESTOP
 *   6. 恢复: EXO_CMD_RECOVER
 *   7. 清错: 先读 fb->motor[i].error_code → 判断 → EXO_CMD_CLEAR_FAULT
 *
 * Byte0 字段说明:
 *   bit7: PDO使能 (EXO_CMD_ENABLE/DISABLE)
 *   bit6: 母线电压 (预留, 当前电机不实现机械抱闸)
 *   bit5: 清错脉冲 (EXO_CMD_CLEAR_FAULT, 上层先读 error_code 再决定)
 *   bit4:1: 控制模式 (EXO_CMD_SET_MODE)
 *
 * 编译: g++ -O2 -o algo_sim algo_sim.cpp -I.. -lrt -lpthread
 * 运行: ./algo_sim
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

/* motor_mode_t values (from motor_hal_types.h, 避免额外 include) */
#define ALGO_MODE_CURRENT   5   /* ALGO_MODE_CURRENT */
#define ALGO_MODE_CSP       3   /* ALGO_MODE_CSP */
#define ALGO_MODE_MIT       6   /* ALGO_MODE_MIT */

/* error_code bits (from motor_hal_types.h) */
#define ALGO_ERR_OVER_TEMP  0x0004  /* ALGO_ERR_OVER_TEMP */
#define ALGO_ERR_STALL      0x0008  /* ALGO_ERR_STALL */

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

/* helper: 快捷填充cmd字段 */
static void cmd_set(exo_shm_t *shm, int idx, uint8_t motor_id,
                    exo_cmd_type_t cmd, int32_t value,
                    int32_t value2, int32_t feedforward, uint64_t ts)
{
    shm->mailbox.cmd[idx].motor_id     = motor_id;
    shm->mailbox.cmd[idx].cmd          = (uint8_t)cmd;
    shm->mailbox.cmd[idx].value        = value;
    shm->mailbox.cmd[idx].value2       = value2;
    shm->mailbox.cmd[idx].feedforward  = feedforward;
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

    printf("[algo_sim] SHM at %p, starting 1KHz loop...\n", (void*)shm);

    uint64_t seq = 0;
    uint64_t loop_count = 0;
    uint64_t t0_us = now_us();

    /* ================================================================
     * Step 1: 等待 motor_node 完成 SDO startup (DS402 → OP)
     * 然后算法显式发 ENABLE → pdo_byte0 bit7=1
     * ================================================================ */
    printf("[algo_sim] waiting for motor startup...\n");

    /* ================================================================
     * Step 2: 设置 PDO 控制模式
     * ================================================================ */
    printf("[algo_sim] setting PDO mode...\n");

    while (running) {
        /* ── 读反馈 ── */
        uint32_t active = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        feedback_frame_t* fb = &shm->fb_buffer[active];
        uint64_t t4_us = now_us();

        if (fb->timestamp_us > 0) {
            fb->ts_algo_read = t4_us;

            uint8_t online = __atomic_load_n(&shm->motor_online, __ATOMIC_ACQUIRE);
            uint8_t state  = __atomic_load_n(&shm->node_state, __ATOMIC_ACQUIRE);
            float t_sec    = (float)(t4_us - t0_us) / 1e6f;

            /* ── STATE_RUNNING 时首次发送 enable + 模式 ── */
            static bool initialized = false;
            if (state == STATE_RUNNING && !initialized) {
                printf("[algo_sim] motors ready → PDO enable + set mode\n");
                /* 双电机: PDO使能 + 力矩模式 */
                cmd_set(shm, 0, 1, EXO_CMD_ENABLE,   0, 0, 0, 0);
                cmd_set(shm, 1, 2, EXO_CMD_ENABLE,   0, 0, 0, 0);
                seq++;
                __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                shm->mailbox.seq_end = seq;
                usleep(5000);

                cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, ALGO_MODE_CURRENT, 0, 0, 0);
                cmd_set(shm, 1, 2, EXO_CMD_SET_MODE, ALGO_MODE_CURRENT, 0, 0, 0);
                seq++;
                __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                shm->mailbox.seq_end = seq;
                usleep(5000);
                initialized = true;
            }

            if (!initialized) goto next_cycle;

            /* ============================================================
             * 控制循环 — 只传 target, Byte0 由 motor_node 内部维护
             *
             * 场景切换:
             *   0~3s:  力矩模式 200mA
             *   3~6s:  位置模式 正弦摆动 ±15°
             *   6~8s:  MIT 柔顺模式
             *   8s+:   错误检测示例
             * ============================================================ */

            if (t_sec < 3.0f) {
                /* ── 力矩控制 ── */
                if (online & 0x01) {
                    cmd_set(shm, 0, 1, EXO_CMD_TORQUE, 200, 0, 0, fb->timestamp_us);
                }
                if (online & 0x02) {
                    cmd_set(shm, 1, 2, EXO_CMD_TORQUE, 200, 0, 0, fb->timestamp_us);
                }

            } else if (t_sec < 6.0f) {
                /* ── 位置控制: 先切模式(一次), 再每帧发目标 ── */
                static bool pos_mode_set = false;
                if (!pos_mode_set) {
                    printf("[algo_sim] switching to POS mode\n");
                    cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, ALGO_MODE_CSP, 0, 0, 0);
                    cmd_set(shm, 1, 2, EXO_CMD_SET_MODE, ALGO_MODE_CSP, 0, 0, 0);
                    seq++;
                    __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                    shm->mailbox.seq_end = seq;
                    usleep(5000);
                    pos_mode_set = true;
                }

                float angle = 15.0f * sinf((t_sec - 3.0f) * 2.0f * M_PI);
                if (online & 0x01) {
                    cmd_set(shm, 0, 1, EXO_CMD_POS,
                            (int32_t)(angle * 100.0f),  0, 0, fb->timestamp_us);
                }
                if (online & 0x02) {
                    cmd_set(shm, 1, 2, EXO_CMD_POS,
                            (int32_t)(-angle * 100.0f), 0, 0, fb->timestamp_us);
                }

            } else if (t_sec < 8.0f) {
                /* ── MIT 阻抗控制: 低刚度柔顺 ── */
                static bool mit_mode_set = false;
                if (!mit_mode_set) {
                    printf("[algo_sim] switching to MIT mode\n");
                    cmd_set(shm, 0, 1, EXO_CMD_SET_MODE, ALGO_MODE_MIT, 0, 0, 0);
                    cmd_set(shm, 1, 2, EXO_CMD_SET_MODE, ALGO_MODE_MIT, 0, 0, 0);
                    seq++;
                    __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
                    shm->mailbox.seq_end = seq;
                    usleep(5000);
                    mit_mode_set = true;
                }

                /* MIT: 中间位置, 低刚度 (可以被推动) */
                if (online & 0x01) {
                    shm->mailbox.cmd[0].motor_id     = 1;
                    shm->mailbox.cmd[0].cmd          = EXO_CMD_MIT;
                    shm->mailbox.cmd[0].mit_pos      = 32768;   /* 中间 */
                    shm->mailbox.cmd[0].mit_vel      = 0;
                    shm->mailbox.cmd[0].mit_kp       = 50;      /* 低刚度 */
                    shm->mailbox.cmd[0].mit_kd       = 5;       /* 低阻尼 */
                    shm->mailbox.cmd[0].mit_torque   = 0;
                    shm->mailbox.cmd[0].timestamp_us = fb->timestamp_us;
                }
                if (online & 0x02) {
                    shm->mailbox.cmd[1].motor_id     = 2;
                    shm->mailbox.cmd[1].cmd          = EXO_CMD_MIT;
                    shm->mailbox.cmd[1].mit_pos      = 32768;
                    shm->mailbox.cmd[1].mit_vel      = 0;
                    shm->mailbox.cmd[1].mit_kp       = 50;
                    shm->mailbox.cmd[1].mit_kd       = 5;
                    shm->mailbox.cmd[1].mit_torque   = 0;
                    shm->mailbox.cmd[1].timestamp_us = fb->timestamp_us;
                }

            } else {
                /* ── bit5 清错示例: 先读 error_code, 再决定是否清除 ── */
                static bool fault_checked = false;
                if (!fault_checked) {
                    printf("[algo_sim] error check demo:\n");
                    for (int i = 0; i < EXO_MOTOR_COUNT; i++) {
                        uint8_t err = fb->motor[i].error_code;
                        printf("  motor[%d] error_code=0x%02X", i + 1, err);
                        if (err & ALGO_ERR_OVER_TEMP) {
                            printf(" (over temp: %.1f°C → waiting to cool)\n",
                                   (float)fb->motor[i].temperature / 10.0f);
                        } else if (err & ALGO_ERR_STALL) {
                            printf(" (stall → clearing via PDO bit5)\n");
                            if (shm->motor_online & (1 << i)) {
                                cmd_set(shm, i, (uint8_t)(i + 1),
                                        EXO_CMD_CLEAR_FAULT, 0, 0, 0, fb->timestamp_us);
                            }
                        } else if (err != 0) {
                            printf(" (generic → clearing via PDO bit5)\n");
                            if (shm->motor_online & (1 << i)) {
                                cmd_set(shm, i, (uint8_t)(i + 1),
                                        EXO_CMD_CLEAR_FAULT, 0, 0, 0, fb->timestamp_us);
                            }
                        } else {
                            printf(" (OK)\n");
                        }
                    }
                    fault_checked = true;
                }
            }

            /* ── 写 mailbox ── */
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

    /* ── 退出前急停 ── */
    printf("[algo_sim] ESTOP all motors...\n");
    cmd_set(shm, 0, 1, EXO_CMD_ESTOP, 0, 0, 0, 0);
    cmd_set(shm, 1, 2, EXO_CMD_ESTOP, 0, 0, 0, 0);
    seq++;
    __atomic_store_n(&shm->mailbox.seq_begin, seq, __ATOMIC_RELEASE);
    shm->mailbox.seq_end = seq;
    usleep(20000);

    printf("[algo_sim] total loops: %lu\n", (unsigned long)loop_count);

    munmap(shm, EXO_SHM_SIZE);
    close(fd);

    return 0;
}
