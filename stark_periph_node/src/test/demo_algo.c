/*
 * demo_algo.c -- 外骨骼算法控制示例
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 演示如何使用 stark_client.h 实现电机控制闭环.
 * 编译: gcc -O2 demo_algo.c -lpthread -lrt -lm -o demo_algo
 * 运行: sudo ./demo_algo [mode]
 *   mode: torque | speed | pos | mit | multi (默认 multi)
 */

#include "../stark_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

static volatile int g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ---- 辅助: 时间戳 (ms) ---- */
static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* ---- 辅助: 编码器 counts 转角度 ---- */
static float counts_to_deg(int16_t counts)
{
    return (float)counts * 360.0f / 65536.0f;
}

/* ---- 辅助: 编码器 counts 转 RPM (需要速度字段) ---- */
static float raw_to_rpm(int16_t velocity)
{
    return (float)velocity;
}

/* ---- (1) 力矩控制模式 ---- */
static void run_torque_control(stark_client_t* c)
{
    printf("[torque] 力矩控制模式: 正弦波扫描, 振幅=5000mA, 周期=2s\n");

    uint64_t t0 = now_ms();
    int32_t ma_prev[3] = {0, 0, 0};

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float phase = (float)(t % 2000) / 2000.0f * 2.0f * M_PI;
        int32_t ma = (int32_t)(5000.0f * sinf(phase));

        stark_multi(c, ma, 0, 0, ma, 0, 0);

        /* 每秒打印一次 */
        if (ma != ma_prev[1]) {
            motor_data_t fb = stark_fb(c, 1);
            float pos = counts_to_deg(fb.position);
            printf("[t=%4lus] target=%5d mA  fb_pos=%.1f deg  fb_cur=%d mA  temp=%.1f C\n",
                   (unsigned long)(t / 1000), ma, pos, fb.current_iq,
                   (float)fb.temperature * 0.1f);
            ma_prev[1] = ma;
        }

        usleep(1000);
    }
}

/* ---- (2) 速度控制模式 ---- */
static void run_speed_control(stark_client_t* c)
{
    printf("[speed] 速度控制模式: 梯形波, ±50RPM, 加速度50RPM/s\n");

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float rpm;

        /* 梯形波: 匀速扫描, ±50RPM, 每段1s */
        uint64_t phase = t % 4000;
        if (phase < 1000) {
            rpm = 50.0f * (float)phase / 1000.0f;
        } else if (phase < 2000) {
            rpm = 50.0f;
        } else if (phase < 3000) {
            rpm = 50.0f - 50.0f * (float)(phase - 2000) / 1000.0f;
        } else {
            rpm = 0.0f - 50.0f * (float)(phase - 3000) / 1000.0f;
        }

        stark_speed(c, 1, rpm);
        stark_speed(c, 2, rpm);

        motor_data_t fb = stark_fb(c, 1);
        if (t % 200 < 10) {
            printf("[t=%3lus] target=%.0f RPM  fb_vel=%d RPM  fb_pos=%.1f deg\n",
                   (unsigned long)(t / 1000), rpm, fb.velocity,
                   counts_to_deg(fb.position));
        }

        usleep(5000);  /* 200Hz */
    }
}

/* ---- (3) 位置控制模式 ---- */
static void run_position_control(stark_client_t* c)
{
    printf("[pos] 位置控制模式: 方波 ±30°, 2s/拍, CSP 模式\n");

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float target = ((t / 2000) % 2 == 0) ? 30.0f : -30.0f;

        stark_position(c, 1,  target);
        stark_position(c, 2, -target);  /* 左髋反向 */

        motor_data_t fb = stark_fb(c, 1);
        if (t % 500 < 10) {
            printf("[t=%3lus] target=%.0f deg  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), target,
                   counts_to_deg(fb.position), fb.current_iq);
        }

        usleep(1000);  /* 1KHz */
    }
}

/* ---- (4) MIT 阻抗控制 ---- */
static void run_mit_control(stark_client_t* c)
{
    printf("[MIT] MIT 阻抗控制模式: kp=200, kd=50, 零目标位置\n");

    while (g_running) {
        /* 零位置 + 低刚度 + 阻尼 */
        stark_mit(c, 1, 0.0f, 0.0f, 200.0f, 50.0f, 0.0f);
        stark_mit(c, 2, 0.0f, 0.0f, 200.0f, 50.0f, 0.0f);

        motor_data_t fb1 = stark_fb(c, 1);
        motor_data_t fb2 = stark_fb(c, 2);

        static int cnt = 0;
        if (++cnt % 50 == 0) {
            printf("[MIT] M1: pos=%.1f° cur=%dmA  M2: pos=%.1f° cur=%dmA\n",
                   counts_to_deg(fb1.position), fb1.current_iq,
                   counts_to_deg(fb2.position), fb2.current_iq);
        }

        usleep(1000);
    }
}

/* ---- (5) 多轴广播 + IMU 反馈 ---- */
static void run_multi_imu_control(stark_client_t* c)
{
    printf("[multi+imu] 多轴广播 + IMU 姿态监控\n");

    while (g_running) {
        /* 多轴广播: 双电机电流 0 (保持使能, 不发力) */
        stark_multi(c, 0, 0, 0, 0, 0, 0);

        motor_data_t fb1 = stark_fb(c, 1);
        motor_data_t fb2 = stark_fb(c, 2);
        imu_data_t   imu = stark_imu(c);

        static int cnt = 0;
        if (++cnt % 100 == 0) {
            printf("[multi] M1=%.1f° %dmA  M2=%.1f° %dmA  "
                   "IMU: yaw=%.1f° pitch=%.1f° roll=%.1f° "
                   "gyro=(%.1f,%.1f,%.1f) acc=(%.3f,%.3f,%.3f)\n",
                   counts_to_deg(fb1.position), fb1.current_iq,
                   counts_to_deg(fb2.position), fb2.current_iq,
                   imu.yaw, imu.pitch, imu.roll,
                   imu.gyro_x, imu.gyro_y, imu.gyro_z,
                   imu.acc_x, imu.acc_y, imu.acc_z);
        }

        usleep(1000);
    }
}

/* ---- 主函数 ---- */

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    const char* mode = (argc >= 2) ? argv[1] : "multi";

    /* (1) 打开 SHM */
    stark_client_t c;
    if (stark_open(&c) != 0) {
        printf("ERR: SHM 打开失败. stark_periph_manager_node 是否在运行?\n");
        printf("     预期 SHM: %s\n", STARK_SHM_NAME);
        return 1;
    }
    printf("[init] SHM 已连接: %s\n", STARK_SHM_NAME);

    /* (2) 等待就绪 (校准完成) */
    printf("[init] 等待电机就绪 (校准)...\n");
    while (!stark_ready(&c)) {
        if (stark_state(&c) == 3) {  /* FAULT */
            printf("ERR: 节点处于 FAULT 状态, 退出\n");
            stark_close(&c);
            return 1;
        }
        usleep(100000);  /* 100ms */
    }

    int online1 = stark_online(&c, 1);
    int online2 = stark_online(&c, 2);
    printf("[init] 就绪! 在线电机: %d %s %d %s\n",
           online1, online1 ? "在线" : "离线",
           online2, online2 ? "在线" : "离线");

    /* (3) 使能两个电机 */
    stark_enable(&c, 1);
    stark_enable(&c, 2);
    usleep(5000);  /* 等 Byte0 生效 */

    printf("[init] 电机已使能, 开始 %s 模式 (Ctrl+C 停止)\n\n", mode);

    /* (4) 运行控制循环 */
    if (strcmp(mode, "torque") == 0) {
        stark_set_mode(&c, 1, 5);  /* 电流模式 */
        stark_set_mode(&c, 2, 5);
        usleep(5000);
        run_torque_control(&c);
    } else if (strcmp(mode, "speed") == 0) {
        stark_set_mode(&c, 1, 2);  /* PV 模式 */
        stark_set_mode(&c, 2, 2);
        usleep(5000);
        run_speed_control(&c);
    } else if (strcmp(mode, "pos") == 0) {
        stark_set_mode(&c, 1, 3);  /* CSP 模式 */
        stark_set_mode(&c, 2, 3);
        usleep(5000);
        run_position_control(&c);
    } else if (strcmp(mode, "mit") == 0) {
        stark_set_mode(&c, 1, 6);  /* MIT 模式 */
        stark_set_mode(&c, 2, 6);
        usleep(5000);
        run_mit_control(&c);
    } else {
        /* 默认: multi + 力矩模式 */
        stark_set_mode(&c, 1, 5);
        stark_set_mode(&c, 2, 5);
        usleep(5000);
        run_multi_imu_control(&c);
    }

    /* (5) 清理 */
    printf("\n[done] 停止, 失能电机...\n");
    stark_estop(&c, 1);
    stark_estop(&c, 2);
    usleep(10000);
    stark_close(&c);
    printf("[done] 安全退出\n");

    return 0;
}
