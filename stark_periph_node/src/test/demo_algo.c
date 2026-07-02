/*
 * demo_algo.c -- 外骨骼算法控制示例
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 用法:
 *   ./demo_algo torque <mA>            电流控制, 正弦波, 振幅=mA
 *   ./demo_algo speed <rpm>            速度控制, 梯形波, 峰值=rpm
 *   ./demo_algo pos <deg>              位置控制, 方波, 振幅=deg
 *   ./demo_algo mit <kp> <kd>          MIT 阻抗控制
 *   ./demo_algo multi <ma1> <ma2>      多轴广播, 恒电流
 *   ./demo_algo stat                   只读反馈, 不发控制
 *
 * 编译: gcc -O2 demo_algo.c -lpthread -lrt -lm -o demo_algo
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

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static float counts_to_deg(int16_t counts)
{
    return (float)counts * 360.0f / 65536.0f;
}

/* 力矩控制: 正弦波扫描 */
static void run_torque(stark_client_t* c, int32_t amplitude_ma)
{
    printf("[torque] 正弦波扫描, 振幅=%d mA, 周期=2s\n", amplitude_ma);

    uint64_t t0 = now_ms();
    int32_t last = 0;

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float phase = (float)(t % 2000) / 2000.0f * 2.0f * M_PI;
        int32_t ma = (int32_t)((float)amplitude_ma * sinf(phase));

        stark_multi(c, ma, 0, 0, ma, 0, 0);

        if (ma != last) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%4lus] target=%5d mA  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), ma, counts_to_deg(fb.position), fb.current_iq);
            last = ma;
        }
        usleep(1000);
    }
}

/* 速度控制: 梯形波 */
static void run_speed(stark_client_t* c, float max_rpm)
{
    printf("[speed] 梯形波, ±%.0f RPM, 每段1s\n", max_rpm);

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float rpm;
        uint64_t phase = t % 4000;

        if (phase < 1000) {
            rpm = max_rpm * (float)phase / 1000.0f;
        } else if (phase < 2000) {
            rpm = max_rpm;
        } else if (phase < 3000) {
            rpm = max_rpm - max_rpm * (float)(phase - 2000) / 1000.0f;
        } else {
            rpm = 0.0f - max_rpm * (float)(phase - 3000) / 1000.0f;
        }

        stark_speed(c, 1, rpm);
        stark_speed(c, 2, rpm);

        if (t % 200 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f RPM  fb_vel=%d RPM  fb_pos=%.1f deg\n",
                   (unsigned long)(t / 1000), rpm, fb.velocity, counts_to_deg(fb.position));
        }
        usleep(5000);
    }
}

/* 位置控制: 方波 */
static void run_position(stark_client_t* c, float amplitude_deg)
{
    printf("[pos] 方波, ±%.1f deg, 2s/拍\n", amplitude_deg);

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float target = ((t / 2000) % 2 == 0) ? amplitude_deg : -amplitude_deg;

        stark_position(c, 1,  target);
        stark_position(c, 2, -target);

        if (t % 500 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f deg  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), target,
                   counts_to_deg(fb.position), fb.current_iq);
        }
        usleep(1000);
    }
}

/* MIT 阻抗控制 */
static void run_mit(stark_client_t* c, float kp, float kd)
{
    printf("[MIT] kp=%.0f, kd=%.0f, 零目标位置\n", kp, kd);

    while (g_running) {
        stark_mit(c, 1, 0.0f, 0.0f, kp, kd, 0.0f);
        stark_mit(c, 2, 0.0f, 0.0f, kp, kd, 0.0f);

        static int cnt = 0;
        if (++cnt % 50 == 0) {
            motor_data_t fb1 = stark_fb(c, 1);
            motor_data_t fb2 = stark_fb(c, 2);
            printf("[MIT] M1: pos=%.1f deg cur=%d mA  M2: pos=%.1f deg cur=%d mA\n",
                   counts_to_deg(fb1.position), fb1.current_iq,
                   counts_to_deg(fb2.position), fb2.current_iq);
        }
        usleep(1000);
    }
}

/* 轮廓位置 PP: 方波 */
static void run_pp(stark_client_t* c, float amplitude_deg, float accel, float vel)
{
    printf("[PP] 轮廓位置, ±%.1f deg, accel=%.0fRPM/s vel=%.0fRPM\n", amplitude_deg, accel, vel);

    uint64_t t0 = now_ms();
    while (g_running) {
        uint64_t t = now_ms() - t0;
        float target = ((t / 2000) % 2 == 0) ? amplitude_deg : -amplitude_deg;

        stark_pp(c, 1,  target, accel, vel);
        stark_pp(c, 2, -target, accel, vel);

        if (t % 500 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f deg  fb_pos=%.1f deg\n",
                   (unsigned long)(t / 1000), target, counts_to_deg(fb.position));
        }
        usleep(1000);
    }
}

/* 轮廓速度 PV: 梯形波 */
static void run_pv(stark_client_t* c, float max_rpm, float accel)
{
    printf("[PV] 轮廓速度, ±%.0f RPM, accel=%.0fRPM/s\n", max_rpm, accel);

    uint64_t t0 = now_ms();
    while (g_running) {
        uint64_t t = now_ms() - t0;
        float rpm;
        uint64_t phase = t % 4000;

        if (phase < 1000)
            rpm = max_rpm * (float)phase / 1000.0f;
        else if (phase < 2000)
            rpm = max_rpm;
        else if (phase < 3000)
            rpm = max_rpm - max_rpm * (float)(phase - 2000) / 1000.0f;
        else
            rpm = 0.0f - max_rpm * (float)(phase - 3000) / 1000.0f;

        stark_pv(c, 1,  rpm, accel);
        stark_pv(c, 2, -rpm, accel);

        if (t % 200 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f RPM  fb_vel=%d RPM\n",
                   (unsigned long)(t / 1000), rpm, fb.velocity);
        }
        usleep(5000);
    }
}

/* 多轴广播: 恒定电流 */
static void run_multi(stark_client_t* c, int32_t ma1, int32_t ma2)
{
    printf("[multi] 恒电流: M1=%d mA, M2=%d mA\n", ma1, ma2);

    while (g_running) {
        stark_multi(c, ma1, 0, 0, ma2, 0, 0);

        static int cnt = 0;
        if (++cnt % 200 == 0) {
            motor_data_t fb1 = stark_fb(c, 1);
            motor_data_t fb2 = stark_fb(c, 2);
            imu_data_t imu = stark_imu(c);
            printf("[multi] M1=%.1f deg %d mA  M2=%.1f deg %d mA  "
                   "IMU: yaw=%.1f pitch=%.1f roll=%.1f\n",
                   counts_to_deg(fb1.position), fb1.current_iq,
                   counts_to_deg(fb2.position), fb2.current_iq,
                   imu.yaw, imu.pitch, imu.roll);
        }
        usleep(1000);
    }
}

/* 周期上报数据打印 — 全字段 */
static void run_report_loop(stark_client_t* c)
{
    printf("[report] 等待数据流开启...\n");

    while (!stark_report_data(c)) {
        usleep(100000);
    }

    while (g_running) {
        uint64_t t0 = now_ms();
        const PeriodicUploadData* d = stark_report_data(c);
        if (!d) { usleep(1000); continue; }

        printf("=== [%ums] ==========================================\n", d->timestamp_ms);

        /* IMU */
        printf("IMU  gyro(%.2f %.2f %.2f) dps  quat(%.4f %.4f %.4f %.4f)  "
               "euler(%.1f %.1f %.1f)°  acc(%.3f %.3f %.3f)g  press(%.1f)hPa\n",
               d->gyro_dps_x, d->gyro_dps_y, d->gyro_dps_z,
               d->quat_w, d->quat_x, d->quat_y, d->quat_z,
               d->gyro_roll, d->gyro_pitch, d->gyro_yaw,
               d->acc_x, d->acc_y, d->acc_z, d->air_pressure);

        /* M1 */
        printf("M1    vel=%6.1fRPM  ang=%6.1f°  Iq=%6.2fA  busI=%5.2fA  "
               "temp=%5.2f°C  fault=0x%04X  state=0x%02X\n",
               d->RealtimeVelocity / 10.0f,
               d->motor_abs_angle / 10.0f,
               d->cal_Iq_current / 100.0f,
               d->cal_bus_current / 100.0f,
               d->motor_temp / 100.0f,
               d->fault_code, d->motor_state);

        /* M2 */
        printf("M2    vel=%6.1fRPM  ang=%6.1f°  Iq=%6.2fA  busI=%5.2fA  "
               "temp=%5.2f°C  fault=0x%04X  state=0x%02X\n",
               d->RealtimeVelocity_left / 10.0f,
               d->motor_abs_angle_left / 10.0f,
               d->cal_Iq_current_left / 100.0f,
               d->cal_bus_current_left / 100.0f,
               d->motor_temp_left / 100.0f,
               d->fault_code_left, d->motor_state_left);

        /* S1 */
        printf("S1    Hall(%u %u %u)  torque=%u  knee=%d  land=%u  valid=%u\n",
               d->hall_a_data, d->hall_b_data, d->hall_c_data,
               d->df181_torque, d->knee_angle,
               d->key_landing, d->torque_valid);

        /* S2 */
        printf("S2    Hall(%u %u %u)  torque=%u  knee=%d  land=%u  valid=%u\n",
               d->hall_a_data_left, d->hall_b_data_left, d->hall_c_data_left,
               d->df181_torque_left, d->knee_angle_left,
               d->key_landing_left, d->torque_valid_left);

        printf("\n");

        /* 每秒打印一次 */
        uint64_t elapsed = now_ms() - t0;
        usleep(elapsed < 900 ? (unsigned int)(1000 - elapsed) : 100000);
    }
}

/* 只读反馈, 不发控制 */
static void run_stat_loop(stark_client_t* c)
{
    printf("[stat] 只读反馈, 不发控制命令\n");

    while (g_running) {
        motor_data_t fb1 = stark_fb(c, 1);
        motor_data_t fb2 = stark_fb(c, 2);
        imu_data_t imu = stark_imu(c);

        printf("[stat] M1: pos=%.1f deg vel=%d RPM cur=%d mA temp=%.1f C  "
               "M2: pos=%.1f deg vel=%d RPM cur=%d mA  "
               "IMU: yaw=%.1f pitch=%.1f roll=%.1f\n",
               counts_to_deg(fb1.position), fb1.velocity, fb1.current_iq, (float)fb1.temperature * 0.1f,
               counts_to_deg(fb2.position), fb2.velocity, fb2.current_iq,
               imu.yaw, imu.pitch, imu.roll);

        usleep(200000);  /* 5Hz */
    }
}

static void usage(void)
{
    printf("用法: ./demo_algo <mode> [args...]\n\n");
    printf("模式:\n");
    printf("  torque <mA>           电流环, 正弦波, 振幅=mA\n");
    printf("  pp     <deg> [acc] [v] 轮廓位置 PP, 方波\n");
    printf("  pv     <rpm> [acc]      轮廓速度 PV, 梯形波\n");
    printf("  csp    <deg>           CSP 位置, 方波\n");
    printf("  csv    <rpm>           CSV 速度, 梯形波\n");
    printf("  speed  <rpm>           速度控制 (同 csv)\n");
    printf("  pos    <deg>           位置控制 (同 csp)\n");
    printf("  mit    <kp> <kd>       MIT 阻抗\n");
    printf("  multi  <ma1> <ma2>     多轴广播, 恒电流\n");
    printf("  stat                   只读反馈\n");
    printf("  report                 周期上报数据打印\n");
    printf("\n示例:\n");
    printf("  ./demo_algo torque 200        # 电流 ±200mA\n");
    printf("  ./demo_algo pp 15 2000 10     # PP ±15°, acc=2000RPM/s vel=10RPM\n");
    printf("  ./demo_algo pv 30 1000        # PV ±30RPM, acc=1000RPM/s\n");
    printf("  ./demo_algo csp 15            # CSP ±15°\n");
    printf("  ./demo_algo csv 10            # CSV ±10RPM\n");
    printf("  ./demo_algo mit 100 50        # MIT kp=100 kd=50\n");
    printf("  ./demo_algo multi 200 200    # 双电机各 200mA\n");
    printf("  ./demo_algo stat             # 只读反馈\n");
}

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    if (argc < 2) { usage(); return 1; }

    const char* mode = argv[1];

    /* 连接 SHM */
    stark_client_t c;
    if (stark_open(&c) != 0) {
        printf("ERR: SHM 连接失败\n");
        return 1;
    }
    printf("[init] SHM 已连接\n");

    /* 等待就绪 */
    printf("[init] 等待电机就绪...\n");
    while (!stark_ready(&c)) {
        if (stark_state(&c) == 3) {
            printf("ERR: FAULT 状态, 退出\n");
            stark_close(&c);
            return 1;
        }
        usleep(100000);
    }
    printf("[init] 就绪, 电机在线: %d %d\n", stark_online(&c, 1), stark_online(&c, 2));

    /* stat/report 模式不需要使能 */
    if (strcmp(mode, "stat") == 0) {
        run_stat_loop(&c);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "report") == 0) {
        run_report_loop(&c);
        stark_close(&c);
        return 0;
    }

    /* 使能双电机 */
    stark_enable(&c, 1);
    stark_enable(&c, 2);
    usleep(5000);

    /* 分发模式 */
    if (strcmp(mode, "torque") == 0) {
        if (argc < 3) { printf("ERR: 需要指定电流 mA\n"); stark_close(&c); return 1; }
        int32_t ma = atoi(argv[2]);
        stark_set_mode(&c, 1, 5); stark_set_mode(&c, 2, 5);
        usleep(5000);
        run_torque(&c, ma);

    } else if (strcmp(mode, "speed") == 0 || strcmp(mode, "csv") == 0) {
        if (argc < 3) { printf("ERR: 需要指定速度 rpm\n"); stark_close(&c); return 1; }
        float rpm = (float)atof(argv[2]);
        stark_set_mode(&c, 1, 4); stark_set_mode(&c, 2, 4);
        usleep(5000);
        run_speed(&c, rpm);

    } else if (strcmp(mode, "pos") == 0 || strcmp(mode, "csp") == 0) {
        if (argc < 3) { printf("ERR: 需要指定角度 deg\n"); stark_close(&c); return 1; }
        float deg = (float)atof(argv[2]);
        stark_set_mode(&c, 1, 3); stark_set_mode(&c, 2, 3);
        usleep(5000);
        run_position(&c, deg);

    } else if (strcmp(mode, "pp") == 0) {
        if (argc < 3) { printf("ERR: 需要指定角度 deg\n"); stark_close(&c); return 1; }
        float deg   = (float)atof(argv[2]);
        float acc   = (argc >= 4) ? (float)atof(argv[3]) : 2000.0f;
        float vel   = (argc >= 5) ? (float)atof(argv[4]) : 10.0f;
        stark_set_mode(&c, 1, 1); stark_set_mode(&c, 2, 1);
        usleep(5000);
        run_pp(&c, deg, acc, vel);

    } else if (strcmp(mode, "pv") == 0) {
        if (argc < 3) { printf("ERR: 需要指定速度 rpm\n"); stark_close(&c); return 1; }
        float rpm  = (float)atof(argv[2]);
        float acc  = (argc >= 4) ? (float)atof(argv[3]) : 1000.0f;
        stark_set_mode(&c, 1, 2); stark_set_mode(&c, 2, 2);
        usleep(5000);
        run_pv(&c, rpm, acc);

    } else if (strcmp(mode, "mit") == 0) {
        if (argc < 4) { printf("ERR: 需要 kp kd\n"); stark_close(&c); return 1; }
        float kp = (float)atof(argv[2]);
        float kd = (float)atof(argv[3]);
        stark_set_mode(&c, 1, 6); stark_set_mode(&c, 2, 6);
        usleep(5000);
        run_mit(&c, kp, kd);

    } else if (strcmp(mode, "multi") == 0) {
        if (argc < 4) { printf("ERR: 需要 ma1 ma2\n"); stark_close(&c); return 1; }
        int32_t ma1 = atoi(argv[2]);
        int32_t ma2 = atoi(argv[3]);
        stark_set_mode(&c, 1, 5); stark_set_mode(&c, 2, 5);
        usleep(5000);
        run_multi(&c, ma1, ma2);

    } else {
        printf("未知模式: %s\n", mode);
        usage();
    }

    printf("\n[done] 停止, 失能电机...\n");
    stark_estop(&c, 1);
    stark_estop(&c, 2);
    usleep(10000);
    stark_close(&c);
    return 0;
}
