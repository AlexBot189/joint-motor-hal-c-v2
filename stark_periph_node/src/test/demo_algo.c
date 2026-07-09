/*
 * demo_algo.c -- 外骨骼算法控制示例
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 用法 (PDO 连续控制, 算法无需 enable/set_mode):
 *   ./demo_algo torque <mA>            电流控制, 正弦波
 *   ./demo_algo speed <rpm>            速度控制 (CSV), 梯形波
 *   ./demo_algo pos <deg>              位置控制 (CSP), 方波
 *   ./demo_algo pp <deg> [acc] [vel]   轮廓位置 PP, 方波
 *   ./demo_algo pv <rpm> [acc]         轮廓速度 PV, 梯形波
 *   ./demo_algo mit <kp> <kd>          MIT 阻抗控制
 *   ./demo_algo multi <ma1> <ma2>      多轴广播, 恒电流
 *
 * SDO 单帧控制 (通过 mailbox → main_loop):
 *   ./demo_algo sdo cur <id> <mA>                    单电机电流
 *   ./demo_algo sdo cur <id1> <id2> <mA>              双电机同值电流
 *   ./demo_algo sdo cur <id1> <id2> <mA1> <mA2>       双电机不同值电流
 *   ./demo_algo sdo pos <id> <deg>                    单电机位置 (PP)
 *   ./demo_algo sdo pos <id1> <id2> <deg>             双电机同值位置
 *   ./demo_algo sdo pos <id1> <id2> <deg1> <deg2>     双电机不同值位置
 *   ./demo_algo sdo vel <id> <rpm>                    单电机速度 (PV)
 *   ./demo_algo sdo vel <id1> <id2> <rpm>             双电机同值速度
 *   ./demo_algo sdo vel <id1> <id2> <rpm1> <rpm2>     双电机不同值速度
 *
 * PDO 单帧控制 (通过 mailbox → RT 线程):
 *   ./demo_algo pdo cur <id> <mA>                    单电机电流
 *   ./demo_algo pdo cur <id1> <id2> <mA>              双电机同值电流
 *   ./demo_algo pdo cur <id1> <id2> <mA1> <mA2>       双电机不同值电流
 *   ./demo_algo pdo pos <id> <deg>                    单电机位置 (PP)
 *   ./demo_algo pdo pos <id1> <id2> <deg>             双电机同值位置
 *   ./demo_algo pdo pos <id1> <id2> <deg1> <deg2>     双电机不同值位置
 *   ./demo_algo pdo vel <id> <rpm>                    单电机速度 (PV)
 *   ./demo_algo pdo vel <id1> <id2> <rpm>             双电机同值速度
 *   ./demo_algo pdo vel <id1> <id2> <rpm1> <rpm2>     双电机不同值速度
 *
 * 管理/状态:
 *   ./demo_algo enable/disable/estop/clearf <id>
 *   ./demo_algo stat / report
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
    uint32_t rpt_ver = 0;

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float phase = (float)(t % 2000) / 2000.0f * 2.0f * M_PI;
        int32_t ma = (int32_t)((float)amplitude_ma * sinf(phase));

        stark_torque(c, 1, ma);
        stark_torque(c, 2, ma);

        if (ma != last) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%4lus] target=%5d mA  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), ma, counts_to_deg(fb.position), fb.current_iq);
            last = ma;
        }

        /* 周期上报数据: stark_report_try_read 一行获取 */
        {
            const PeriodicUploadData* d;
            if (stark_report_try_read(c, &rpt_ver, &d)) {
                printf("[rpt] cyc=%u ts=%u | "
                       "IMU roll=%.1f pitch=%.1f yaw=%.1f | "
                       "R: vel=%dRPM ang=%.1fdeg cur=%dmA temp=%.2fC "
                       "L: vel=%dRPM ang=%.1fdeg cur=%dmA temp=%.2fC | "
                       "S1: hall=%u,%u,%u tor=%u kne=%d "
                       "S2: hall=%u,%u,%u tor=%u kne=%d\n",
                       d->frame_cycle, d->imu_ts_us,
                       d->gyro_roll, d->gyro_pitch, d->gyro_yaw,
                       d->RealtimeVelocity, d->motor_abs_angle / 10.0f,
                       d->cal_Iq_current, d->motor_temp / 100.0f,
                       d->RealtimeVelocity_left, d->motor_abs_angle_left / 10.0f,
                       d->cal_Iq_current_left, d->motor_temp_left / 100.0f,
                       d->hall_a_data, d->hall_b_data, d->hall_c_data,
                       d->df181_torque, d->knee_hall,
                       d->hall_a_data_left, d->hall_b_data_left, d->hall_c_data_left,
                       d->df181_torque_left, d->knee_hall_left);
            }
        }

        stark_heartbeat(c);
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
        stark_heartbeat(c);
        usleep(5000);
    }
}

/* 位置控制: 方波 (PP 模式, 电机不支持 CSP) */
static void run_position(stark_client_t* c, float amplitude_deg)
{
    float accel = 2000.0f;
    float vel   = 10.0f;
    printf("[pos] 方波 (PP), ±%.1f deg, accel=%.0f vel=%.0f, 2s/拍\n", amplitude_deg, accel, vel);

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float target = ((t / 2000) % 2 == 0) ? amplitude_deg : -amplitude_deg;

        stark_pp(c, 1,  target, accel, vel);
        stark_pp(c, 2, -target, accel, vel);

        if (t % 500 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f deg  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), target,
                   counts_to_deg(fb.position), fb.current_iq);
        }
        stark_heartbeat(c);
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
        stark_heartbeat(c);
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
        stark_heartbeat(c);
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
        stark_heartbeat(c);
        usleep(5000);
    }
}

/* 多轴广播: 恒定电流 */
static void run_multi(stark_client_t* c, int32_t ma1, int32_t ma2)
{
    printf("[multi] 恒电流: M1=%d mA, M2=%d mA\n", ma1, ma2);

    while (g_running) {
        stark_torque(c, 1, ma1);
        stark_torque(c, 2, ma2);

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
        stark_heartbeat(c);
        usleep(1000);
    }
}

/* PeriodicUploadData display */
static void run_report_loop(stark_client_t* c)
{
    printf("[report] waiting for data stream...\n");

    while (!stark_report_data(c)) {
        usleep(100000);
    }

    uint32_t rpt_ver = 0;
    while (g_running) {
        const PeriodicUploadData* d;
        if (!stark_report_try_read(c, &rpt_ver, &d)) { usleep(1000); continue; }

        printf("=== [%ums] ver=%u frame=%u m_ts=%u i_ts=%u s_ts=%u ===\n",
               d->timestamp_ms, rpt_ver,
               d->frame_cycle, d->motor_ts_us, d->imu_ts_us, d->sensor_ts_us);

        /* IMU */
        printf("IMU  gyro(x=%.2f y=%.2f z=%.2f)dps  "
               "quat(w=%.4f x=%.4f y=%.4f z=%.4f)  "
               "euler(roll=%.1f pitch=%.1f yaw=%.1f)deg  "
               "acc(x=%.3f y=%.3f z=%.3f)g  press=%.1fhPa\n",
               d->gyro_dps_x, d->gyro_dps_y, d->gyro_dps_z,
               d->quat_w, d->quat_x, d->quat_y, d->quat_z,
               d->gyro_roll, d->gyro_pitch, d->gyro_yaw,
               d->acc_x, d->acc_y, d->acc_z, d->air_pressure);

        /* M1 */
        printf("M1   vel=%7dRPM  ang=%6.1fdeg  Iq=%5dmA  "
               "busI=%4dmA  temp=%6.2fC  fault=0x%04X  state=0x%02X\n",
               d->RealtimeVelocity,
               d->motor_abs_angle / 10.0f,
               d->cal_Iq_current,
               d->cal_bus_current * 10,
               d->motor_temp / 100.0f,
               d->fault_code, d->motor_state);

        /* M2 */
        printf("M2   vel=%7dRPM  ang=%6.1fdeg  Iq=%5dmA  "
               "busI=%4dmA  temp=%6.2fC  fault=0x%04X  state=0x%02X\n",
               d->RealtimeVelocity_left,
               d->motor_abs_angle_left / 10.0f,
               d->cal_Iq_current_left,
               d->cal_bus_current_left * 10,
               d->motor_temp_left / 100.0f,
               d->fault_code_left, d->motor_state_left);

        /* S1 */
        printf("S1   hall(a=%u b=%u c=%u)  torque=%u  knee=%d  land=%u  valid=%u\n",
               d->hall_a_data, d->hall_b_data, d->hall_c_data,
               d->df181_torque, d->knee_hall,
               d->key_landing, d->torque_valid);

        /* S2 */
        printf("S2   hall(a=%u b=%u c=%u)  torque=%u  knee=%d  land=%u  valid=%u\n\n",
               d->hall_a_data_left, d->hall_b_data_left, d->hall_c_data_left,
               d->df181_torque_left, d->knee_hall_left,
               d->key_landing_left, d->torque_valid_left);

        /* 0x6B0 力矩原始计数, 已并入 PeriodicUploadData, 直接从 d 取 */
        printf("SPI  M1[force_s24=%d valid=%u err=%u]  M2[force_s24=%d valid=%u err=%u]  (0x6B0 SPI力矩)\n",
               d->spi_force_raw_s24, d->spi_valid, d->spi_error,
               d->spi_force_raw_s24_left, d->spi_valid_left, d->spi_error_left);

        printf("\n");
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
    printf("PDO 连续控制 (算法无需 enable/set_mode):\n");
    printf("  torque <mA>           电流环, 正弦波\n");
    printf("  multi  <ma1> <ma2>    多轴广播, 恒电流\n");
    printf("  speed  <rpm>          速度控制 (CSV), 梯形波\n");
    printf("  csv    <rpm>          CSV 速度, 梯形波\n");
    printf("  pv     <rpm> [acc]    轮廓速度 PV, 梯形波\n");
    printf("  pos    <deg>          位置控制 (PP), 方波 (电机不支持 CSP)\n");
    printf("  csp    <deg>          PP 位置 (同 pos)\n");
    printf("  pp     <deg> [acc] [v] 轮廓位置 PP, 方波\n");
    printf("  mit    <kp> <kd>      MIT 阻抗\n");
    printf("\nSDO 单帧控制 (sdo cur/pos/vel, 支持单/双电机):\n");
    printf("  sdo cur <id> <mA>                    单电机电流\n");
    printf("  sdo cur <id1> <id2> <mA>             双电机同值电流\n");
    printf("  sdo cur <id1> <id2> <mA1> <mA2>      双电机不同值电流\n");
    printf("  sdo pos <id> <deg>                   单电机位置 (PP)\n");
    printf("  sdo pos <id1> <id2> <deg>            双电机同值位置 (PP)\n");
    printf("  sdo pos <id1> <id2> <deg1> <deg2>    双电机不同值位置 (PP)\n");
    printf("  sdo vel <id> <rpm>                   单电机速度 (PV)\n");
    printf("  sdo vel <id1> <id2> <rpm>            双电机同值速度 (PV)\n");
    printf("  sdo vel <id1> <id2> <rpm1> <rpm2>    双电机不同值速度 (PV)\n");
    printf("\nPDO 单帧控制 (pdo cur/pos/vel, 支持单/双电机):\n");
    printf("  pdo cur <id> <mA>                    单电机电流\n");
    printf("  pdo cur <id1> <id2> <mA>             双电机同值电流\n");
    printf("  pdo cur <id1> <id2> <mA1> <mA2>      双电机不同值电流\n");
    printf("  pdo pos <id> <deg> [acc] [vel]       单电机位置 (PP)\n");
    printf("  pdo pos <id1> <id2> <deg> [acc] [vel]双电机同值位置 (PP)\n");
    printf("  pdo pos <id1> <id2> <d1> <d2> [a] [v]双电机不同值位置 (PP)\n");
    printf("  pdo vel <id> <rpm> [acc]             单电机速度 (PV)\n");
    printf("  pdo vel <id1> <id2> <rpm> [acc]      双电机同值速度 (PV)\n");
    printf("  pdo vel <id1> <id2> <r1> <r2> [acc]  双电机不同值速度 (PV)\n");
    printf("\n管理命令:\n");
    printf("  enable  <id>          使能电机\n");
    printf("  disable <id>          失能电机\n");
    printf("  estop   <id>          急停\n");
    printf("  clearf  <id>          清故障\n");
    printf("  calib                 触发复杂校准 (按键/命令)\n");
    printf("\n状态:\n");
    printf("  stat                  只读反馈\n");
    printf("  report                周期上报数据\n");
    printf("\n示例:\n");
    printf("  ./demo_algo torque 200            # 电流 ±200mA 正弦波\n");
    printf("  ./demo_algo speed 10              # 速度 ±10RPM 梯形波\n");
    printf("  ./demo_algo pv 30 1000            # PV ±30RPM acc=1000\n");
    printf("  ./demo_algo pos 15                # 位置 ±15° 方波\n");
    printf("  ./demo_algo pp 15 2000 10         # PP ±15° 方波\n");
    printf("  ./demo_algo sdo cur 1 500         # SDO M1=500mA\n");
    printf("  ./demo_algo sdo cur 1 2 500       # SDO M1=M2=500mA\n");
    printf("  ./demo_algo sdo cur 1 2 500 300   # SDO M1=500 M2=300mA\n");
    printf("  ./demo_algo sdo pos 1 30          # SDO M1=30°\n");
    printf("  ./demo_algo sdo pos 1 2 30        # SDO M1=M2=30°\n");
    printf("  ./demo_algo sdo pos 1 2 30 20     # SDO M1=30° M2=20°\n");
    printf("  ./demo_algo sdo vel 1 10          # SDO M1=10RPM\n");
    printf("  ./demo_algo sdo vel 1 2 15        # SDO M1=M2=15RPM\n");
    printf("  ./demo_algo pdo cur 1 500         # PDO M1=500mA\n");
    printf("  ./demo_algo pdo cur 1 2 500 300   # PDO M1=500 M2=300mA\n");
    printf("  ./demo_algo pdo pos 1 30          # PDO M1=30° (PP)\n");
    printf("  ./demo_algo pdo pos 1 2 30 500 10 # PDO M1=M2=30° acc=500 vel=10\n");
    printf("  ./demo_algo pdo vel 1 10 500      # PDO M1=10RPM acc=500\n");
    printf("  ./demo_algo report                # 周期上报\n");
    printf("  ./demo_algo stat                  # 只读反馈\n");
}

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    if (argc < 2) { usage(); return 1; }

    const char* mode = argv[1];

    /* 连接 SHM (允许 stark_node 后启动) */
    stark_client_t c;
    printf("[init] 等待 stark_node...\n");
    while (stark_open(&c) != 0) {
        usleep(100000);
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

    /* stat/report/mgmt 模式不需要使能 */
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

    /* 管理命令: 直接执行后退出 */
    if (strcmp(mode, "enable") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_enable(&c, id);
        printf("motor %d enabled\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "disable") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_disable(&c, id);
        printf("motor %d disabled\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "estop") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_estop(&c, id);
        printf("motor %d emergency stopped\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "clearf") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_clear_fault(&c, id);
        printf("motor %d fault cleared\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "calib") == 0) {
        printf("Requesting complex calibration...\n");
        stark_request_calib(&c);
        printf("Calibration request sent. Waiting for completion...\n");
        while (!stark_ready(&c)) {
            int cs = stark_calib(&c);
            printf("  calib_state=%d (1=calibrating 2=done 3=timeout)\n", cs);
            usleep(500000);
        }
        printf("Calibration complete!\n");
        stark_close(&c);
        return 0;
    }

    /* ================================================================
     * SDO / PDO 单帧控制 (sdo cur/pos/vel, pdo cur/pos/vel)
     * 支持 4 种模式: 单电机 / 双电机同值 / 双电机不同值
     * ================================================================ */
    if (strcmp(mode, "sdo") == 0 || strcmp(mode, "pdo") == 0) {
        int is_pdo = (mode[0] == 'p');
        if (argc < 4) {
            printf("ERR: usage: %s cur/pos/vel <id> <val> [...]\n", mode);
            stark_close(&c); return 1;
        }
        const char* sub = argv[2];

        /* 解析电机 ID: argv[3]=id1, 若 argv[4] 为 1/2 则双电机 */
        int id1 = atoi(argv[3]);
        int id2 = 0, dual = 0;
        int val_idx = 4;
        if (argc > 5) {
            int m = atoi(argv[4]);
            if ((m == 1 || m == 2) && m != id1) {
                id2 = m;
                dual = 1;
                val_idx = 5;
            }
        }
        if (id1 < 1 || id1 > 2) { printf("ERR: invalid motor id=%d\n", id1); stark_close(&c); return 1; }

        if (strcmp(sub, "cur") == 0) {
            if (argc < val_idx + 1) { printf("ERR: need mA value\n"); stark_close(&c); return 1; }
            int ma1 = atoi(argv[val_idx]);
            int ma2 = (dual && argc >= val_idx + 2) ? atoi(argv[val_idx + 1]) : ma1;

            if (is_pdo) {
                stark_torque(&c, id1, ma1);
                if (dual) stark_torque(&c, id2, ma2);
                printf("PDO cur: M%d=%dmA", id1, ma1);
            } else {
                stark_sdo_cur(&c, id1, ma1);
                if (dual) stark_sdo_cur(&c, id2, ma2);
                printf("SDO cur: M%d=%dmA", id1, ma1);
            }
            if (dual) printf(" M%d=%dmA", id2, ma2);
            printf("\n");
            if (!is_pdo) {
                usleep(500000);  /* SDO: 等 enable(160ms) + mode + target + fb */
                motor_data_t fb = stark_fb(&c, id1);
                printf("  fb: pos=%.1fdeg cur=%dmA vel=%dRPM\n",
                       counts_to_deg(fb.position), fb.current_iq, fb.velocity);
            } else {
                usleep(100000);  /* PDO: 等 motor 使能 + 执行 + 反馈到达 */
                motor_data_t fb = stark_fb(&c, id1);
                printf("  fb: pos=%.1fdeg cur=%dmA vel=%dRPM\n",
                       counts_to_deg(fb.position), fb.current_iq, fb.velocity);
            }
            stark_close(&c); return 0;
        }

        if (strcmp(sub, "pos") == 0) {
            if (argc < val_idx + 1) { printf("ERR: need deg value\n"); stark_close(&c); return 1; }
            float deg1 = (float)atof(argv[val_idx]);
            float deg2 = (dual && argc >= val_idx + 2 && (atof(argv[val_idx + 1]) != 0.0f || argv[val_idx + 1][0] == '0'))
                         ? (float)atof(argv[val_idx + 1]) : deg1;
            /* dual diff 时有 2 个值后可能的 acc/vel */
            int opt_idx = dual ? val_idx + 2 : val_idx + 1;
            float acc = (argc > opt_idx) ? (float)atof(argv[opt_idx]) : 500.0f;
            float vel = (argc > opt_idx + 1) ? (float)atof(argv[opt_idx + 1]) : 10.0f;

            if (is_pdo) {
                stark_pp(&c, id1, deg1, acc, vel);
                if (dual) stark_pp(&c, id2, deg2, acc, vel);
                printf("PDO pos (PP): M%d=%.2f°", id1, deg1);
            } else {
                stark_sdo_pos(&c, id1, deg1, acc, vel);
                if (dual) stark_sdo_pos(&c, id2, deg2, acc, vel);
                printf("SDO pos (PP): M%d=%.2f°", id1, deg1);
            }
            if (dual) printf(" M%d=%.2f°", id2, deg2);
            printf(" accel=%.0f vel=%.0f\n", acc, vel);
            if (!is_pdo) {
                usleep(500000);  /* SDO: 等 enable(160ms) + mode + trajectory + fb */
                motor_data_t fb = stark_fb(&c, id1);
                printf("  fb: pos=%.1fdeg cur=%dmA vel=%dRPM\n",
                       counts_to_deg(fb.position), fb.current_iq, fb.velocity);
            } else {
                usleep(100000);  /* PDO: 等 motor 使能 + 执行 + 反馈到达 */
                motor_data_t fb = stark_fb(&c, id1);
                printf("  fb: pos=%.1fdeg cur=%dmA vel=%dRPM\n",
                       counts_to_deg(fb.position), fb.current_iq, fb.velocity);
            }
            stark_close(&c); return 0;
        }

        if (strcmp(sub, "vel") == 0) {
            if (argc < val_idx + 1) { printf("ERR: need rpm value\n"); stark_close(&c); return 1; }
            int rpm1 = atoi(argv[val_idx]);
            int rpm2 = (dual && argc >= val_idx + 2) ? atoi(argv[val_idx + 1]) : rpm1;
            int opt_idx = dual ? val_idx + 2 : val_idx + 1;
            int acc = (argc > opt_idx) ? atoi(argv[opt_idx]) : 500;

            if (is_pdo) {
                stark_pv(&c, id1, (float)rpm1, (float)acc);
                if (dual) stark_pv(&c, id2, (float)rpm2, (float)acc);
                printf("PDO vel (PV): M%d=%dRPM", id1, rpm1);
            } else {
                stark_sdo_vel(&c, id1, rpm1, acc);
                if (dual) stark_sdo_vel(&c, id2, rpm2, acc);
                printf("SDO vel (PV): M%d=%dRPM", id1, rpm1);
            }
            if (dual) printf(" M%d=%dRPM", id2, rpm2);
            printf(" accel=%d\n", acc);
            if (!is_pdo) {
                usleep(500000);  /* SDO: 等 enable(160ms) + mode + accel + vel + fb */
                motor_data_t fb = stark_fb(&c, id1);
                printf("  fb: pos=%.1fdeg cur=%dmA vel=%dRPM\n",
                       counts_to_deg(fb.position), fb.current_iq, fb.velocity);
            } else {
                usleep(100000);  /* PDO: 等 motor 使能 + 执行 + 反馈到达 */
                motor_data_t fb = stark_fb(&c, id1);
                printf("  fb: pos=%.1fdeg cur=%dmA vel=%dRPM\n",
                       counts_to_deg(fb.position), fb.current_iq, fb.velocity);
            }
            stark_close(&c); return 0;
        }

        printf("ERR: unknown %s sub-command: %s\n", mode, sub);
        stark_close(&c); return 1;
    }

    /* 分发 PDO 连续控制模式 */
    if (strcmp(mode, "torque") == 0) {
        if (argc < 3) { printf("ERR: 需要指定电流 mA\n"); stark_close(&c); return 1; }
        int32_t ma = atoi(argv[2]);
        run_torque(&c, ma);

    } else if (strcmp(mode, "speed") == 0 || strcmp(mode, "csv") == 0) {
        if (argc < 3) { printf("ERR: 需要指定速度 rpm\n"); stark_close(&c); return 1; }
        float rpm = (float)atof(argv[2]);
        run_speed(&c, rpm);

    } else if (strcmp(mode, "pos") == 0 || strcmp(mode, "csp") == 0) {
        if (argc < 3) { printf("ERR: 需要指定角度 deg\n"); stark_close(&c); return 1; }
        float deg = (float)atof(argv[2]);
        run_position(&c, deg);

    } else if (strcmp(mode, "pp") == 0) {
        if (argc < 3) { printf("ERR: 需要指定角度 deg\n"); stark_close(&c); return 1; }
        float deg   = (float)atof(argv[2]);
        float acc   = (argc >= 4) ? (float)atof(argv[3]) : 2000.0f;
        float vel   = (argc >= 5) ? (float)atof(argv[4]) : 10.0f;
        run_pp(&c, deg, acc, vel);

    } else if (strcmp(mode, "pv") == 0) {
        if (argc < 3) { printf("ERR: 需要指定速度 rpm\n"); stark_close(&c); return 1; }
        float rpm  = (float)atof(argv[2]);
        float acc  = (argc >= 4) ? (float)atof(argv[3]) : 1000.0f;
        run_pv(&c, rpm, acc);

    } else if (strcmp(mode, "mit") == 0) {
        if (argc < 4) { printf("ERR: 需要 kp kd\n"); stark_close(&c); return 1; }
        float kp = (float)atof(argv[2]);
        float kd = (float)atof(argv[3]);
        run_mit(&c, kp, kd);

    } else if (strcmp(mode, "multi") == 0) {
        if (argc < 4) { printf("ERR: 需要 ma1 ma2\n"); stark_close(&c); return 1; }
        int32_t ma1 = atoi(argv[2]);
        int32_t ma2 = atoi(argv[3]);
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
