/*
 * stark_tool.c — 电机调试 CLI 工具
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 用法:
 *   直接命令 (快速调试, 直连 SHM):
 *     stark_tool torque 1 2000      电机1 电流=2000mA
 *     stark_tool speed  2 50        电机2 速度=50RPM
 *     stark_tool abs    1 45        电机1 绝对位置=45°
 *     stark_tool rel    1 10        电机1 相对+10°
 *     stark_tool enable  2          电机2 使能
 *     stark_tool disable 1          电机1 失能
 *     stark_tool estop   1          电机1 急停
 *     stark_tool recover 1          电机1 恢复
 *     stark_tool clearf  1          电机1 清零故障
 *     stark_tool mode    1 5        电机1 设电流模式
 *     stark_tool multi 2000 0 0 2000 0 0  双电机各2000mA
 *     stark_tool mit 1 45 10 500 100 0    MIT阻抗
 *     stark_tool stat              查看状态
 *     stark_tool watch [period_ms] 持续打印反馈 (默认200ms)
 *     stark_tool stop              停止所有电机
 *
 *   Daemon 模式 (Unix Socket):
 *     stark_tool daemon             启动 /tmp/stark_cmd.sock
 *     另一终端: echo "torque 1 2000" | nc -U /tmp/stark_cmd.sock
 *
 * 编译: gcc -O2 stark_tool.c -lpthread -lrt -o stark_tool
 */

#include "stark_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>

#define SOCK_PATH "/tmp/stark_cmd.sock"

/* ================================================================
 * 帮助
 * ================================================================ */

static void usage(void)
{
    printf("stark_tool — 电机调试 CLI\n\n");
    printf("直接命令:\n");
    printf("  torque  <id> <mA>           电流控制 (%d-%d mA)\n", -20000, 20000);
    printf("  speed   <id> <rpm>          速度控制 (RPM)\n");
    printf("  abs     <id> <deg>          绝对位置控制 (°)\n");
    printf("  rel     <id> <delta_deg>    相对位置控制 (°)\n");
    printf("  csp     <id> <deg>          循环同步位置 (°)\n");
    printf("  pp      <id> <deg> <acc> <v>轮廓位置 (deg, RPM/s, RPM)\n");
    printf("  multi   <t1> <v1> <p1> <t2> <v2> <p2>  多轴广播\n");
    printf("  mit     <id> <pos> <vel> <kp> <kd> <tor>  MIT阻抗\n\n");
    printf("管理命令:\n");
    printf("  enable   <id>               使能\n");
    printf("  disable  <id>               失能\n");
    printf("  estop    <id>               急停\n");
    printf("  recover  <id>               恢复\n");
    printf("  clearf   <id>               清零故障\n");
    printf("  mode     <id> <mode>        设控制模式 (5=电流 3=CSP 4=CSV 2=PV 1=PP)\n");
    printf("  stop                        失能全部电机\n\n");
    printf("状态:\n");
    printf("  stat                        查看电机状态\n");
    printf("  watch [period_ms]           持续打印反馈 (默认 200ms)\n\n");
    printf("Daemon:\n");
    printf("  daemon                      启动 Unix Socket 服务\n\n");
}

/* ================================================================
 * stat — 打印状态
 * ================================================================ */

static void print_stat(stark_client_t* c)
{
    printf("State: %d  ", stark_state(c));
    switch (stark_state(c)) {
    case 0: printf("BOOTING"); break;
    case 1: printf("READY"); break;
    case 2: printf("RUNNING"); break;
    case 3: printf("FAULT"); break;
    default: printf("UNKNOWN");
    }

    printf("  Calib: %d (", stark_calib(c));
    switch (stark_calib(c)) {
    case 0: printf("idle"); break;
    case 1: printf("running"); break;
    case 2: printf("done"); break;
    case 3: printf("timeout"); break;
    default: printf("?");
    }
    printf(")  Severity: %d  Fault: %d",
           stark_severity(c), stark_fault_reason(c));

    printf("\nMotor: ");
    for (int i = 1; i <= 2; i++) {
        if (stark_online(c, i)) {
            motor_data_t fb = stark_fb(c, i);
            float deg = (float)fb.position * 360.0f / 65536.0f;
            printf("[%d] pos=%.1f° vel=%dRPM cur=%dmA temp=%.1f°C status=0x%02X  ",
                   i, deg, fb.velocity, fb.current_iq,
                   (float)fb.temperature * 0.1f, fb.status_byte);
        } else {
            printf("[%d] OFFLINE  ", i);
        }
    }
    printf("\n");
}

/* ================================================================
 * watch — 持续打印反馈
 * ================================================================ */

static volatile int g_watch_running = 1;

static void watch_sig(int sig) { (void)sig; g_watch_running = 0; }

static void cmd_watch(stark_client_t* c, int period_ms)
{
    signal(SIGINT, watch_sig);
    printf("Watching feedback every %dms (Ctrl+C to stop)\n\n", period_ms);

    printf("%8s %6s %6s %6s %6s %6s %6s %8s\n",
           "time", "M1pos°", "M1vel", "M1mA", "M2pos°", "M2vel", "M2mA", "St/Calib");

    while (g_watch_running) {
        motor_data_t fb1 = stark_fb(c, 1);
        motor_data_t fb2 = stark_fb(c, 2);

        float deg1 = (float)fb1.position * 360.0f / 65536.0f;
        float deg2 = (float)fb2.position * 360.0f / 65536.0f;

        printf("%8d %6.1f %6d %6d %6.1f %6d %6d %d/%d\n",
               stark_state(c), deg1, fb1.velocity, fb1.current_iq,
               deg2, fb2.velocity, fb2.current_iq,
               stark_state(c), stark_calib(c));

        usleep(period_ms * 1000);
    }
}

/* ================================================================
 * stop — 失能全部电机
 * ================================================================ */

static void cmd_stop(stark_client_t* c)
{
    stark_estop(c, 1);
    stark_estop(c, 2);
    printf("All motors ESTOP\n");
}

/* ================================================================
 * 命令分发
 * ================================================================ */

static int dispatch(stark_client_t* c, int argc, char** argv)
{
    if (argc < 2) { usage(); return 1; }

    const char* cmd = argv[1];

    if (strcmp(cmd, "torque") == 0) {
        if (argc < 4) { printf("Usage: torque <id> <mA>\n"); return 1; }
        int id = atoi(argv[2]);
        int ma = atoi(argv[3]);
        stark_torque(c, id, ma);
        printf("Motor %d: torque = %d mA\n", id, ma);

    } else if (strcmp(cmd, "speed") == 0) {
        if (argc < 4) { printf("Usage: speed <id> <rpm>\n"); return 1; }
        int id = atoi(argv[2]);
        float rpm = (float)atof(argv[3]);
        stark_speed(c, id, rpm);
        printf("Motor %d: speed = %.1f RPM\n", id, rpm);

    } else if (strcmp(cmd, "abs") == 0) {
        if (argc < 4) { printf("Usage: abs <id> <deg>\n"); return 1; }
        int id = atoi(argv[2]);
        float deg = (float)atof(argv[3]);
        stark_position(c, id, deg);
        printf("Motor %d: abs position = %.1f°\n", id, deg);

    } else if (strcmp(cmd, "rel") == 0) {
        if (argc < 4) { printf("Usage: rel <id> <delta_deg>\n"); return 1; }
        int id = atoi(argv[2]);
        float delta = (float)atof(argv[3]);
        stark_rel_position(c, id, delta);
        printf("Motor %d: relative +%.1f°\n", id, delta);

    } else if (strcmp(cmd, "csp") == 0) {
        if (argc < 4) { printf("Usage: csp <id> <deg>\n"); return 1; }
        int id = atoi(argv[2]);
        float deg = (float)atof(argv[3]);
        stark_position(c, id, deg);  /* CSP 同 POS */
        printf("Motor %d: CSP = %.1f°\n", id, deg);

    } else if (strcmp(cmd, "pp") == 0) {
        if (argc < 6) { printf("Usage: pp <id> <deg> <accel> <vel>\n"); return 1; }
        int id = atoi(argv[2]);
        float deg = (float)atof(argv[3]);
        float acc = (float)atof(argv[4]);
        float vel = (float)atof(argv[5]);
        stark_pp(c, id, deg, acc, vel);
        printf("Motor %d: PP %.1f° acc=%.1f vel=%.1f\n", id, deg, acc, vel);

    } else if (strcmp(cmd, "mit") == 0) {
        if (argc < 8) { printf("Usage: mit <id> <pos> <vel> <kp> <kd> <tor>\n"); return 1; }
        int id = atoi(argv[2]);
        float pos = (float)atof(argv[3]);
        float vel = (float)atof(argv[4]);
        float kp  = (float)atof(argv[5]);
        float kd  = (float)atof(argv[6]);
        float tor = (float)atof(argv[7]);
        stark_mit(c, id, pos, vel, kp, kd, tor);
        printf("Motor %d: MIT pos=%.1f° vel=%.1f kp=%.1f kd=%.1f tor=%.1f\n",
               id, pos, vel, kp, kd, tor);

    } else if (strcmp(cmd, "multi") == 0) {
        if (argc < 8) { printf("Usage: multi <t1> <v1> <p1> <t2> <v2> <p2>\n"); return 1; }
        int32_t t1 = atoi(argv[2]);
        int32_t v1 = atoi(argv[3]);
        int32_t p1 = atoi(argv[4]);
        int32_t t2 = atoi(argv[5]);
        int32_t v2 = atoi(argv[6]);
        int32_t p2 = atoi(argv[7]);
        stark_multi(c, t1, v1, p1, t2, v2, p2);
        printf("MULTI: M1(t=%d v=%d p=%d) M2(t=%d v=%d p=%d)\n", t1,v1,p1, t2,v2,p2);

    } else if (strcmp(cmd, "enable") == 0) {
        if (argc < 3) { printf("Usage: enable <id>\n"); return 1; }
        int id = atoi(argv[2]);
        stark_enable(c, id);
        printf("Motor %d: ENABLE\n", id);

    } else if (strcmp(cmd, "disable") == 0) {
        if (argc < 3) { printf("Usage: disable <id>\n"); return 1; }
        int id = atoi(argv[2]);
        stark_disable(c, id);
        printf("Motor %d: DISABLE\n", id);

    } else if (strcmp(cmd, "estop") == 0) {
        if (argc < 3) { printf("Usage: estop <id>\n"); return 1; }
        int id = atoi(argv[2]);
        stark_estop(c, id);
        printf("Motor %d: ESTOP\n", id);

    } else if (strcmp(cmd, "recover") == 0) {
        if (argc < 3) { printf("Usage: recover <id>\n"); return 1; }
        int id = atoi(argv[2]);
        stark_recover(c, id);
        printf("Motor %d: RECOVER\n", id);

    } else if (strcmp(cmd, "clearf") == 0) {
        if (argc < 3) { printf("Usage: clearf <id>\n"); return 1; }
        int id = atoi(argv[2]);
        stark_clear_fault(c, id);
        printf("Motor %d: CLEAR FAULT\n", id);

    } else if (strcmp(cmd, "mode") == 0) {
        if (argc < 4) { printf("Usage: mode <id> <mode>\n"); return 1; }
        int id   = atoi(argv[2]);
        int mode = atoi(argv[3]);
        stark_set_mode(c, id, mode);
        printf("Motor %d: mode = %d\n", id, mode);

    } else if (strcmp(cmd, "stop") == 0) {
        cmd_stop(c);

    } else if (strcmp(cmd, "stat") == 0) {
        print_stat(c);

    } else if (strcmp(cmd, "watch") == 0) {
        int period = (argc >= 3) ? atoi(argv[3]) : 200;
        cmd_watch(c, period);

    } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "-h") == 0) {
        usage();

    } else {
        printf("Unknown command: %s\n", cmd);
        usage();
        return 1;
    }

    return 0;
}

/* ================================================================
 * Daemon — Unix Domain Socket 服务器
 * ================================================================ */

static volatile int g_daemon_running = 1;

static void daemon_sig(int sig) { (void)sig; g_daemon_running = 0; }

static int daemon_main(void)
{
    signal(SIGINT,  daemon_sig);
    signal(SIGTERM, daemon_sig);

    unlink(SOCK_PATH);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return 1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(fd); return 1;
    }
    if (listen(fd, 5) < 0) {
        perror("listen"); close(fd); unlink(SOCK_PATH); return 1;
    }

    printf("stark_tool daemon listening on %s\n", SOCK_PATH);
    printf("Usage: echo 'torque 1 2000' | nc -U %s\n", SOCK_PATH);
    printf("       echo 'stat' | nc -U %s\n\n", SOCK_PATH);

    while (g_daemon_running) {
        int client = accept(fd, NULL, NULL);
        if (client < 0) break;

        char buf[1024];
        ssize_t n = read(client, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            /* 去掉换行 */
            char* nl = strchr(buf, '\n');
            if (nl) *nl = '\0';

            /* 解析命令 (空格分隔) */
            char* args[16];
            int argc = 0;
            char* tok = strtok(buf, " ");
            while (tok && argc < 16) {
                args[argc++] = tok;
                tok = strtok(NULL, " ");
            }

            /* 第一个参数假为程序名 */
            char* full_args[17];
            full_args[0] = (char*)"stark_tool";
            for (int i = 0; i < argc; i++) full_args[i+1] = args[i];

            stark_client_t c;
            if (stark_open(&c) != 0) {
                dprintf(client, "ERR: cannot open SHM\n");
            } else {
                dispatch(&c, argc + 1, full_args);
                stark_close(&c);
            }
        }
        close(client);
    }

    close(fd);
    unlink(SOCK_PATH);
    printf("daemon stopped\n");
    return 0;
}

/* ================================================================
 * main
 * ================================================================ */

int main(int argc, char** argv)
{
    if (argc >= 2 && strcmp(argv[1], "daemon") == 0) {
        return daemon_main();
    }

    /* 普通命令: 直连 SHM */
    stark_client_t c;
    if (stark_open(&c) != 0) {
        printf("ERR: SHM not available (%s). Is stark_periph_node running?\n",
               STARK_SHM_NAME);
        return 1;
    }

    int ret = dispatch(&c, argc, argv);
    stark_close(&c);
    return ret;
}
