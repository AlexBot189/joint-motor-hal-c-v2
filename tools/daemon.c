/**
 * @file daemon.c
 * @brief motor_tool 守护进程 — Unix socket + HAL 管理
 *
 * daemon 流程:
 *   1. 初始化 CANFD (motor_hal_init)
 *   2. 注册电机配置 (ID 1~4, 仅注册, 不启动)
 *   3. 后台化 (fork — ★必须在创建线程之前)
 *   4. 启动接收线程 (在子进程中, 捕获后续 Bootup/SDO/PDO)
 *   5. 信号 + socket + accept 循环
 *   6. 客户端命令: motor_tool startup 按需启停电机
 *   7. 收到 stop 命令时优雅退出
 */

#include "daemon.h"
#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>
#include <errno.h>
#include <time.h>

/* ================================================================
 * 全局 daemon 状态
 * ================================================================ */

static volatile int g_daemon_running = 0;
static int g_listen_fd = -1;
static int g_client_fd = -1;

static void _sig_handler(int sig)
{
    (void)sig;
    g_daemon_running = 0;
}

/* ================================================================
 * 后台化
 * ================================================================ */

static void _daemonize(void)
{
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(1); }
    if (pid > 0) {
        printf("motor_tool daemon starting (pid=%d)\n", pid);
        _exit(0);
    }
    setsid();
    signal(SIGHUP, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);
}

/* ================================================================
 * Unix socket 创建
 * ================================================================ */

static int _create_socket(void)
{
    unlink(MOTOR_TOOL_SOCK_PATH);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, MOTOR_TOOL_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(fd); return -1;
    }
    if (listen(fd, 5) < 0) {
        perror("listen"); close(fd); return -1;
    }

    return fd;
}

/* ================================================================
 * 执行命令 + 返回 JSON 响应
 * ================================================================ */

#define RESP_BUF_SIZE 4096

static void _send_response(int fd, const char *type, const char *msg, int ret)
{
    char buf[RESP_BUF_SIZE];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"%s\",\"ret\":%d,\"msg\":\"%s\"}\n",
             type, ret, msg);
    write(fd, buf, strlen(buf));
}

static void _send_watch_line(int fd, const char *line)
{
    char buf[RESP_BUF_SIZE];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"watch\",\"data\":\"%s\"}\n", line);
    write(fd, buf, strlen(buf));
}

/* ================================================================
 * 处理单条命令
 * ================================================================ */

static void _process_command(int client_fd, char *cmdline)
{
    /* 解析 argv (空格分隔) */
    char *argv[32];
    int argc = 0;
    char *saveptr;
    char *token = strtok_r(cmdline, " \t\r\n", &saveptr);
    while (token && argc < 31) {
        argv[argc++] = token;
        token = strtok_r(NULL, " \t\r\n", &saveptr);
    }
    argv[argc] = NULL;

    if (argc == 0) {
        _send_response(client_fd, "error", "empty command", -1);
        return;
    }

    /* help 命令: 直接返回帮助文本 */
    if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
        /* 静默执行 help, 不管返回值 */
        g_client_fd = client_fd;
        cmd_do_help(g_hal, CMD_HELP, argc + 1, argv);  /* +1 因为 cmd_dispatch 需要 argv[0] */
        /* help 输出到 stdout, 这里走 daemon 的 stdout (可能被重定向) */
        _send_response(client_fd, "ok", "help sent to daemon stdout", 0);
        return;
    }

    /* 对于需要持续输出的 watch 命令, 设置全局 client_fd */
    if (strcmp(argv[0], "watch") == 0) {
        g_client_fd = client_fd;
    }

    /* 对 argv 补一个前缀 (模拟 motor_tool 命令名), 适配 cmd_dispatch */
    char *full_argv[34];
    full_argv[0] = "motor_tool";
    for (int i = 0; i < argc; i++) full_argv[i + 1] = argv[i];
    full_argv[argc + 1] = NULL;

    int ret = cmd_dispatch(g_hal, argc + 1, full_argv);

    if (strcmp(argv[0], "watch") != 0) {
        _send_response(client_fd, "ok", "done", ret);
    }
}

/* ================================================================
 * watch 输出重定向
 * ================================================================ */

void daemon_watch_print(const char *line)
{
    if (g_client_fd >= 0) {
        _send_watch_line(g_client_fd, line);
    }
}

int daemon_get_client_fd(void) { return g_client_fd; }

/* ================================================================
 * 主循环: accept → 读命令 → 执行 → 响应
 * ================================================================ */

static void _accept_loop(void)
{
    char buf[4096];

    while (g_daemon_running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(g_listen_fd, &fds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };

        int ret = select(g_listen_fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;

        int client_fd = accept(g_listen_fd, NULL, NULL);
        if (client_fd < 0) continue;

        ssize_t n = read(client_fd, buf, sizeof(buf) - 1);
        if (n <= 0) { close(client_fd); continue; }
        buf[n] = '\0';

        g_client_fd = client_fd;
        _process_command(client_fd, buf);

        close(client_fd);
        g_client_fd = -1;
    }
}

/* ================================================================
 * 公共 API
 * ================================================================ */

int daemon_start(const char *iface)
{
    /* 1. 初始化 CAN */
    if (tool_init(iface) < 0) return -1;

    /* 2. 注册电机配置 (仅注册, 不启动 — 由 motor_tool startup 手动触发) */
    motor_config_t cfg = {0};
    cfg.heartbeat_ms      = 2000;
    cfg.profile_accel     = 5000;
    cfg.profile_decel     = 5000;
    cfg.profile_velocity  = 20;
    cfg.disable_watchdog  = true;
    cfg.auto_enable       = true;
    cfg.bootup_timeout_ms = 3000;

    int motor_ids[] = {1, 2, 3, 4};
    int motor_count = sizeof(motor_ids) / sizeof(motor_ids[0]);

    for (int i = 0; i < motor_count; i++) {
        cfg.node_id = (uint8_t)motor_ids[i];
        motor_hal_add_motor(g_hal, &cfg);
    }

    /* 3. 后台化 ★ 必须在创建线程之前 fork, 否则子线程在子进程中丢失 */
    _daemonize();

    /* 4. 启动接收线程 — 在子进程中创建, 安全 */
    motor_hal_recv_start(g_hal);

    /* 5. 信号处理 */
    signal(SIGINT,  _sig_handler);
    signal(SIGTERM, _sig_handler);

    /* 6. 创建 socket */
    g_listen_fd = _create_socket();
    if (g_listen_fd < 0) { tool_cleanup(); return -1; }

    /* 7. 主循环 */
    g_daemon_running = 1;

    printf("motor_tool daemon ready. Use 'motor_tool startup <id>' to start motors.\n");

    _accept_loop();

    /* 8. 清理 */
    close(g_listen_fd);
    unlink(MOTOR_TOOL_SOCK_PATH);
    motor_hal_nmt_broadcast(g_hal, NMT_CMD_STOP);
    tool_cleanup();

    printf("motor_tool daemon stopped.\n");
    return 0;
}

int daemon_stop(void)
{
    /* 发送 stop 命令给 daemon */
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, MOTOR_TOOL_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "Daemon not running? (%s)\n", strerror(errno));
        close(fd); return -1;
    }

    char *cmd = "stop 0";
    write(fd, cmd, strlen(cmd));
    close(fd);
    return 0;
}

/* ================================================================
 * 客户端
 * ================================================================ */

int client_send(int argc, char **argv)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, MOTOR_TOOL_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "ERROR: daemon not running. Start with:\n");
        fprintf(stderr, "  motor_tool daemon can0\n");
        close(fd); return -1;
    }

    /* 拼接命令行 */
    char cmdline[2048] = {0};
    for (int i = 1; i < argc; i++) {  /* 跳过 argv[0]="motor_tool" */
        if (i > 1) strncat(cmdline, " ", sizeof(cmdline) - strlen(cmdline) - 1);
        strncat(cmdline, argv[i], sizeof(cmdline) - strlen(cmdline) - 1);
    }

    write(fd, cmdline, strlen(cmdline));

    /* 读响应 (可能多行, 用于 watch) */
    char buf[4096];
    while (1) {
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n <= 0) break;
        buf[n] = '\0';

        /* 解析 JSON 行并输出 */
        char *line = strtok(buf, "\n");
        while (line) {
            /* 简单处理: 提取 msg 字段 */
            /* watch 行的 data 字段 */
            char *data_start = strstr(line, "\"data\":\"");
            if (data_start) {
                data_start += 8;  /* strlen("\"data\":\"") */
                char *data_end = strchr(data_start, '"');
                if (data_end) *data_end = '\0';
                printf("%s\n", data_start);
            } else {
                /* 普通响应: 提取 msg */
                char *msg_start = strstr(line, "\"msg\":\"");
                if (msg_start) {
                    msg_start += 7;
                    char *msg_end = strchr(msg_start, '"');
                    if (msg_end) *msg_end = '\0';
                    printf("%s\n", msg_start);
                }
            }
            line = strtok(NULL, "\n");
        }

        /* 非 watch 命令只收一次响应 */
        if (argc >= 2 && strcmp(argv[1], "watch") != 0) break;
    }

    close(fd);
    return 0;
}
