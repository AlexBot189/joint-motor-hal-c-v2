/**
 * @file daemon.h
 * @brief motor_tool 后台守护进程
 *
 * 架构:
 *   motor_tool daemon can0    → 启动 daemon, 初始化 CAN, 启动双电机
 *   motor_tool speed 1 5000   → 通过 Unix socket 向 daemon 发命令
 *   motor_tool watch 200      → 通过 Unix socket 连接, 持续接收反馈流
 *   motor_tool stop           → 优雅停止 daemon
 */

#ifndef MOTOR_TOOL_DAEMON_H
#define MOTOR_TOOL_DAEMON_H

#define MOTOR_TOOL_SOCK_PATH  "/tmp/motor_tool.sock"

/* 客户端 → daemon: 发送命令行字符串 */
/* daemon → 客户端: 发送 JSON 格式响应 (单行) */
/* watch 模式: daemon 持续发送 {"type":"watch",...} 直到连接断开 */

int daemon_start(const char *iface);    /* 启动 daemon, 初始化 CAN + 双电机 */
int daemon_stop(void);                  /* 停止 daemon: 脱使能 + 清理 */
int client_send(int argc, char **argv); /* 客户端: 连接 daemon, 发送命令, 打印响应 */
int client_sensor_watch(int argc, char **argv); /* 传感器看板客户端 (长连接) */
int daemon_get_client_fd(void);         /* 获取当前客户端 fd (供 watch 等使用) */

#endif /* MOTOR_TOOL_DAEMON_H */
