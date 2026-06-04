/**
 * @file tool_hal.h
 * @brief motor_hal 薄封装层 — ×100换算 + id=0广播 + 格式化输出
 */

#ifndef TOOL_HAL_H
#define TOOL_HAL_H

#include "motor_hal.h"

#define TOOL_MAX_MOTORS  16
#define TOOL_BROADCAST_ID 0

/* ================================================================
 * 全局 HAL 上下文 (单例, 多个命令共享)
 * ================================================================ */

extern motor_hal_t *g_hal;

/* ================================================================
 * 工具初始化 / 清理
 * ================================================================ */

int  tool_init(const char *iface);
void tool_cleanup(void);
int  tool_register_motor(int node_id);  /* 注册电机到广播列表 */
int  tool_motor_count(void);            /* 已注册电机数量 */
int  tool_motor_id(int index);          /* 获取第 index 个电机的 ID (-1=越界) */

/* ================================================================
 * 控制封装 — ×100 换算 + id=0 广播支持
 * 返回值: 0=全部成功, -1=部分或全部失败
 * ================================================================ */

int tool_set_speed(int id, int rpm_x100);           /* 速度: RPM×100 */
int tool_set_accel(int id, int acc_x100);           /* 加减速: RPM/s×100 (同值) */
int tool_set_abs_pos(int id, int deg_x100);         /* 绝对位置: 度×100 */
int tool_set_rel_pos(int id, int delta_x100);       /* 相对位置: 度×100 */
int tool_set_max_vel(int id, int rpm_x100);         /* 位置模式最大速度: RPM×100 */
int tool_stop(int id);                              /* 停止 */
int tool_set_mode(int id, const char *mode_str);    /* 切换模式 */

/* ================================================================
 * 读取封装 — id=0 广播支持, 格式化输出
 * ================================================================ */

int tool_read_angle(int id);         /* 输出编码器count值 */
int tool_read_speed(int id);         /* 输出 RPM */
int tool_read_current(int id);       /* 输出 mA */
int tool_read_temp(int id);          /* 输出 raw (×0.1°C) */
int tool_read_state(int id);         /* 输出 DS402 状态名称 */
int tool_read_error(int id);         /* 输出 错误码 */
int tool_read_version(int id);       /* 输出 固件版本 */
int tool_read_all(int id);           /* 输出 全部 */

/* ================================================================
 * 轮询封装 
 * ================================================================ */

void tool_poll(int timeout_ms);

/* ================================================================
 * watch 模式
 * ================================================================ */

int tool_watch_start(int period_ms, int out_fd);

#endif /* TOOL_HAL_H */
