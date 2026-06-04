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
int tool_set_torque(int id, int ma);                /* 电流/力矩: mA */
int tool_set_csp(int id, int deg_x100);             /* CSP 同步位置: 度×100 */
int tool_set_mit(int id, float pos, float vel, float kp, float kd, float torque);
int tool_set_brake(int id, bool release);            /* 抱闸: true=松开, false=吸合 */
int tool_quickstop(int id);                         /* 急停 */
int tool_save_flash(int id);                        /* 保存到 Flash */
int tool_set_zero(int id);                          /* 零位标定 */
int tool_set_pid(int id, uint16_t cp, uint16_t ci,  /* PID 设置 */
                 uint16_t vp, uint16_t vi, uint16_t pp, uint16_t pi);
int tool_sdo_read(int id, uint16_t index, uint8_t subidx);   /* 通用 SDO 读 */
int tool_sdo_write(int id, uint16_t index, uint8_t subidx,   /* 通用 SDO 写 */
                   uint32_t value, uint8_t size);

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
