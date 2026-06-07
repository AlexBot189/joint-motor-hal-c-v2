/**
 * @file tool_hal.h
 * @brief motor_hal 薄封装层 — ×100换算 + id=0广播 + SDO时序 + 格式化输出
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
int  tool_register_motor(int node_id);
int  tool_motor_count(void);
int  tool_motor_id(int index);

/* ================================================================
 * 系统命令
 * ================================================================ */

int tool_disable(int id);
int tool_fault_reset(int id);
int tool_reboot(int id);

/* ================================================================
 * SDO 电流控制 — 完整时序 (使能→切模式→写目标电流)
 *   torque <id> <mA>  范围 0~20000 mA (0~20A)
 * ================================================================ */

int tool_torque_sdo(int id, int ma);

/* ================================================================
 * SDO 速度控制 — 完整时序 (使能→切模式→设加减速→写目标速度)
 *   speed <id> <rpm*100> [acc*100] [dec*100]
 *   加减速范围 0~10000 RPM/s, 速度无上限
 * ================================================================ */

int tool_speed_sdo(int id, int rpm_x100, int acc_x100, int dec_x100);

/* ================================================================
 * SDO 位置控制 — 完整时序 (使能→切模式→设加减速/轨迹速度→目标→启动)
 *   abs <id> <deg*100>
 *   加减速范围 0~10000 RPM/s (默认2000)
 *   轨迹速度范围 0~30 RPM (默认10)
 *   目标位置范围 -32767~32768 counts (-180°~180°)
 * ================================================================ */

int tool_abs_sdo(int id, int deg_x100);

/* 位置控制参数配置 */
int tool_abs_set_accel(int id, int acc_x100);  /* 加减速 RPM/s×100 */
int tool_abs_set_speed(int id, int rpm_x100);  /* 轨迹速度 RPM×100 */

/* 停止位置运动 (CW=0x0F) */
int tool_abs_stop(int id);

/* ================================================================
 * 单控 SDO 命令
 * ================================================================ */

int tool_set_zero_auto(int id);                 /* setzero: 自动失能→写0x2531 */
int tool_limit_pos_set(int id, int deg_x100);   /* limit_pos: 自动失能→写0x607D/02→save_flash */
int tool_limit_neg_set(int id, int deg_x100);   /* limit_neg: 自动失能→写0x607D/01→save_flash */
int tool_limit_pos_read(int id);                /* 读 0x607D/02 */
int tool_limit_neg_read(int id);                /* 读 0x607D/01 */
int tool_save_flash(int id);
int tool_set_pid(int id, uint16_t cp, uint16_t ci,
                 uint16_t vp, uint16_t vi, uint16_t pp, uint16_t pi);
int tool_read_pid(int id);

/* 通用 SDO 读写 */
int tool_sdo_read(int id, uint16_t index, uint8_t subidx);
int tool_sdo_write(int id, uint16_t index, uint8_t subidx,
                   uint32_t value, uint8_t size);

/* 传感器看板 */
int tool_sensor_watch_start(int id, int out_fd);
int tool_sensor_watch_stop(void);

/* ================================================================
 * 读取封装 — id=0 广播, 格式化输出
 * ================================================================ */

int tool_read_angle(int id);
int tool_read_speed(int id);
int tool_read_current(int id);
int tool_read_temp(int id);
int tool_read_state(int id);
int tool_read_error(int id);
int tool_read_version(int id);
int tool_read_mode(int id);
int tool_read_all(int id);

/* ================================================================
 * PDO 映射
 * ================================================================ */

int tool_pdo_map(uint8_t id, pdo_type_t type,
                 const pdo_map_entry_cfg_t *entries, uint8_t count,
                 uint32_t cob_id, uint8_t trans_type);

/* ================================================================
 * watch 模式
 * ================================================================ */

int tool_watch_start(int period_ms, int out_fd);

#endif /* TOOL_HAL_H */
