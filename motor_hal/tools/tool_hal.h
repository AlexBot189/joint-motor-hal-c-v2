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
int  tool_unregister_motor(int node_id);
int  tool_motor_count(void);
int  tool_motor_id(int index);
int  tool_filter_online(int *ids, int n);  /* 过滤离线电机, 返回在线数量 */

/* ================================================================
 * 系统命令
 * ================================================================ */

int tool_disable(int id);
int tool_fault_reset(int id);
int tool_reboot(int id);

/* ================================================================
 * SDO 电流/位置/速度控制 — 统一接口 (自动使能 + 切模式 + 写目标)
 *   tool_sdo_cur/pos/csp/vel/csv(n1, n2, val1, val2, is_dual)
 *   单电机: n2=0, val2=0, is_dual=0
 *   双电机同值: n1, n2, val1=val2=value, is_dual=1
 *   双电机异值: n1, n2, val1, val2, is_dual=1
 * ================================================================ */

int tool_sdo_cur(int n1, int n2, int ma1, int ma2, int is_dual);
int tool_sdo_pos(int n1, int n2, float deg1, float deg2, int is_dual);
int tool_sdo_csp(int n1, int n2, float deg1, float deg2, int is_dual);
int tool_sdo_vel(int n1, int n2, int rpm1, int rpm2, int is_dual);
int tool_sdo_csv(int n1, int n2, int rpm1, int rpm2, int is_dual);

/* ================================================================
 * PDO 控制 — 统一接口 (多轴广播 COB 0x200, 不触发 SDO)
 *   tool_pdo_cur/pos/csp/vel/csv(n1, n2, val1, val2, is_dual)
 * ================================================================ */

int tool_pdo_cur(int n1, int n2, int ma1, int ma2, int is_dual);
int tool_pdo_pos(int n1, int n2, float deg1, float deg2, int is_dual);
int tool_pdo_csp(int n1, int n2, float deg1, float deg2, int is_dual);
int tool_pdo_vel(int n1, int n2, int rpm1, int rpm2, int is_dual);
int tool_pdo_csv(int n1, int n2, int rpm1, int rpm2, int is_dual);

/* ================================================================
 * 单控 SDO 命令
 * ================================================================ */

int tool_set_zero_auto(int id);                 /* setzero: 自动失能, 写0x2531 */
int tool_limit_pos_set(int id, float deg);   /* limit_pos: 自动失能, 写0x607D/02, save_flash */
int tool_limit_neg_set(int id, float deg);   /* limit_neg: 自动失能, 写0x607D/01, save_flash */
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

/** @brief 发送标准 RPDO 控制帧 */
int tool_rpdo_send(uint8_t id, const uint8_t *data, uint8_t dlc);

/* ================================================================
 * watch 模式
 * ================================================================ */

int tool_watch_start(int period_ms, int out_fd);

#endif /* TOOL_HAL_H */
