/*
 * stark_client.h — 上层算法控制接口 (Header-Only)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 用法: 算法进程只需 #include "stark_client.h", 无需链接任何库.
 *
 * 快速开始:
 *   stark_client_t c;
 *   stark_open(&c);                    // 连接 SHM
 *   while (!stark_ready(&c)) usleep(10000);  // 等校准完成
 *   while (1) {
 *       motor_data_t fb = stark_fb(&c, 1);         // 读电机反馈
 *       imu_data_t imu = stark_imu(&c);            // 读 IMU
 *       stark_multi(&c, tor1, 0, 0, tor2, 0, 0);   // 写控制命令
 *       usleep(1000);
 *   }
 *   stark_close(&c);
 *
 * 控制模式:
 *   - 力矩控制: stark_torque(c, id, ma)   : ID对应电机, mA
 *   - 速度控制: stark_speed(c, id, rpm)    : RPM
 *   - 位置控制: stark_position(c, id, deg) : 绝对角度(deg)
 *   - 多轴广播: stark_multi(c, t1,v1,p1, t2,v2,p2)  : 一帧CANFD双电机
 *   - MIT阻抗:  stark_mit(c,id,pos,vel,kp,kd,tor)
 *
 * 管理命令:
 *   - stark_enable(c, id) / stark_disable(c, id)
 *   - stark_estop(c, id) / stark_recover(c, id)
 *   - stark_clear_fault(c, id)
 *
 * 注意事项:
 *   1. 本文件依赖 stark_shm.h (共享内存数据结构), 保持版本一致
 *   2. 所有函数均为 static inline, 零函数调用开销
 *   3. 仅适用于 SHM 存在且 motor_node 已启动的场景
 *   4. 管理命令 (enable/disable/estop) 和实时命令 (torque/speed/multi)
 *      不要在同一周期混合发送; 管理命令触发 Byte0 修改, 实时命令复用
 */
#pragma once

#include "stark_shm.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 客户端句柄 */
typedef struct {
    int         fd;
    stark_shm_t* shm;
} stark_client_t;

/* ================================================================
 * 生命周期
 * ================================================================ */

static inline int stark_open(stark_client_t* c)
{
    if (!c) return -1;
    c->fd = shm_open(STARK_SHM_NAME, O_RDWR, 0666);
    if (c->fd < 0) return -1;
    c->shm = (stark_shm_t*)mmap(NULL, STARK_SHM_SIZE,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 c->fd, 0);
    if (c->shm == MAP_FAILED) {
        close(c->fd);
        c->fd = -1;
        return -1;
    }
    return 0;
}

static inline void stark_close(stark_client_t* c)
{
    if (!c || !c->shm) return;
    munmap(c->shm, STARK_SHM_SIZE);
    close(c->fd);
    c->shm = NULL;
    c->fd  = -1;
}

/* ================================================================
 * 状态查询
 * ================================================================ */

static inline int stark_ready(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return (c->shm->calib_state == 2);
}

static inline int stark_online(stark_client_t* c, int id)
{
    if (!c || !c->shm || id < 1 || id > 2) return 0;
    return (c->shm->motor_online & (1 << (id - 1))) != 0;
}

static inline int stark_state(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return __atomic_load_n(&c->shm->node_state, __ATOMIC_ACQUIRE);
}

static inline int stark_calib(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return c->shm->calib_state;
}

static inline int stark_severity(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return c->shm->motor_severity;
}

static inline int stark_fault_reason(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return c->shm->fault_reason;
}

/* ================================================================
 * 反馈读取 (零拷贝, 读 active Buffer)
 * ================================================================ */

static inline motor_data_t stark_fb(stark_client_t* c, int id)
{
    motor_data_t fb = {0};
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return fb;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    const feedback_frame_t* f = &c->shm->fb_buffer[idx];
    return f->motor[id - 1];
}

static inline imu_data_t stark_imu(stark_client_t* c)
{
    imu_data_t imu = {0};
    if (!c || !c->shm) return imu;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    return c->shm->fb_buffer[idx].imu;
}

static inline stark_sensor_data_t stark_sensor(stark_client_t* c, int id)
{
    stark_sensor_data_t s = {0};
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return s;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    return c->shm->fb_buffer[idx].sensor[id - 1];
}

static inline barometer_data_t stark_baro(stark_client_t* c)
{
    barometer_data_t b = {0};
    if (!c || !c->shm) return b;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    return c->shm->fb_buffer[idx].baro;
}

/* ================================================================
 * 控制命令 — 实时路径 (写 SHM mailbox, 走 PDO)
 * ================================================================ */

/* 内部: 准备 mailbox 写 (seq_begin++) */
static inline uint64_t _stark_mbox_begin(stark_client_t* c)
{
    return __atomic_add_fetch(&c->shm->mailbox.seq_begin, 1, __ATOMIC_RELEASE);
}

/* 内部: 提交 mailbox */
static inline void _stark_mbox_end(stark_client_t* c, uint64_t seq)
{
    __atomic_store_n(&c->shm->mailbox.seq_end, seq, __ATOMIC_RELEASE);
}

/* 力矩控制 */
static inline void stark_torque(stark_client_t* c, int id, int32_t ma)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    int idx = id - 1;

    c->shm->mailbox.cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.cmd[idx].cmd         = STARK_CMD_TORQUE;
    c->shm->mailbox.cmd[idx].value       = ma;

    _stark_mbox_end(c, seq);
}

/* 速度控制 */
static inline void stark_speed(stark_client_t* c, int id, float rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    int idx = id - 1;

    c->shm->mailbox.cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.cmd[idx].cmd         = STARK_CMD_SPEED;
    c->shm->mailbox.cmd[idx].value       = (int32_t)(rpm * 100.0f);

    _stark_mbox_end(c, seq);
}

/* 循环同步速度 (CSV 模式) */
static inline void stark_csv(stark_client_t* c, int id, float rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    int idx = id - 1;

    c->shm->mailbox.cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.cmd[idx].cmd         = STARK_CMD_CSV;
    c->shm->mailbox.cmd[idx].value       = (int32_t)(rpm * 100.0f);

    _stark_mbox_end(c, seq);
}

/* 绝对位置控制 (CSP 模式) */
static inline void stark_position(stark_client_t* c, int id, float deg)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    int idx = id - 1;

    c->shm->mailbox.cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.cmd[idx].cmd         = STARK_CMD_POS;
    c->shm->mailbox.cmd[idx].value       = (int32_t)(deg * 100.0f);

    _stark_mbox_end(c, seq);
}

/* 相对位置控制 — 算法读当前位置 + 偏移, 写绝对目标 */
static inline void stark_rel_position(stark_client_t* c, int id, float delta_deg)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    /* 读当前位置 (角度), 加偏移 */
    motor_data_t fb = stark_fb(c, id);
    float cur_deg  = (float)fb.position * (360.0f / 65536.0f);
    float target   = cur_deg + delta_deg;

    /* 钳位到 ±180° */
    if (target > 180.0f)  target -= 360.0f;
    if (target < -180.0f) target += 360.0f;

    stark_position(c, id, target);
}

/* 轮廓位置模式 (PP 模式, 带加减速) */
static inline void stark_pp(stark_client_t* c, int id,
                             float deg, float accel_rpm, float vel_rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    int idx = id - 1;

    c->shm->mailbox.cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.cmd[idx].cmd         = STARK_CMD_PP;
    c->shm->mailbox.cmd[idx].value       = (int32_t)(deg * 100.0f);
    c->shm->mailbox.cmd[idx].value2      = (int32_t)(accel_rpm * 100.0f);
    c->shm->mailbox.cmd[idx].feedforward = (int32_t)(vel_rpm * 100.0f);

    _stark_mbox_end(c, seq);
}

/* MIT 阻抗控制 */
static inline void stark_mit(stark_client_t* c, int id,
                              float pos_deg, float vel_rpm,
                              float kp, float kd, float torque_ma)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    int idx = id - 1;

    c->shm->mailbox.cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.cmd[idx].cmd         = STARK_CMD_MIT;
    c->shm->mailbox.cmd[idx].mit_pos     = (uint16_t)((pos_deg + 180.0f) * 65535.0f / 360.0f);
    c->shm->mailbox.cmd[idx].mit_vel     = (uint16_t)(vel_rpm);
    c->shm->mailbox.cmd[idx].mit_kp      = (uint16_t)(kp * 100.0f);
    c->shm->mailbox.cmd[idx].mit_kd      = (uint16_t)(kd * 100.0f);
    c->shm->mailbox.cmd[idx].mit_torque  = (uint16_t)(torque_ma);

    _stark_mbox_end(c, seq);
}

/* 多轴广播 — 一帧 64B CANFD 同时发双电机
 *
 * 参数:
 *   t1, t2 = 目标力矩 (mA), 电流模式有效
 *   v1, v2 = 速度前馈 (RPM), 位置/速度模式有效
 *   p1, p2 = 位置 (deg), 位置模式有效
 *
 * 当前驱动板控制的模式决定哪个字段生效.
 * 默认电流模式: 只用 t1/t2, v1/v2/p1/p2 填 0 */
static inline void stark_multi(stark_client_t* c,
                                int32_t t1, int32_t v1, int32_t p1,
                                int32_t t2, int32_t v2, int32_t p2)
{
    if (!c || !c->shm) return;

    uint64_t seq = _stark_mbox_begin(c);

    c->shm->mailbox.cmd[0].motor_id    = 1;
    c->shm->mailbox.cmd[0].cmd         = STARK_CMD_MULTI;
    c->shm->mailbox.cmd[0].value       = t1;
    c->shm->mailbox.cmd[0].value2      = v1;
    c->shm->mailbox.cmd[0].feedforward = p1;

    c->shm->mailbox.cmd[1].motor_id    = 2;
    c->shm->mailbox.cmd[1].cmd         = STARK_CMD_MULTI;
    c->shm->mailbox.cmd[1].value       = t2;
    c->shm->mailbox.cmd[1].value2      = v2;
    c->shm->mailbox.cmd[1].feedforward = p2;

    _stark_mbox_end(c, seq);
}

/* ================================================================
 * 管理命令 — Byte0 路径 (PDO Byte0 修改, RT 线程下一周期生效)
 *
 * 注意: 这些命令和实时控制命令 (torque/speed/multi) 不要在同一周期混合发送.
 * 管理命令触发 Byte0 修改后在当前周期立即生效, 实时命令复用该 Byte0 状态.
 * 如需安全: 先发管理命令, 等待至少 1ms, 再发实时命令.
 * ================================================================ */

static inline void _stark_byte0_cmd(stark_client_t* c, int id, int cmd)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    c->shm->mailbox.cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.cmd[id - 1].cmd      = (uint8_t)cmd;
    _stark_mbox_end(c, seq);
}

static inline void stark_enable(stark_client_t* c, int id)    { _stark_byte0_cmd(c, id, STARK_CMD_ENABLE); }
static inline void stark_disable(stark_client_t* c, int id)   { _stark_byte0_cmd(c, id, STARK_CMD_DISABLE); }
static inline void stark_estop(stark_client_t* c, int id)     { _stark_byte0_cmd(c, id, STARK_CMD_ESTOP); }
static inline void stark_recover(stark_client_t* c, int id)   { _stark_byte0_cmd(c, id, STARK_CMD_RECOVER); }
static inline void stark_clear_fault(stark_client_t* c, int id) { _stark_byte0_cmd(c, id, STARK_CMD_CLEAR_FAULT); }

static inline void stark_set_mode(stark_client_t* c, int id, int mode)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    uint64_t seq = _stark_mbox_begin(c);
    c->shm->mailbox.cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.cmd[id - 1].cmd      = STARK_CMD_SET_MODE;
    c->shm->mailbox.cmd[id - 1].value    = mode;
    _stark_mbox_end(c, seq);
}

#ifdef __cplusplus
}
#endif
