/**
 * @file pdo_handler.c
 * @brief PDO 构造与反馈解析 - 高层封装
 *
 * 封装 canopen_frames 的 PDO 相关函数,
 * 提供更简洁的调用接口供 motor_hal.c 使用。
 */

#include "canopen_frames.h"
#include "can_driver_internal.h"
#include <stdio.h>

/* ---------- 诊断: hex dump ---------- */
static void _dump_hex(const char *dir, const canfd_frame_t *f)
{
    fprintf(stderr, "[PDO %s] id=0x%03X dlc=%d :", dir, f->id, f->dlc);
    for (int i = 0; i < f->dlc && i < 64; i++) {
        fprintf(stderr, " %02X", f->data[i]);
    }
    fprintf(stderr, "\n");
}

/* ---------- 控制 PDO 发送 ---------- */

void pdo_ctrl_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                   bool enable, bool release_brake, bool clear_err,
                   int16_t target1, uint16_t target2, int16_t feedforward)
{
    canfd_frame_t f;
    canopen_custom_pdo_build(node, mode, enable, release_brake, clear_err,
                             target1, target2, feedforward, &f);
    _dump_hex("TX", &f);
    can_driver_send(drv, &f);
}

void pdo_mit_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                  bool enable, bool release_brake, bool clear_err,
                  uint16_t position, uint16_t velocity,
                  uint16_t kp, uint16_t kd, uint16_t torque)
{
    canfd_frame_t f;
    canopen_mit_pdo_build(node, mode, enable, release_brake, clear_err,
                          position, velocity, kp, kd, torque, &f);
    _dump_hex("TX", &f);
    can_driver_send(drv, &f);
}

void pdo_multi_send(can_driver_t *drv, const multi_axis_cmd_t *cmds, uint8_t count)
{
    canfd_frame_t f;
    canopen_multi_ctrl_build(cmds, count, &f);
    _dump_hex("TX", &f);
    can_driver_send(drv, &f);
}

/* ---------- SYNC ---------- */

void pdo_sync_send(can_driver_t *drv)
{
    canfd_frame_t f;
    canopen_sync_build(&f);
    _dump_hex("TX", &f);
    can_driver_send(drv, &f);
}

/* ---------- 反馈解析 ---------- */

void pdo_feedback_parse(const canfd_frame_t *f, motor_feedback_t *fb)
{
    canopen_parse_feedback(f, fb);
}
