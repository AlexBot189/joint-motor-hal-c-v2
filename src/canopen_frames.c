/**
 * @file canopen_frames.c
 * @brief CANopen 帧构造与解析实现
 */

#include "canopen_frames.h"
#include <sys/time.h>

/* =====================================================
 * 时间戳
 * ===================================================== */

static uint64_t _now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000UL + (uint64_t)tv.tv_usec;
}

/* =====================================================
 * SDO 帧
 * ===================================================== */

void canopen_sdo_write_build(uint8_t node, uint16_t index, uint8_t subidx,
                              uint32_t value, uint8_t size_bytes,
                              canfd_frame_t *f)
{
    uint8_t cmd;
    switch (size_bytes) {
        case 1: cmd = SDO_CC_DOWNLOAD_1B; break;
        case 2: cmd = SDO_CC_DOWNLOAD_2B; break;
        case 4:
        default:cmd = SDO_CC_DOWNLOAD_4B; break;
    }

    memset(f, 0, sizeof(*f));
    f->id    = COB_SDO_TX_BASE + node;
    f->dlc   = 8;
    f->is_fd = false;

    f->data[0] = cmd;
    f->data[1] = (uint8_t)(index & 0xFF);
    f->data[2] = (uint8_t)((index >> 8) & 0xFF);
    f->data[3] = subidx;
    f->data[4] = (uint8_t)(value & 0xFF);
    f->data[5] = (uint8_t)((value >> 8) & 0xFF);
    f->data[6] = (uint8_t)((value >> 16) & 0xFF);
    f->data[7] = (uint8_t)((value >> 24) & 0xFF);
}

void canopen_sdo_read_build(uint8_t node, uint16_t index, uint8_t subidx,
                             canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_SDO_TX_BASE + node;
    f->dlc   = 8;
    f->is_fd = false;

    f->data[0] = SDO_CC_UPLOAD_REQ;
    f->data[1] = (uint8_t)(index & 0xFF);
    f->data[2] = (uint8_t)((index >> 8) & 0xFF);
    f->data[3] = subidx;
}

bool canopen_sdo_parse_response(const canfd_frame_t *f,
                                uint32_t *value, uint16_t *out_index,
                                uint8_t *out_subidx, uint32_t *abort_code)
{
    if (!f) return false;

    uint8_t cmd = f->data[0];

    if (cmd == SDO_CC_ABORT) {
        if (abort_code) {
            *abort_code = (uint32_t)f->data[4]
                        | ((uint32_t)f->data[5] << 8)
                        | ((uint32_t)f->data[6] << 16)
                        | ((uint32_t)f->data[7] << 24);
        }
        return false;
    }

    if (out_index)
        *out_index = (uint16_t)f->data[1] | ((uint16_t)f->data[2] << 8);
    if (out_subidx)
        *out_subidx = f->data[3];

    switch (cmd) {
        case SDO_CC_UPLOAD_RSP_1B:  /* 0x4F */
            if (value) *value = f->data[4];
            return true;
        case SDO_CC_UPLOAD_RSP_2B:  /* 0x4B */
            if (value) *value = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8);
            return true;
        case SDO_CC_UPLOAD_RSP_4B:  /* 0x43 */
        case 0x41:                  /* 0x41: expedited 4B, e=0 variant (巨蟹固件) */
        case 0x47:                  /* 0x47: expedited 3B, e=0 variant */
            if (value) {
                *value = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8)
                       | ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24);
            }
            return true;
        case 0x60:  /* SDO 写确认 (从站→主站) */
            return true;
        case SDO_CC_DOWNLOAD_1B:
        case SDO_CC_DOWNLOAD_2B:
        case SDO_CC_DOWNLOAD_4B:
            /* 从站回显下载命令码 (某些驱动用这种方式确认) */
            return true;
        default:
            return false;
    }
}

/* =====================================================
 * NMT / SYNC
 * ===================================================== */

void canopen_nmt_build(uint8_t cmd, uint8_t node, canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_NMT;
    f->dlc   = 2;
    f->is_fd = false;
    f->data[0] = cmd;
    f->data[1] = node;
}

void canopen_sync_build(canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_SYNC;
    f->dlc   = 0;
    f->is_fd = false;
}

/* =====================================================
 * 自定义单轴 PDO
 * ===================================================== */

void canopen_custom_pdo_build(uint8_t node, motor_mode_t mode,
                               bool enable, bool release_brake, bool clear_err,
                               int16_t target1, uint16_t target2,
                               int16_t feedforward,
                               canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_PDO_CTRL_BASE + node;
    f->dlc   = 7;
    f->is_fd = true;

    uint8_t flags = 0;
    if (enable)        flags |= (1 << 7);
    if (release_brake) flags |= (1 << 6);
    if (clear_err)     flags |= (1 << 5);
    flags |= ((uint8_t)mode & 0x0F) << 1;

    f->data[0] = flags;
    f->data[1] = (uint8_t)(target1 & 0xFF);
    f->data[2] = (uint8_t)((target1 >> 8) & 0xFF);
    f->data[3] = (uint8_t)(target2 & 0xFF);
    f->data[4] = (uint8_t)((target2 >> 8) & 0xFF);
    f->data[5] = (uint8_t)(feedforward & 0xFF);
    f->data[6] = (uint8_t)((feedforward >> 8) & 0xFF);
}

/* =====================================================
 * MIT 模式 PDO
 * ===================================================== */

void canopen_mit_pdo_build(uint8_t node, motor_mode_t mode,
                            bool enable, bool release_brake, bool clear_err,
                            uint16_t position, uint16_t velocity,
                            uint16_t kp, uint16_t kd, uint16_t torque,
                            canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_PDO_MIT_BASE + node;
    f->dlc   = 9;
    f->is_fd = true;

    uint8_t flags = 0;
    if (enable)        flags |= (1 << 7);
    if (release_brake) flags |= (1 << 6);
    if (clear_err)     flags |= (1 << 5);
    flags |= ((uint8_t)mode & 0x0F) << 1;

    f->data[0] = flags;
    f->data[1] = (uint8_t)(position & 0xFF);
    f->data[2] = (uint8_t)((position >> 8) & 0xFF);
    f->data[3] = (uint8_t)(velocity & 0xFF);
    f->data[4] = (uint8_t)(((velocity >> 8) & 0x0F) | ((kp & 0x0F) << 4));
    f->data[5] = (uint8_t)((kp >> 4) & 0xFF);
    f->data[6] = (uint8_t)(kd & 0xFF);
    f->data[7] = (uint8_t)((kd >> 8) & 0xFF);
    f->data[8] = (uint8_t)(torque & 0xFF);
}

/* =====================================================
 * 多轴广播
 * ===================================================== */

void canopen_multi_ctrl_build(const multi_axis_cmd_t *cmds, uint8_t count,
                               canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_MULTI_CTRL;
    f->dlc   = 64;
    f->is_fd = true;

    if (count > 8) count = 8;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t base = i * 7;

        uint8_t flags = 0;
        if (cmds[i].enable)        flags |= (1 << 7);
        if (cmds[i].release_brake) flags |= (1 << 6);
        if (cmds[i].clear_error)   flags |= (1 << 5);
        flags |= ((uint8_t)cmds[i].mode & 0x0F) << 1;

        f->data[base + 0] = flags;
        f->data[base + 1] = (uint8_t)(cmds[i].target1 & 0xFF);
        f->data[base + 2] = (uint8_t)((cmds[i].target1 >> 8) & 0xFF);
        f->data[base + 3] = (uint8_t)(cmds[i].target2 & 0xFF);
        f->data[base + 4] = (uint8_t)((cmds[i].target2 >> 8) & 0xFF);
        f->data[base + 5] = (uint8_t)(cmds[i].feedforward & 0xFF);
        f->data[base + 6] = (uint8_t)((cmds[i].feedforward >> 8) & 0xFF);
    }

    /* Byte[56-63]: ID 映射 */
    for (uint8_t i = 0; i < count; i++) {
        f->data[56 + i] = cmds[i].node_id;
    }
}

/* =====================================================
 * 反馈帧解析
 * ===================================================== */

void canopen_parse_feedback(const canfd_frame_t *f, motor_feedback_t *fb)
{
    memset(fb, 0, sizeof(*fb));
    fb->timestamp_us = _now_us();

    if (!f || f->dlc < 12) return;

    /* Byte[0-1]: 位置 (int16, 小端) */
    fb->position = (int16_t)((uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8));

    /* Byte[2-3]: 速度 (int16, 小端, RPM) */
    fb->velocity = (int16_t)((uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8));

    /* Byte[4-5]: Iq 电流 (int16, 小端, mA) */
    fb->current_iq = (int16_t)((uint16_t)f->data[4] | ((uint16_t)f->data[5] << 8));

    /* Byte[6-7]: 错误码 */
    fb->error_code = (uint16_t)f->data[6] | ((uint16_t)f->data[7] << 8);

    /* Byte[8-9]: 温度 (int16, 小端, 0.1°C) */
    fb->temperature = (int16_t)((uint16_t)f->data[8] | ((uint16_t)f->data[9] << 8));

    /* Byte[10]: 控制模式 */
    fb->mode = f->data[10];

    /* Byte[11]: 状态字节 */
    fb->status_byte = f->data[11];
}
