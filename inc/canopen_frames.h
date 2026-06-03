/**
 * @file canopen_frames.h
 * @brief CANopen 帧构造与解析工具箱
 *
 * 涵盖: SDO / NMT / SYNC / 自定义PDO / MIT PDO / 多轴广播 / 反馈帧
 *
 * 所有帧封装为纯函数, 无状态依赖。
 */

#ifndef CANOPEN_FRAMES_H
#define CANOPEN_FRAMES_H

#include "motor_hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================
 * SDO 帧构造
 * ===================================================== */

/**
 * @brief 构造 SDO 写请求帧
 * @param node      节点 ID
 * @param index     对象字典索引
 * @param subidx    子索引
 * @param value     写入值
 * @param size_bytes 数据大小 (1, 2, 4)
 * @param f         [out] CANFD 帧
 */
void canopen_sdo_write_build(uint8_t node, uint16_t index, uint8_t subidx,
                             uint32_t value, uint8_t size_bytes,
                             canfd_frame_t *f);

/**
 * @brief 构造 SDO 读请求帧
 */
void canopen_sdo_read_build(uint8_t node, uint16_t index, uint8_t subidx,
                            canfd_frame_t *f);

/**
 * @brief 解析 SDO 响应帧
 * @return true=成功, false=错误 (检查 abort_code)
 */
bool canopen_sdo_parse_response(const canfd_frame_t *f,
                                uint32_t *value,
                                uint16_t *out_index,
                                uint8_t *out_subidx,
                                uint32_t *abort_code);

/* =====================================================
 * NMT 帧
 * ===================================================== */

/**
 * @param cmd  NMT 命令码
 * @param node 目标节点, 0=广播
 */
void canopen_nmt_build(uint8_t cmd, uint8_t node, canfd_frame_t *f);

/* =====================================================
 * SYNC 帧
 * ===================================================== */

void canopen_sync_build(canfd_frame_t *f);

/* =====================================================
 * 自定义单轴 PDO (7字节, COB=0x100+ID)
 * ===================================================== */

/**
 * @brief 构造自定义单轴 PDO 控制帧
 *
 * Byte[0]: [7]Enable [6]Brake [5]ClearErr [4:1]Mode [0]Rsvd
 * Byte[1-2]: target1   (int16, 位置/速度/电流)
 * Byte[3-4]: target2   (uint16, 加减速)
 * Byte[5-6]: feedforward (int16, 前馈)
 */
void canopen_custom_pdo_build(uint8_t node, motor_mode_t mode,
                              bool enable, bool release_brake, bool clear_err,
                              int16_t target1, uint16_t target2,
                              int16_t feedforward,
                              canfd_frame_t *f);

/* =====================================================
 * MIT 模式 PDO (9字节, COB=0x110+ID)
 * ===================================================== */

void canopen_mit_pdo_build(uint8_t node, motor_mode_t mode,
                           bool enable, bool release_brake, bool clear_err,
                           uint16_t position, uint16_t velocity,
                           uint16_t kp, uint16_t kd, uint16_t torque,
                           canfd_frame_t *f);

/* =====================================================
 * 多轴广播 (64字节, COB=0x200)
 * ===================================================== */

/**
 * @param cmds   命令数组 (最多8个)
 * @param count  命令数量
 * @param f      [out] CANFD 帧
 */
void canopen_multi_ctrl_build(const multi_axis_cmd_t *cmds, uint8_t count,
                              canfd_frame_t *f);

/* =====================================================
 * 反馈帧解析 (12字节, COB=0x300+ID)
 * ===================================================== */

/**
 * @brief 解析驱动板上报的反馈帧
 *
 * Byte[0-1]:  负载端位置 (int16, 小端)
 * Byte[2-3]:  电机端速度 (int16, 小端, RPM)
 * Byte[4-5]:  Iq 电流 (int16, 小端, mA)
 * Byte[6-7]:  错误码 (uint16, 小端)
 * Byte[8-9]:  线圈温度 (int16, 小端, 0.1°C)
 * Byte[10]:   控制模式
 * Byte[11]:   状态字节
 */
void canopen_parse_feedback(const canfd_frame_t *f, motor_feedback_t *fb);

/* =====================================================
 * 帧识别工具
 * ===================================================== */

static inline uint8_t canopen_extract_node(uint32_t can_id, uint32_t base) {
    return (uint8_t)(can_id - base);
}

static inline bool canopen_is_bootup(uint32_t can_id, uint8_t data0) {
    return ((can_id & 0x780) == COB_BOOTUP_BASE) && (data0 == 0x00);
}

static inline bool canopen_is_emcy(uint32_t can_id) {
    return (can_id & 0x780) == COB_EMCY_BASE;
}

static inline uint32_t canopen_func_code(uint32_t can_id) {
    return can_id & 0x780;
}

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_FRAMES_H */
