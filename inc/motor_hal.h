/**
 * @file motor_hal.h
 * @brief 关节模组 HAL 层 - 公共 API
 *
 * 纯 C 实现, 无外部依赖 (仅 Linux + SocketCAN)。
 *
 * 典型用法:
 * @code
 *   motor_hal_t *hal = motor_hal_create();
 *   motor_hal_init(hal, "can0");
 *
 *   motor_config_t cfg = { .node_id = 1, .disable_watchdog = true };
 *   motor_hal_add_motor(hal, &cfg);
 *
 *   motor_hal_startup(hal, 1, 5000);
 *
 *   while (running) {
 *       motor_hal_poll(hal, 1);
 *       motor_hal_set_position(hal, 1, 30.0f);
 *   }
 *
 *   motor_hal_destroy(hal);
 * @endcode
 */

#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include "motor_hal_types.h"
#include "canopen_frames.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * HAL 句柄 (不透明类型)
 * ============================================================================ */

typedef struct motor_hal motor_hal_t;

/* ============================================================================
 * 生命周期
 * ============================================================================ */

/**
 * @brief 创建 HAL 实例
 * @return 非 NULL = 成功
 */
motor_hal_t* motor_hal_create(void);

/**
 * @brief 销毁 HAL, 自动脱使能所有电机
 */
void motor_hal_destroy(motor_hal_t *hal);

/**
 * @brief 初始化 CAN 接口
 * @param hal          HAL 实例
 * @param iface        CAN 接口名 ("can0")
 * @param arb_bitrate  仲裁段速率 bps (默认 1000000)
 * @param data_bitrate 数据段速率 bps (默认 5000000)
 * @return 0=成功, <0=errno
 */
int motor_hal_init(motor_hal_t *hal, const char *iface,
                   uint32_t arb_bitrate, uint32_t data_bitrate);

/* ============================================================================
 * 电机管理
 * ============================================================================ */

/**
 * @brief 注册电机
 * @param hal  HAL 实例
 * @param cfg  电机配置 (拷贝语义)
 * @return 0=成功, <0=失败 (ID 重复或数量超限)
 */
int motor_hal_add_motor(motor_hal_t *hal, const motor_config_t *cfg);

/** 移除电机 */
void motor_hal_remove_motor(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 启动 & 关闭
 * ============================================================================ */

/**
 * @brief 完整启动流程
 *  1. 等待 Bootup 帧
 *  2. SDO 配置心跳
 *  3. 关看门狗 (可选)
 *  4. 读固件版本验证
 *  5. 使能 (走 DS402 状态机)
 *  6. 延时 100ms (抱闸释放)
 *
 * @param timeout_ms 总超时
 * @return 0=成功
 */
int motor_hal_startup(motor_hal_t *hal, uint8_t node_id, int timeout_ms);

/** 仅等待 Bootup */
int motor_hal_wait_bootup(motor_hal_t *hal, uint8_t node_id, int timeout_ms);

/** 使能电机 (Shutdown→SwitchOn→EnableOp) */
int motor_hal_enable(motor_hal_t *hal, uint8_t node_id);

/** 脱使能 */
int motor_hal_disable(motor_hal_t *hal, uint8_t node_id);

/** 故障复位 */
int motor_hal_fault_reset(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 实时控制 (自定义 PDO, 低延迟)
 * ============================================================================ */

/**
 * @brief 位置模式控制 (PP/CSP)
 * @param angle_deg 目标角度 (度, -180~180)
 */
int motor_hal_set_position(motor_hal_t *hal, uint8_t node_id, float angle_deg);

/** 速度模式控制 (PV/CSV), rpm_motor = 电机端 RPM */
int motor_hal_set_velocity(motor_hal_t *hal, uint8_t node_id, float rpm_motor);

/** 电流模式控制, current_ma = 目标 Iq 电流 (mA) */
int motor_hal_set_torque(motor_hal_t *hal, uint8_t node_id, int16_t current_ma);

/** MIT 阻抗控制 */
int motor_hal_mit_control(motor_hal_t *hal, uint8_t node_id,
                          float position, float velocity,
                          float kp, float kd, float torque);

/** 通用 PDO 控制 (指定模式) */
int motor_hal_ctrl_raw(motor_hal_t *hal, uint8_t node_id,
                       motor_mode_t mode,
                       int16_t target1, uint16_t target2, int16_t feedforward);

/** 停止运动 */
int motor_hal_stop(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 模式与参数配置 (SDO)
 * ============================================================================ */

int motor_hal_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode);
int motor_hal_set_accel_decel(motor_hal_t *hal, uint8_t node_id,
                              uint16_t accel, uint16_t decel);
int motor_hal_set_profile_velocity(motor_hal_t *hal, uint8_t node_id, uint16_t rpm_out);
int motor_hal_set_pid(motor_hal_t *hal, uint8_t node_id, const motor_pid_t *pid);
int motor_hal_save_flash(motor_hal_t *hal, uint8_t node_id);
int motor_hal_set_zero(motor_hal_t *hal, uint8_t node_id);
int motor_hal_set_limits(motor_hal_t *hal, uint8_t node_id, float pos_deg, float neg_deg);
int motor_hal_disable_watchdog(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 状态查询 (SDO)
 * ============================================================================ */

motor_state_t motor_hal_get_state(motor_hal_t *hal, uint8_t node_id);
uint16_t motor_hal_get_statusword(motor_hal_t *hal, uint8_t node_id);
int32_t motor_hal_get_position(motor_hal_t *hal, uint8_t node_id);
int32_t motor_hal_get_velocity(motor_hal_t *hal, uint8_t node_id);
int32_t motor_hal_get_current(motor_hal_t *hal, uint8_t node_id);
int motor_hal_read_pid(motor_hal_t *hal, uint8_t node_id, motor_pid_t *pid);

/* ============================================================================
 * 反馈缓存
 * ============================================================================ */

/** 获取最近一次反馈 (线程安全拷贝) */
int motor_hal_get_feedback(motor_hal_t *hal, uint8_t node_id, motor_feedback_t *fb);

/* ============================================================================
 * 回调注册
 * ============================================================================ */

void motor_hal_set_feedback_cb(motor_hal_t *hal, uint8_t node_id,
                               motor_feedback_cb_t cb, void *ctx);
void motor_hal_set_error_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_error_cb_t cb, void *ctx);
void motor_hal_set_state_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_state_cb_t cb, void *ctx);

/* ============================================================================
 * 轮询
 * ============================================================================ */

/**
 * @brief 轮询 CAN 帧, 自动分发
 * @param timeout_ms 单帧等待超时 (ms)
 *
 * 内部根据 COB-ID 自动路由:
 *   0x580  → 内部 SDO 等待队列
 *   0x300  → 解析反馈 → 更新缓存 → 触发回调
 *   0x700  → Bootup/Heartbeat 处理
 *   0x080  → EMCY 紧急处理
 *
 * 需在主循环中高频 (≥100Hz) 调用。
 */
void motor_hal_poll(motor_hal_t *hal, int timeout_ms);

/* ============================================================================
 * 全局控制
 * ============================================================================ */

/** 发送 NMT 广播 */
void motor_hal_nmt_broadcast(motor_hal_t *hal, uint8_t cmd);

/** 发送 SYNC 帧 */
void motor_hal_sync(motor_hal_t *hal);

/** 多轴广播控制 */
void motor_hal_multi_ctrl(motor_hal_t *hal, const multi_axis_cmd_t *cmds, uint8_t count);

/* ============================================================================
 * 工具函数
 * ============================================================================ */

/** 编码器 count → 角度 (°) */
static inline float motor_counts_to_deg(int16_t counts) {
    return (float)counts * 360.0f / (float)ENCODER_LOAD_RES;
}

/** 角度 (°) → 编码器 count */
static inline int16_t motor_deg_to_counts(float degrees) {
    return (int16_t)(degrees * ENCODER_LOAD_RES / 360.0f);
}

/** 温度 raw → °C */
static inline float motor_temp_to_c(int16_t raw) {
    return raw * 0.1f;
}

/** 电流 mA → A */
static inline float motor_ma_to_a(int16_t ma) {
    return ma * 0.001f;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HAL_H */
