/**
 * @file motor_hal.c
 * @brief 关节模组 HAL 层 - 核心实现
 *
 * 所有公共 API 的入口, 内部管理电机列表和 CAN 帧分发。
 *
 * 依赖模块:
 *   can_driver      → SocketCAN 驱动
 *   sdo_client      → SDO 读写
 *   pdo_handler     → PDO 发送
 *   feedback_parser → 反馈解析
 *   nmt_master      → NMT 命令
 *   heartbeat       → 心跳管理
 *   motor_hal_startup → 启动流程
 *   utils           → 工具函数
 */

#include "motor_hal.h"

#include "can_driver_internal.h"
#include "sdo_client_internal.h"

#include <errno.h>
#include <pthread.h>
#include <math.h>

/* =====================================================
 * 内部: 前向声明拆分模块函数
 * ===================================================== */

/* can_driver.c */
/* (通过 can_driver_internal.h) */

/* sdo_client.c */
/* (通过 sdo_client_internal.h) */

/* nmt_master.c */
void nmt_send(can_driver_t *drv, uint8_t cmd, uint8_t node);
void nmt_broadcast(can_driver_t *drv, uint8_t cmd);

/* heartbeat.c */
void heartbeat_set_period(can_driver_t *drv, uint8_t node, uint32_t ms);
void heartbeat_disable_watchdog(can_driver_t *drv, uint8_t node);
void heartbeat_feed(can_driver_t *drv);

/* pdo_handler.c */
void pdo_ctrl_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                   bool enable, bool release_brake, bool clear_err,
                   int16_t target1, uint16_t target2, int16_t feedforward);
void pdo_mit_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                  bool enable, bool release_brake, bool clear_err,
                  uint16_t position, uint16_t velocity,
                  uint16_t kp, uint16_t kd, uint16_t torque);
void pdo_multi_send(can_driver_t *drv, const multi_axis_cmd_t *cmds, uint8_t count);
void pdo_sync_send(can_driver_t *drv);
void pdo_feedback_parse(const canfd_frame_t *f, motor_feedback_t *fb);

/* feedback_parser.c */
const char* feedback_error_string(uint16_t err);

/* motor_hal_startup.c */
int motor_startup_wait_bootup(can_driver_t *drv, uint8_t node_id, int timeout_ms);
int motor_startup_enable(can_driver_t *drv, uint8_t node_id);
int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg);

/* utils.c */
uint64_t motor_utils_now_us(void);
void motor_utils_sleep_ms(int ms);

/* =====================================================
 * 单电机状态
 * ===================================================== */

typedef struct {
    uint8_t     node_id;
    motor_state_t state;
    bool        enabled;
    bool        bootup_received;
    motor_config_t config;

    /* 反馈缓存 */
    pthread_mutex_t fb_lock;
    motor_feedback_t cached_fb;
    uint64_t last_fb_us;

    /* 回调 */
    motor_feedback_cb_t fb_cb;
    motor_error_cb_t    err_cb;
    motor_state_cb_t    state_cb;
    void               *fb_ctx;
    void               *err_ctx;
    void               *state_ctx;
} motor_node_t;

/* =====================================================
 * HAL 主结构
 * ===================================================== */

struct motor_hal {
    can_driver_t *drv;
    bool          initialized;

    pthread_mutex_t lock;
    motor_node_t    motors[MOTOR_HAL_MAX_MOTORS];
    int             motor_count;
};

/* =====================================================
 * 内部: 查找电机
 * ===================================================== */

static motor_node_t* _find_motor(motor_hal_t *hal, uint8_t node_id)
{
    for (int i = 0; i < hal->motor_count; i++) {
        if (hal->motors[i].node_id == node_id)
            return &hal->motors[i];
    }
    return NULL;
}

/* =====================================================
 * 内部: 状态迁移
 * ===================================================== */

static void _set_state(motor_hal_t *hal __attribute__((unused)), motor_node_t *m, motor_state_t new_state)
{
    motor_state_t old = m->state;
    m->state = new_state;
    if (m->state_cb) {
        m->state_cb(m->node_id, old, new_state, m->state_ctx);
    }
}

/* =====================================================
 * 公共 API: 生命周期
 * ===================================================== */

motor_hal_t* motor_hal_create(void)
{
    motor_hal_t *hal = calloc(1, sizeof(motor_hal_t));
    if (!hal) return NULL;

    pthread_mutex_init(&hal->lock, NULL);
    hal->motor_count = 0;

    for (int i = 0; i < MOTOR_HAL_MAX_MOTORS; i++) {
        hal->motors[i].node_id = 0;
        hal->motors[i].state = MOTOR_STATE_NOT_READY;
        pthread_mutex_init(&hal->motors[i].fb_lock, NULL);
    }

    return hal;
}

void motor_hal_destroy(motor_hal_t *hal)
{
    if (!hal) return;

    /* 脱使能所有电机 */
    for (int i = 0; i < hal->motor_count; i++) {
        if (hal->motors[i].enabled) {
            motor_startup_enable(hal->drv, hal->motors[i].node_id);
            /* 写 Disable Voltage */
            sdo_write_simple(hal->drv, hal->motors[i].node_id,
                             OD_CONTROLWORD, 0x00, CW_DISABLE_VOLTAGE, 2);
        }
        pthread_mutex_destroy(&hal->motors[i].fb_lock);
    }

    if (hal->drv) {
        can_driver_close(hal->drv);
        hal->drv = NULL;
    }

    pthread_mutex_destroy(&hal->lock);
    free(hal);
}

int motor_hal_init(motor_hal_t *hal, const char *iface,
                   uint32_t arb_bitrate, uint32_t data_bitrate)
{
    if (!hal || hal->initialized) return -EBUSY;

    int ret = can_driver_open(iface, arb_bitrate, data_bitrate, &hal->drv);
    if (ret < 0) return ret;

    hal->initialized = true;
    return 0;
}

/* =====================================================
 * 公共 API: 电机管理
 * ===================================================== */

int motor_hal_add_motor(motor_hal_t *hal, const motor_config_t *cfg)
{
    if (!hal || !cfg) return -EINVAL;

    pthread_mutex_lock(&hal->lock);

    if (hal->motor_count >= MOTOR_HAL_MAX_MOTORS) {
        pthread_mutex_unlock(&hal->lock);
        return -ENOSPC;
    }

    /* 检查重复 */
    if (_find_motor(hal, cfg->node_id)) {
        pthread_mutex_unlock(&hal->lock);
        return -EEXIST;
    }

    motor_node_t *m = &hal->motors[hal->motor_count];
    m->node_id  = cfg->node_id;
    m->state    = MOTOR_STATE_NOT_READY;
    m->enabled  = false;
    m->bootup_received = false;
    memcpy(&m->config, cfg, sizeof(motor_config_t));

    hal->motor_count++;

    pthread_mutex_unlock(&hal->lock);
    return 0;
}

void motor_hal_remove_motor(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return;

    pthread_mutex_lock(&hal->lock);

    int idx = -1;
    for (int i = 0; i < hal->motor_count; i++) {
        if (hal->motors[i].node_id == node_id) { idx = i; break; }
    }
    if (idx < 0) { pthread_mutex_unlock(&hal->lock); return; }

    pthread_mutex_destroy(&hal->motors[idx].fb_lock);

    /* 紧凑数组 */
    if (idx < hal->motor_count - 1) {
        memmove(&hal->motors[idx], &hal->motors[idx + 1],
                sizeof(motor_node_t) * (hal->motor_count - idx - 1));
    }
    hal->motor_count--;

    pthread_mutex_unlock(&hal->lock);
}

/* =====================================================
 * 公共 API: 启动 / 关闭
 * ===================================================== */

int motor_hal_startup(motor_hal_t *hal, uint8_t node_id, int timeout_ms __attribute__((unused)))
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    int ret = motor_startup_full(hal->drv, &m->config);
    if (ret == 0) {
        m->bootup_received = true;
        m->enabled = true;
        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    }
    return ret;
}

int motor_hal_wait_bootup(motor_hal_t *hal, uint8_t node_id, int timeout_ms)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;
    if (m->bootup_received) return 0;

    int ret = motor_startup_wait_bootup(hal->drv, node_id, timeout_ms);
    if (ret == 0) m->bootup_received = true;
    return ret;
}

int motor_hal_enable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    int ret = motor_startup_enable(hal->drv, node_id);
    if (ret == 0) {
        m->enabled = true;
        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    }
    return ret;
}

int motor_hal_disable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    int ret = sdo_write_simple(hal->drv, node_id,
                               OD_CONTROLWORD, 0x00, CW_SHUTDOWN, 2);
    if (ret == 0) {
        m->enabled = false;
        _set_state(hal, m, MOTOR_STATE_READY_TO_SW_ON);
    }
    return ret;
}

int motor_hal_fault_reset(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    int ret = sdo_write_simple(hal->drv, node_id,
                               OD_CONTROLWORD, 0x00, CW_FAULT_RESET, 2);
    if (ret == 0) {
        motor_node_t *m = _find_motor(hal, node_id);
        if (m) _set_state(hal, m, MOTOR_STATE_SWITCH_ON_DIS);
    }
    return ret;
}

/* =====================================================
 * 公共 API: 实时控制
 * ===================================================== */

int motor_hal_set_position(motor_hal_t *hal, uint8_t node_id, float angle_deg)
{
    int16_t counts = motor_deg_to_counts(angle_deg);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;
    return motor_hal_ctrl_raw(hal, node_id, MOTOR_MODE_PROFILE_POS,
                              counts, m->config.profile_accel, 0);
}

int motor_hal_set_velocity(motor_hal_t *hal, uint8_t node_id, float rpm_motor)
{
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;
    return motor_hal_ctrl_raw(hal, node_id, MOTOR_MODE_PROFILE_VEL,
                              (int16_t)rpm_motor, m->config.profile_accel, 0);
}

int motor_hal_set_torque(motor_hal_t *hal, uint8_t node_id, int16_t current_ma)
{
    return motor_hal_ctrl_raw(hal, node_id, MOTOR_MODE_CURRENT,
                              current_ma, 0, 0);
}

int motor_hal_mit_control(motor_hal_t *hal, uint8_t node_id,
                          float position, float velocity,
                          float kp, float kd, float torque)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;
    if (!m->enabled) return -EAGAIN;

    uint16_t pos  = (uint16_t)(position * 65535.0f / 360.0f);
    uint16_t vel  = (uint16_t)fabsf(velocity);
    uint16_t kp_v = (uint16_t)(kp * 100.0f);
    uint16_t kd_v = (uint16_t)(kd * 100.0f);
    uint16_t torq = (uint16_t)(torque * 1000.0f);

    pdo_mit_send(hal->drv, node_id, MOTOR_MODE_MIT,
                 true, true, false,
                 pos, vel, kp_v, kd_v, torq);
    return 0;
}

int motor_hal_ctrl_raw(motor_hal_t *hal, uint8_t node_id,
                       motor_mode_t mode,
                       int16_t target1, uint16_t target2, int16_t feedforward)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;
    if (!m->enabled) return -EAGAIN;

    pdo_ctrl_send(hal->drv, node_id, mode, true, true, false,
                  target1, target2, feedforward);
    return 0;
}

int motor_hal_stop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m || !m->enabled) return 0;

    pdo_ctrl_send(hal->drv, node_id, MOTOR_MODE_PROFILE_POS,
                  true, true, false, 0, 0, 0);
    return 0;
}

/* =====================================================
 * 公共 API: 模式 / 参数 / PID
 * ===================================================== */

int motor_hal_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_MODE_OF_OP, 0x00, (uint32_t)mode, 1);
}

int motor_hal_set_accel_decel(motor_hal_t *hal, uint8_t node_id,
                              uint16_t accel, uint16_t decel)
{
    if (!hal || !hal->drv) return -ENODEV;
    int r1 = sdo_write_simple(hal->drv, node_id, OD_PROFILE_ACCEL, 0x00, accel, 4);
    int r2 = sdo_write_simple(hal->drv, node_id, OD_PROFILE_DECEL, 0x00, decel, 4);
    return (r1 == 0 && r2 == 0) ? 0 : -1;
}

int motor_hal_set_profile_velocity(motor_hal_t *hal, uint8_t node_id, uint16_t rpm_out)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_PROFILE_VEL, 0x00, rpm_out, 4);
}

int motor_hal_set_pid(motor_hal_t *hal, uint8_t node_id, const motor_pid_t *pid)
{
    if (!hal || !hal->drv || !pid) return -EINVAL;
    int ret = 0;
    ret |= sdo_write_simple(hal->drv, node_id, OD_CURRENT_P,  0x00, pid->current_p,  4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_CURRENT_I,  0x00, pid->current_i,  4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_VELOCITY_P, 0x00, pid->velocity_p, 4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_VELOCITY_I, 0x00, pid->velocity_i, 4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_POSITION_P, 0x00, pid->position_p, 4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_POSITION_I, 0x00, pid->position_i, 4);
    return ret;
}

int motor_hal_save_flash(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_SAVE_FLASH, 0x00, 1, 4);
}

int motor_hal_set_zero(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_ZERO_POSITION, 0x00, 1, 4);
}

int motor_hal_set_limits(motor_hal_t *hal, uint8_t node_id, float pos_deg, float neg_deg)
{
    if (!hal || !hal->drv) return -ENODEV;

    int32_t pos = (int32_t)(pos_deg * ENCODER_LOAD_RES / 360.0f);
    int32_t neg = (int32_t)(neg_deg * ENCODER_LOAD_RES / 360.0f);

    /* subindex: 0x01=负限位, 0x02=正限位 */
    int r1 = sdo_write_simple(hal->drv, node_id, OD_POS_LIMIT, 0x02,
                              (uint32_t)pos, 4);
    int r2 = sdo_write_simple(hal->drv, node_id, OD_POS_LIMIT, 0x01,
                              (uint32_t)neg, 4);
    return (r1 == 0 && r2 == 0) ? 0 : -1;
}

int motor_hal_disable_watchdog(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
}

/* =====================================================
 * 公共 API: 状态查询 (SDO)
 * ===================================================== */

motor_state_t motor_hal_get_state(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return MOTOR_STATE_UNKNOWN;

    uint32_t sw_raw = 0;
    if (sdo_read_simple(hal->drv, node_id, OD_STATUSWORD, 0x00, &sw_raw) != 0)
        return MOTOR_STATE_UNKNOWN;

    uint16_t sw = (uint16_t)(sw_raw & SW_STATE_MASK);

    if (sw & SW_FAULT)  return MOTOR_STATE_FAULT;
    if (sw & SW_QUICK_STOP) return MOTOR_STATE_QUICK_STOP;

    switch (sw) {
        case SW_NOT_READY:      return MOTOR_STATE_NOT_READY;
        case SW_ON_DISABLED:    return MOTOR_STATE_SWITCH_ON_DIS;
        case SW_READY_TO_SW_ON: return MOTOR_STATE_READY_TO_SW_ON;
        case SW_SWITCHED_ON:    return MOTOR_STATE_SWITCHED_ON;
        case SW_OP_ENABLED:     return MOTOR_STATE_OP_ENABLED;
        default: return MOTOR_STATE_UNKNOWN;
    }
}

uint16_t motor_hal_get_statusword(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_STATUSWORD, 0x00, &val);
    return (uint16_t)val;
}

int32_t motor_hal_get_position(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_POSITION_ACTUAL, 0x00, &val);
    return (int32_t)val;
}

int32_t motor_hal_get_velocity(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_VELOCITY_ACTUAL, 0x00, &val);
    return (int32_t)val;
}

int32_t motor_hal_get_current(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_CURRENT_ACTUAL, 0x00, &val);
    return (int32_t)val;
}

int motor_hal_read_pid(motor_hal_t *hal, uint8_t node_id, motor_pid_t *pid)
{
    if (!hal || !hal->drv || !pid) return -EINVAL;

    uint32_t v;
    sdo_read_simple(hal->drv, node_id, OD_CURRENT_P,  0x00, &v); pid->current_p  = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_CURRENT_I,  0x00, &v); pid->current_i  = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_VELOCITY_P, 0x00, &v); pid->velocity_p = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_VELOCITY_I, 0x00, &v); pid->velocity_i = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_POSITION_P, 0x00, &v); pid->position_p = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_POSITION_I, 0x00, &v); pid->position_i = (uint16_t)v;
    return 0;
}

int motor_hal_sdo_read_u32(motor_hal_t *hal, uint8_t node_id,
                           uint16_t index, uint8_t subidx, uint32_t *value)
{
    if (!hal || !hal->drv || !value) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, index, subidx, value);
}

/* =====================================================
 * 公共 API: 反馈缓存
 * ===================================================== */

int motor_hal_get_feedback(motor_hal_t *hal, uint8_t node_id, motor_feedback_t *fb)
{
    if (!hal || !fb) return -EINVAL;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    pthread_mutex_lock(&m->fb_lock);
    memcpy(fb, &m->cached_fb, sizeof(motor_feedback_t));
    pthread_mutex_unlock(&m->fb_lock);

    return 0;
}

/* =====================================================
 * 公共 API: 回调
 * ===================================================== */

void motor_hal_set_feedback_cb(motor_hal_t *hal, uint8_t node_id,
                               motor_feedback_cb_t cb, void *ctx)
{
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return;
    m->fb_cb  = cb;
    m->fb_ctx = ctx;
}

void motor_hal_set_error_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_error_cb_t cb, void *ctx)
{
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return;
    m->err_cb  = cb;
    m->err_ctx = ctx;
}

void motor_hal_set_state_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_state_cb_t cb, void *ctx)
{
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return;
    m->state_cb  = cb;
    m->state_ctx = ctx;
}

/* =====================================================
 * 公共 API: 轮询 (核心)
 * ===================================================== */

void motor_hal_poll(motor_hal_t *hal, int timeout_ms)
{
    if (!hal || !hal->drv) return;

    canfd_frame_t f;
    int ret = can_driver_recv(hal->drv, &f, timeout_ms);
    if (ret <= 0) return;

    uint32_t func = canopen_func_code(f.id);

    switch (func) {
    case 0x580: {  /* SDO 响应 */
        /* SDO 响应由 sdo_client 的 _sdo_wait_response 内部处理。
           这里处理异步的 SDO 响应 (正常不会到这里)。 */
        break;
    }

    case 0x700: {  /* Bootup / Heartbeat */
        if (canopen_is_bootup(f.id, f.data[0])) {
            uint8_t node = canopen_extract_node(f.id, COB_BOOTUP_BASE);
            motor_node_t *m = _find_motor(hal, node);
            if (m) m->bootup_received = true;
        }
        break;
    }

    case 0x300: {  /* 反馈帧 */
        uint8_t node = canopen_extract_node(f.id, COB_FEEDBACK_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        motor_feedback_t fb;
        pdo_feedback_parse(&f, &fb);

        /* 更新缓存 */
        pthread_mutex_lock(&m->fb_lock);
        memcpy(&m->cached_fb, &fb, sizeof(fb));
        m->last_fb_us = fb.timestamp_us;
        pthread_mutex_unlock(&m->fb_lock);

        /* 触发反馈回调 */
        if (m->fb_cb) {
            m->fb_cb(node, &fb, m->fb_ctx);
        }

        /* 检测错误 */
        if (fb.status_byte & 0x20) {  /* bit5 = error */
            if (m->err_cb) {
                m->err_cb(node, fb.error_code, m->err_ctx);
            }
        }
        break;
    }

    case 0x080: {  /* EMCY 紧急报文 */
        uint8_t node = canopen_extract_node(f.id, COB_EMCY_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        uint16_t err = (uint16_t)f.data[0] | ((uint16_t)f.data[1] << 8);
        if (m->err_cb) {
            m->err_cb(node, err, m->err_ctx);
        }
        break;
    }

    default:
        break;
    }
}

/* =====================================================
 * 公共 API: 全局控制
 * ===================================================== */

void motor_hal_nmt_broadcast(motor_hal_t *hal, uint8_t cmd)
{
    if (!hal || !hal->drv) return;
    nmt_broadcast(hal->drv, cmd);
}

void motor_hal_sync(motor_hal_t *hal)
{
    if (!hal || !hal->drv) return;
    pdo_sync_send(hal->drv);
}

void motor_hal_multi_ctrl(motor_hal_t *hal, const multi_axis_cmd_t *cmds, uint8_t count)
{
    if (!hal || !hal->drv) return;
    pdo_multi_send(hal->drv, cmds, count);
}
