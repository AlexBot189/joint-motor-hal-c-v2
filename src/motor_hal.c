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
#include "pdo_mapper.h"

#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>

/* ---------- 诊断: CAN 帧 hex dump (仅调试模式) ---------- */
#ifdef MOTOR_DEBUG_HEX
static void _dump_can_frame(const char *dir, const canfd_frame_t *f)
{
    fprintf(stderr, "[%s] id=0x%03X dlc=%d :", dir, f->id, f->dlc);
    for (int i = 0; i < f->dlc && i < 64; i++) {
        fprintf(stderr, " %02X", f->data[i]);
    }
    fprintf(stderr, "\n");
}
#else
static inline void _dump_can_frame(const char *dir, const canfd_frame_t *f) { (void)dir; (void)f; }
#endif

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
int motor_startup_wait_bootup(can_driver_t *drv, uint8_t node_id, int timeout_ms,
                              const volatile bool *bootup_flag);
int motor_startup_enable(can_driver_t *drv, uint8_t node_id);
int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg,
                       const volatile bool *bootup_flag);

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
    motor_sensor_cb_t   sensor_cb;
    void               *fb_ctx;
    void               *err_ctx;
    void               *state_ctx;
    void               *sensor_ctx;

    /* 传感器缓存 */
    pthread_mutex_t  sensor_lock;
    motor_sensor_t   cached_sensor;
    uint64_t         last_sensor_us;
} motor_node_t;

/* =====================================================
 * HAL 主结构
 * ===================================================== */

struct motor_hal {
    can_driver_t *drv;
    bool          initialized;

    /* 接收线程 */
    pthread_t    recv_thread;
    bool         recv_running;

    /* SYNC 定时器线程 */
    pthread_t    sync_thread;
    bool         sync_running;
    uint32_t     sync_period_us;

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

    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&hal->lock, &ma);
    hal->motor_count = 0;

    for (int i = 0; i < MOTOR_HAL_MAX_MOTORS; i++) {
        hal->motors[i].node_id = 0;
        hal->motors[i].state = MOTOR_STATE_NOT_READY;
        pthread_mutex_init(&hal->motors[i].fb_lock, &ma);
        pthread_mutex_init(&hal->motors[i].sensor_lock, &ma);
    }
    pthread_mutexattr_destroy(&ma);

    /* 初始化 SDO 响应队列 */
    sdo_queue_init();

    return hal;
}

void motor_hal_destroy(motor_hal_t *hal)
{
    if (!hal) return;

    /* 1. 先脱使能所有电机 (需要 recv 线程运行才能收到 SDO 响应) */
    for (int i = 0; i < hal->motor_count; i++) {
        if (hal->motors[i].enabled) {
            sdo_write_simple(hal->drv, hal->motors[i].node_id,
                             OD_CONTROLWORD, 0x00, CW_DISABLE_VOLTAGE, 2);
        }
    }

    /* 2. 停止接收线程 */
    if (hal->recv_running) {
        hal->recv_running = false;
        pthread_join(hal->recv_thread, NULL);
    }

    /* 2.5 停止 SYNC 定时器 */
    if (hal->sync_running) {
        hal->sync_running = false;
        pthread_join(hal->sync_thread, NULL);
    }

    /* 3. 销毁 mutex */
    for (int i = 0; i < hal->motor_count; i++) {
        pthread_mutex_destroy(&hal->motors[i].fb_lock);
        pthread_mutex_destroy(&hal->motors[i].sensor_lock);
    }

    /* 4. 关闭 CAN */
    if (hal->drv) {
        can_driver_close(hal->drv);
        hal->drv = NULL;
    }

    /* 5. 销毁 SDO 队列 */
    sdo_queue_destroy();

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
    pthread_mutex_destroy(&hal->motors[idx].sensor_lock);

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

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }

    /* 用调用者传入的 timeout_ms 覆盖 cfg 中的默认值 */
    m->config.bootup_timeout_ms = timeout_ms;
    int ret = motor_startup_full(hal->drv, &m->config, &m->bootup_received);
    if (ret == 0) {
        m->bootup_received = true;
        m->enabled = true;
        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_wait_bootup(motor_hal_t *hal, uint8_t node_id, int timeout_ms)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    if (m->bootup_received) { pthread_mutex_unlock(&hal->lock); return 0; }

    int ret = motor_startup_wait_bootup(hal->drv, node_id, timeout_ms, &m->bootup_received);
    if (ret == 0) m->bootup_received = true;
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_enable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }

    int ret = motor_startup_enable(hal->drv, node_id);
    if (ret == 0) {
        m->enabled = true;
        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_disable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }

    int ret = sdo_write_simple(hal->drv, node_id,
                               OD_CONTROLWORD, 0x00, CW_SHUTDOWN, 2);
    if (ret == 0) {
        m->enabled = false;
        _set_state(hal, m, MOTOR_STATE_READY_TO_SW_ON);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_fault_reset(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    int ret = sdo_write_simple(hal->drv, node_id,
                               OD_CONTROLWORD, 0x00, CW_FAULT_RESET, 2);
    if (ret == 0) {
        motor_node_t *m = _find_motor(hal, node_id);
        if (m) _set_state(hal, m, MOTOR_STATE_SWITCH_ON_DIS);
    }
    pthread_mutex_unlock(&hal->lock);
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
    uint16_t vel  = (uint16_t)((int16_t)velocity);  /* 保留符号, 由电机按有符号12bit解析 */
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

int motor_hal_set_brake(motor_hal_t *hal, uint8_t node_id, bool release)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;
    if (!m->enabled) return -EAGAIN;

    /* 通过 PDO data[0] bit6 控制抱闸, 保持当前模式和使能状态 */
    pdo_ctrl_send(hal->drv, node_id, MOTOR_MODE_PROFILE_POS,
                  true, release, false, 0, 0, 0);
    return 0;
}

int motor_hal_quick_stop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id,
                            OD_CONTROLWORD, 0x00, CW_QUICK_STOP, 2);
}

/* =====================================================
 * 公共 API: 模式 / 参数 / PID
 * ===================================================== */

int motor_hal_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode)
{
    if (!hal || !hal->drv) return -ENODEV;

    /* 巨蟹协议: PDO mode flag 和 SDO 0x6060 是两套编码 */
    static const uint8_t sdo_mode_map[] = {
        [MOTOR_MODE_PROFILE_POS] = 0x01,  /* PP:      PDO=1, SDO=0x01 ✓ */
        [MOTOR_MODE_PROFILE_VEL] = 0x03,  /* PV:      PDO=2, SDO=0x03 */
        [MOTOR_MODE_CSP]         = 0x03,  /* CSP:     PDO=3, SDO=0x03 (暂用) */
        [MOTOR_MODE_CSV]         = 0x04,  /* CSV:     PDO=4, SDO=0x04 (暂用) */
        [MOTOR_MODE_CURRENT]     = 0x0A,  /* 电流环:  PDO=5, SDO=0x0A */
        [MOTOR_MODE_MIT]         = 0x06,  /* MIT:     PDO=6, SDO=0x06 (自定义) */
    };

    if (mode >= sizeof(sdo_mode_map)) return -EINVAL;
    return sdo_write_simple(hal->drv, node_id, OD_MODE_OF_OP, 0x00,
                            sdo_mode_map[mode], 1);
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

int motor_hal_set_pos_ctrl(motor_hal_t *hal, uint8_t node_id, bool start)
{
    if (!hal || !hal->drv) return -ENODEV;
    /* 0x4F=启动绝对位置运动, 0x0F=停止 */
    uint32_t cw = start ? 0x4FU : 0x0FU;
    return sdo_write_simple(hal->drv, node_id, OD_CONTROLWORD, 0x00, cw, 3);
}

int motor_hal_set_pos_target(motor_hal_t *hal, uint8_t node_id, int32_t target_counts)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_TARGET_POS, 0x00,
                            (uint32_t)target_counts, 4);
}

int motor_hal_set_speed_target(motor_hal_t *hal, uint8_t node_id, int32_t target_rpm)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_TARGET_VELOCITY, 0x00,
                            (uint32_t)target_rpm, 4);
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
    /* 巨蟹 0x6078 返回 2 字节 int16, 需符号扩展 */
    return (int32_t)(int16_t)(val & 0xFFFF);
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

int motor_hal_sdo_write(motor_hal_t *hal, uint8_t node_id,
                        uint16_t index, uint8_t subidx,
                        uint32_t value, uint8_t size)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, index, subidx, value, size);
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

void motor_hal_set_sensor_cb(motor_hal_t *hal, uint8_t node_id,
                             motor_sensor_cb_t cb, void *ctx)
{
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return;
    m->sensor_cb  = cb;
    m->sensor_ctx = ctx;
}

/* =====================================================
 * 内部: 帧分发 (接收线程 + poll 共用)
 * ===================================================== */

/* =====================================================
 * 内部: 传感器数据解析 (8字节, 小端, bit-packed)
 * ===================================================== */

static void _parse_sensor_frame(const canfd_frame_t *f, motor_sensor_t *s)
{
    memset(s, 0, sizeof(*s));
    if (!f || f->dlc < 8) return;

    /* 拼成 uint64 (little-endian) */
    uint64_t p = 0;
    for (int i = 0; i < 8; i++) {
        p |= ((uint64_t)f->data[i]) << (8U * i);
    }

    s->hall_adc0   = (uint16_t)((p >> 0)  & 0x0FFFU);
    s->hall_adc1   = (uint16_t)((p >> 12) & 0x0FFFU);
    s->hall_adc2   = (uint16_t)((p >> 24) & 0x0FFFU);
    s->force_raw   = (uint16_t)((p >> 36) & 0x3FFFU);
    s->knee_adc    = (uint16_t)((p >> 50) & 0x0FFFU);
    s->hw_sw_pc9   = (uint8_t)((p >> 62) & 0x01U);
    s->data_valid  = (uint8_t)((p >> 63) & 0x01U);
}

static void _dispatch_frame(motor_hal_t *hal, const canfd_frame_t *f)
{
    uint32_t func = canopen_func_code(f->id);

    _dump_can_frame("recv", f);

    switch (func) {
    case 0x580: {  /* SDO 响应 → 入队, 等待 sdo_client 消费 */
        sdo_push_response(f);
        break;
    }

    case 0x700: {  /* Bootup / Heartbeat */
        if (canopen_is_bootup(f->id, f->data[0])) {
            uint8_t node = canopen_extract_node(f->id, COB_BOOTUP_BASE);
            motor_node_t *m = _find_motor(hal, node);
            if (m) {
                m->bootup_received = true;
                fprintf(stderr, "  → Bootup node=%d\n", node);
                /* 自动启动 (如果配置了 auto_enable) */
                if (m->config.auto_enable && m->state == MOTOR_STATE_NOT_READY) {
                    fprintf(stderr, "  → auto-startup node=%d...\n", node);
                    int ret = motor_startup_full(hal->drv, &m->config, &m->bootup_received);
                    if (ret == 0) {
                        m->enabled = true;
                        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
                        fprintf(stderr, "  → node=%d OPERATION_ENABLED (auto)\n", node);
                    } else {
                        fprintf(stderr, "  → node=%d auto-startup failed (ret=%d)\n", node, ret);
                    }
                }
            } else {
                fprintf(stderr, "  → WARN: Bootup node=%d not registered\n", node);
            }
        } else {
            const char *st = motor_utils_nmt_state_str(f->data[0]);
            fprintf(stderr, "  → Heartbeat state=%s (0x%02X)\n",
                    st ? st : "?", f->data[0]);
        }
        break;
    }

    case 0x300: {  /* 反馈帧 (巨蟹私有) */
        uint8_t node = canopen_extract_node(f->id, COB_FEEDBACK_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        motor_feedback_t fb;
        pdo_feedback_parse(f, &fb);

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
        if (fb.status_byte & 0x20) {
            if (m->err_cb) {
                m->err_cb(node, fb.error_code, m->err_ctx);
            }
        }
        break;
    }

    case 0x180: {  /* 标准 TPDO1 (同步周期上报: 0x180+node) */
        uint8_t node = (uint8_t)(f->id & 0x7F);
        motor_node_t *m = _find_motor(hal, node);
        if (!m || f->dlc < 8) break;

        /* 解析: Statusword(16b) + Position(32b) + Velocity(32b) = 10 bytes */
        uint16_t sw = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        int32_t  pos = (int32_t)((uint32_t)f->data[2]
                      | ((uint32_t)f->data[3] << 8)
                      | ((uint32_t)f->data[4] << 16)
                      | ((uint32_t)f->data[5] << 24));
        /* 如果 TPDO 包含了 Velocity + Current (12 bytes), 也解析 */
        int32_t  vel = 0;
        int16_t  cur = 0;
        if (f->dlc >= 12) {
            vel = (int32_t)((uint32_t)f->data[6]
                  | ((uint32_t)f->data[7] << 8)
                  | ((uint32_t)f->data[8] << 16)
                  | ((uint32_t)f->data[9] << 24));
            cur = (int16_t)((uint16_t)f->data[10] | ((uint16_t)f->data[11] << 8));
        }

        /* 更新缓存 (补充 TPDO 数据到缓存) */
        pthread_mutex_lock(&m->fb_lock);
        m->cached_fb.position   = (int16_t)pos;  /* 截断到 16bit, 兼容现有类型 */
        m->cached_fb.velocity   = (int16_t)vel;
        m->cached_fb.current_iq = cur;
        m->cached_fb.timestamp_us = motor_utils_now_us();
        /* 从 statusword 推导状态 */
        m->cached_fb.status_byte = (sw & 0x000F) |  /* 低4位=状态 */
                                    ((sw & 0x1000) ? 0 : 0x80);  /* bit12=0→enabled */
        pthread_mutex_unlock(&m->fb_lock);

        /* 触发反馈回调 */
        if (m->fb_cb) {
            m->fb_cb(node, &m->cached_fb, m->fb_ctx);
        }
        break;
    }

    case 0x080: {  /* EMCY 紧急报文 */
        uint8_t node = canopen_extract_node(f->id, COB_EMCY_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        uint16_t err = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        if (m->err_cb) {
            m->err_cb(node, err, m->err_ctx);
        }
        break;
    }

    case 0x680: {  /* 传感器透传 (0x680 + node_id) */
        uint8_t node = canopen_extract_node(f->id, COB_SENSOR_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        motor_sensor_t s;
        _parse_sensor_frame(f, &s);
        s.timestamp_us = motor_utils_now_us();

        pthread_mutex_lock(&m->sensor_lock);
        memcpy(&m->cached_sensor, &s, sizeof(s));
        m->last_sensor_us = s.timestamp_us;
        pthread_mutex_unlock(&m->sensor_lock);

        if (m->sensor_cb) {
            m->sensor_cb(node, &s, m->sensor_ctx);
        }
        break;
    }

    default:
        break;
    }
}

/* =====================================================
 * 接收线程 (唯一 recv 入口)
 * ===================================================== */

static void* _recv_thread_fn(void *arg)
{
    motor_hal_t *hal = (motor_hal_t*)arg;

    while (hal->recv_running) {
        canfd_frame_t f;
        int ret = can_driver_recv(hal->drv, &f, 100);
        if (ret <= 0) continue;
        _dispatch_frame(hal, &f);
    }

    return NULL;
}

/* =====================================================
 * 公共 API: 接收线程控制
 * ===================================================== */

int motor_hal_recv_start(motor_hal_t *hal)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (hal->recv_running) return -EBUSY;

    hal->recv_running = true;

    int ret = pthread_create(&hal->recv_thread, NULL, _recv_thread_fn, hal);
    if (ret != 0) {
        hal->recv_running = false;
        return -ret;
    }

    return 0;
}

int motor_hal_recv_stop(motor_hal_t *hal)
{
    if (!hal || !hal->recv_running) return -EINVAL;

    hal->recv_running = false;
    pthread_join(hal->recv_thread, NULL);

    return 0;
}

bool motor_hal_recv_is_running(motor_hal_t *hal)
{
    return hal && hal->recv_running;
}

/* =====================================================
 * 公共 API: 轮询 (向前兼容, 接收线程启动后无需调用)
 * ===================================================== */

void motor_hal_poll(motor_hal_t *hal, int timeout_ms)
{
    if (!hal || !hal->drv || hal->recv_running) return;

    canfd_frame_t f;
    int ret = can_driver_recv(hal->drv, &f, timeout_ms);
    if (ret <= 0) return;
    _dispatch_frame(hal, &f);
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

/* =====================================================
 * SYNC 定时器线程
 * ===================================================== */

static void* _sync_thread_fn(void *arg)
{
    motor_hal_t *hal = (motor_hal_t*)arg;
    uint32_t period_us = hal->sync_period_us;

    while (hal->sync_running) {
        pdo_sync_send(hal->drv);
        usleep(period_us);
    }
    return NULL;
}

int motor_hal_sync_start(motor_hal_t *hal, uint32_t period_us)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (hal->sync_running) return -EBUSY;
    if (period_us < 500 || period_us > 1000000) return -EINVAL;

    hal->sync_period_us = period_us;
    hal->sync_running = true;

    int ret = pthread_create(&hal->sync_thread, NULL, _sync_thread_fn, hal);
    if (ret != 0) {
        hal->sync_running = false;
        return -ret;
    }

    fprintf(stderr, "[SYNC] started: period=%u us (%.1f Hz)\n",
            period_us, 1000000.0f / (float)period_us);
    return 0;
}

int motor_hal_sync_stop(motor_hal_t *hal)
{
    if (!hal || !hal->sync_running) return -EINVAL;

    hal->sync_running = false;
    pthread_join(hal->sync_thread, NULL);

    fprintf(stderr, "[SYNC] stopped\n");
    return 0;
}

bool motor_hal_sync_is_running(motor_hal_t *hal)
{
    return hal && hal->sync_running;
}

/* =====================================================
 * TPDO 配置 — 同步周期上报
 * ===================================================== */

int motor_hal_tpdo_config(motor_hal_t *hal, uint8_t node_id, uint8_t sync_count)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (sync_count == 0 || sync_count > 240) return -EINVAL;

    uint32_t cob_id = (uint32_t)PDO_TPDO1_COB(node_id);
    int ret;

    /* 1. 停用 TPDO1 */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_COMM, 0x01,
                           cob_id | 0x80000000UL, 4);
    if (ret != 0) { fprintf(stderr, "[TPDO] node=%d disable failed\n", node_id); return ret; }

    /* 2. 清映射 */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_MAP, 0x00, 0, 1);
    if (ret != 0) return ret;

    /* 3. 写入映射: Statusword(0x6041, 16b) */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_MAP, 0x01,
                           pdo_map_entry(OD_STATUSWORD, 0x00, 16), 4);
    if (ret != 0) return ret;

    /* 4. Position Actual (0x6064, 32b) */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_MAP, 0x02,
                           pdo_map_entry(OD_POSITION_ACTUAL, 0x00, 32), 4);
    if (ret != 0) return ret;

    /* 5. Velocity Actual (0x606C, 32b) */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_MAP, 0x03,
                           pdo_map_entry(OD_VELOCITY_ACTUAL, 0x00, 32), 4);
    if (ret != 0) return ret;

    /* 6. Current Actual (0x6078, 16b) */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_MAP, 0x04,
                           pdo_map_entry(OD_CURRENT_ACTUAL, 0x00, 16), 4);
    if (ret != 0) return ret;

    /* 7. 设映射数量 = 4 */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_MAP, 0x00, 4, 1);
    if (ret != 0) return ret;

    /* 8. 设传输类型 = sync_count (同步周期) */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_COMM, 0x02,
                           sync_count, 1);
    if (ret != 0) return ret;

    /* 9. 启用 TPDO1 */
    ret = sdo_write_simple(hal->drv, node_id, OD_TPDO1_COMM, 0x01, cob_id, 4);
    if (ret != 0) return ret;

    fprintf(stderr, "[TPDO] node=%d configured: COB=0x%03X, sync_every=%d\n",
            node_id, cob_id, sync_count);
    return 0;
}

void motor_hal_multi_ctrl(motor_hal_t *hal, const multi_axis_cmd_t *cmds, uint8_t count)
{
    if (!hal || !hal->drv) return;
    pdo_multi_send(hal->drv, cmds, count);
}

/* =====================================================
 * 公共 API: 传感器透传控制
 * ===================================================== */

int motor_hal_sensor_config(motor_hal_t *hal, uint8_t node_id,
                            uint16_t period_div, uint8_t bus_format)
{
    if (!hal || !hal->drv) return -ENODEV;

    uint32_t cfg = (uint32_t)(period_div & 0xFFFF)
                 | ((uint32_t)(bus_format & 0x03) << 16);

    return sdo_write_simple(hal->drv, node_id, OD_SENSOR_CONFIG,
                            OD_SENSOR_CONFIG_SUB, cfg, 4);
}

int motor_hal_sensor_stop(motor_hal_t *hal, uint8_t node_id)
{
    return motor_hal_sensor_config(hal, node_id, 0, 0);
}

int motor_hal_get_sensor(motor_hal_t *hal, uint8_t node_id, motor_sensor_t *s)
{
    if (!hal || !s) return -EINVAL;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    pthread_mutex_lock(&m->sensor_lock);
    memcpy(s, &m->cached_sensor, sizeof(motor_sensor_t));
    pthread_mutex_unlock(&m->sensor_lock);

    return 0;
}
