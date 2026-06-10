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
void pdo_ctrl_send_raw(can_driver_t *drv, uint8_t node, uint8_t byte0,
                        int16_t target1, uint16_t target2, int16_t feedforward);
void pdo_mit_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                  bool enable, bool release_brake, bool clear_err,
                  uint16_t position, uint16_t velocity,
                  uint16_t kp, uint16_t kd, int16_t torque);
void pdo_mit_send_raw(can_driver_t *drv, uint8_t node, uint8_t byte0,
                       uint16_t position, uint16_t velocity,
                       uint16_t kp, uint16_t kd, int16_t torque);
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
    bool        pending_startup;   /* auto_enable 触发的待处理启动, 由主线程消费 */
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
    motor_tpdo_raw_cb_t tpdo_raw_cb;  /* 标准 TPDO 原始帧回调 */
    void               *fb_ctx;
    void               *err_ctx;
    void               *state_ctx;
    void               *sensor_ctx;
    void               *tpdo_raw_ctx;

    /* 传感器缓存 */
    pthread_mutex_t  sensor_lock;
    motor_sensor_t   cached_sensor;
    uint64_t         last_sensor_us;

    /* PDO Byte0 — 仅由 PDO API 管理, SDO 不碰 */
    uint8_t  pdo_byte0;         /* Byte0 持久值, 默认 0x00 */
    bool     clr_err_pending;   /* bit5 脉冲标志 */
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
 * 内部: 读 pdo_byte0 + clr_err 脉冲自动清除 (锁内调用)
 * ===================================================== */

static uint8_t _read_pdo_byte0(motor_node_t *m)
{
    uint8_t b0 = m->pdo_byte0;
    if (m->clr_err_pending) {
        b0 &= ~PDO_BYTE0_CLR_ERR;
        m->pdo_byte0 = b0;
        m->clr_err_pending = false;
    }
    return b0;
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
    for (int i = 0; i < MOTOR_HAL_MAX_MOTORS; i++) {
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
    m->pending_startup = false;
    m->pdo_byte0       = 0x00;  /* 默认全关, 算法显式调 pdo_enable 才开启 */
    m->clr_err_pending = false;
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
    if (!hal || !hal->drv) return -ENODEV;

    int16_t counts = motor_deg_to_counts(angle_deg);

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint16_t accel = m ? m->config.profile_accel : 0;
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;  /* 锁内读, clr_err脉冲自动清除 */
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, counts, accel, 0);
    return 0;
}

int motor_hal_set_velocity(motor_hal_t *hal, uint8_t node_id, float rpm_motor)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint16_t accel = m ? m->config.profile_accel : 0;
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, (int16_t)rpm_motor, accel, 0);
    return 0;
}

int motor_hal_set_torque(motor_hal_t *hal, uint8_t node_id, int16_t current_ma)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, current_ma, 0, 0);
    return 0;
}

int motor_hal_mit_control(motor_hal_t *hal, uint8_t node_id,
                          float position, float velocity,
                          float kp, float kd, float torque)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    uint16_t pos  = (uint16_t)((position + 180.0f) / 360.0f * 65535.0f);
    uint16_t vel  = (uint16_t)((int16_t)velocity);
    uint16_t kp_v = (uint16_t)(kp * 100.0f);
    uint16_t kd_v = (uint16_t)(kd * 100.0f);
    int16_t  torq = (int16_t)(torque * 1000.0f);

    pdo_mit_send_raw(hal->drv, node_id, b0, pos, vel, kp_v, kd_v, torq);
    return 0;
}

int motor_hal_ctrl_raw(motor_hal_t *hal, uint8_t node_id,
                       motor_mode_t mode,
                       int16_t target1, uint16_t target2, int16_t feedforward)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    /* 传入 mode 更新 Byte0 mode 字段, 保持其他 bit 不变 */
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;
    if (m) { b0 = (b0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(mode); m->pdo_byte0 = b0; }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, target1, target2, feedforward);
    return 0;
}

int motor_hal_stop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;
    pthread_mutex_unlock(&hal->lock);

    if (!m || !enabled) return 0;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, 0, 0, 0);
    return 0;
}

int motor_hal_set_brake(motor_hal_t *hal, uint8_t node_id, bool release)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _read_pdo_byte0(m) : 0;
    if (m) {
        if (release) b0 |=  PDO_BYTE0_BUS_ON;
        else         b0 &= ~PDO_BYTE0_BUS_ON;
        m->pdo_byte0 = b0;
    }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    /* bit6 母线电压 (当前电机不实现机械抱闸, 预留) */
    pdo_ctrl_send_raw(hal->drv, node_id, b0, 0, 0, 0);
    return 0;
}

int motor_hal_quick_stop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id,
                            OD_CONTROLWORD, 0x00, CW_QUICK_STOP, 2);
}

/* =====================================================
 * 公共 API: PDO Byte0 — 实时控制字节
 *
 *   SDO 不管 PDO: startup/enable/disable 不设 pdo_byte0
 *   PDO 不管 SDO: 以下函数只改 pdo_byte0, 不改 m->enabled/state
 *
 *   算法层使用:
 *     startup → pdo_enable + pdo_set_mode → 控制循环(不碰Byte0)
 *     急停    → pdo_estop
 *     恢复    → pdo_recover
 *     清错    → 先读 fb->error_code, 再根据错误类型决定是否 clearcf
 *     切模式  → pdo_set_mode
 *
 *   bit5 清错流程 (由算法层决策):
 *     1. 读 motor_hal_get_feedback → fb.error_code
 *     2. 判断: 温度过高? 等降温后清除; 过流? 失能后清除; 其他? 直接清除
 *     3. 调用 motor_hal_pdo_clear_fault → bit5 脉冲, 下一帧自动清0
 * ===================================================== */

int motor_hal_pdo_enable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 |= PDO_BYTE0_ENABLE;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_disable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 &= ~PDO_BYTE0_ENABLE;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_bus_on(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 |= PDO_BYTE0_BUS_ON;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_bus_off(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 &= ~PDO_BYTE0_BUS_ON;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_clear_fault(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 |= PDO_BYTE0_CLR_ERR;
    m->clr_err_pending = true;  /* 下一帧控制函数自动清除 */
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode)
{
    if (!hal) return -EINVAL;
    if (mode > MOTOR_MODE_MIT) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 = (m->pdo_byte0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(mode);
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_estop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 &= PDO_BYTE0_MODE_MASK;  /* enable=0, bus=0, 保留mode */
    m->clr_err_pending = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_recover(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 = (m->pdo_byte0 & PDO_BYTE0_MODE_MASK)
                 | PDO_BYTE0_ENABLE | PDO_BYTE0_BUS_ON;
    m->clr_err_pending = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_set_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t byte0)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 = byte0;
    m->clr_err_pending = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_get_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t *byte0)
{
    if (!hal || !byte0) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    *byte0 = m->pdo_byte0;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

/* =====================================================
 * 公共 API: 模式 / 参数 / PID
 * ===================================================== */

int motor_hal_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode)
{
    if (!hal || !hal->drv) return -ENODEV;

    /* 巨蟹协议: PDO mode flag 和 SDO 0x6060 是两套编码 */
    static const uint8_t sdo_mode_map[] = {
        [MOTOR_MODE_PROFILE_POS] = 0x01,  /* PP:      CiA 402 标准值  1 */
        [MOTOR_MODE_PROFILE_VEL] = 0x03,  /* PV:      PDF 确认值  3 */
        [MOTOR_MODE_CSP]         = 0x03,  /* CSP:     CiA 402 标准值  3 */
        [MOTOR_MODE_CSV]         = 0x04,  /* CSV:     CiA 402 标准值  4 */
        [MOTOR_MODE_CURRENT]     = 0x0A,  /* 电流环:  CiA 402 标准值 10 */
        [MOTOR_MODE_MIT]         = 0x06,  /* MIT:     厂商自定义        */
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
    /* 0x4F=启动绝对位置运动, 0x0F=停止 — 协议要求 2 字节 SDO 写 */
    uint32_t cw = start ? 0x4FU : 0x0FU;
    return sdo_write_simple(hal->drv, node_id, OD_CONTROLWORD, 0x00, cw, 2);
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
    /* Operation Enabled 必须在 Quick Stop 之前判定:
       0x0027(OP) & 0x004F == 0x0007 会误匹配 Quick Stop 过滤器 */
    if (sw == SW_OP_ENABLED) return MOTOR_STATE_OP_ENABLED;
    if ((sw & 0x004F) == SW_QUICK_STOP_STATE) return MOTOR_STATE_QUICK_STOP;

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
    /* 0x6064 是 int16 (±32767→±180°), SDO 43 响应 4 字节但驱动板不扩展符号位
     * 必须手动取低 16 位做 int16 符号扩展, 否则负数变成大的正数 (差 65536) */
    return (int32_t)(int16_t)(val & 0xFFFF);
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
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->fb_cb  = cb;
    m->fb_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_set_error_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_error_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->err_cb  = cb;
    m->err_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_set_state_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_state_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->state_cb  = cb;
    m->state_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_set_sensor_cb(motor_hal_t *hal, uint8_t node_id,
                             motor_sensor_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->sensor_cb  = cb;
    m->sensor_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

/* =====================================================
 * 公共 API: 专用 SDO 控制接口 — 对应巨蟹协议 4.3 章
 * ===================================================== */

int motor_hal_nmt_send(motor_hal_t *hal, uint8_t node_id, uint8_t cmd)
{
    if (!hal || !hal->drv) return -ENODEV;
    canfd_frame_t f;
    canopen_nmt_build(cmd, node_id, &f);
    return can_driver_send(hal->drv, &f) >= 0 ? 0 : -errno;
}

int motor_hal_get_fault_code(motor_hal_t *hal, uint8_t node_id, uint16_t *code)
{
    if (!hal || !hal->drv || !code) return -EINVAL;
    uint32_t val = 0;
    int ret = sdo_read_simple(hal->drv, node_id, 0x603F, 0x00, &val);
    *code = (uint16_t)(val & 0xFFFF);
    return ret;
}

int motor_hal_get_mos_temp(motor_hal_t *hal, uint8_t node_id, int32_t *temp)
{
    if (!hal || !hal->drv || !temp) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, 0x2662, 0x00, (uint32_t *)temp);
}

int motor_hal_get_motor_temp(motor_hal_t *hal, uint8_t node_id, int32_t *temp)
{
    if (!hal || !hal->drv || !temp) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, 0x2663, 0x00, (uint32_t *)temp);
}

int motor_hal_get_max_current(motor_hal_t *hal, uint8_t node_id, uint32_t *ma)
{
    if (!hal || !hal->drv || !ma) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, 0x2538, 0x00, ma);
}

int motor_hal_set_max_current(motor_hal_t *hal, uint8_t node_id, uint32_t ma)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, 0x2538, 0x00, ma, 4);
}

int motor_hal_set_heartbeat(motor_hal_t *hal, uint8_t node_id, uint32_t ms)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, 0x1017, 0x00, ms, 2);
}

int motor_hal_set_node_id(motor_hal_t *hal, uint8_t node_id, uint8_t new_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (new_id < 1 || new_id > 127) return -EINVAL;
    return sdo_write_simple(hal->drv, node_id, 0x2530, 0x00, new_id, 4);
}

int motor_hal_set_canfd_baud(motor_hal_t *hal, uint8_t node_id, uint8_t baud)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (baud < 1 || baud > 4) return -EINVAL;
    return sdo_write_simple(hal->drv, node_id, 0x2540, 0x00, baud, 4);
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
                /* 自动启动: 只设标志, 由主线程 motor_hal_process_pending_startups() 消费 */
                /* 不能在 recv 线程里调 motor_startup_full — SDO 阻塞会导致自己收不到响应 */
                if (m->config.auto_enable && m->state == MOTOR_STATE_NOT_READY) {
                    m->pending_startup = true;
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
        if (!m) break;

        /* 优先调用户自定义回调 (自定义映射时) */
        if (m->tpdo_raw_cb) {
            m->tpdo_raw_cb(node, f, m->tpdo_raw_ctx);
            break;
        }

        /* 回退: 默认硬编码解析 Statusword+Position+Velocity+Current */
        if (f->dlc < 8) break;
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
 * 公共 API: 处理待启动电机 (主线程调用, 不能从 recv 线程调)
 *
 * 当 recv 线程收到 bootup 帧且 auto_enable=true 时,
 * 只设 pending_startup 标志, 由主线程定期调用此函数执行
 * motor_startup_full (SDO 操作).
 * 这样 SDO 响应由 recv 线程正常接收, 避免死锁.
 * ===================================================== */

int motor_hal_process_pending_startups(motor_hal_t *hal)
{
    if (!hal || !hal->drv) return 0;

    int started = 0;

    for (int i = 0; i < hal->motor_count; i++) {
        motor_node_t *m = &hal->motors[i];

        /* 快速检查: 无需持锁 */
        if (!m->pending_startup) continue;

        pthread_mutex_lock(&hal->lock);
        /* 双重检查 — 可能被其他 startup 命令抢走 */
        if (!m->pending_startup || m->state != MOTOR_STATE_NOT_READY) {
            pthread_mutex_unlock(&hal->lock);
            continue;
        }
        m->pending_startup = false;
        pthread_mutex_unlock(&hal->lock);

        /* 放锁后调 SDO — recv 线程可以正常收帧 */
        fprintf(stderr, "  → processing auto-startup node=%d...\n", m->node_id);
        int ret = motor_startup_full(hal->drv, &m->config, &m->bootup_received);
        if (ret == 0) {
            pthread_mutex_lock(&hal->lock);
            m->enabled = true;
            _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
            pthread_mutex_unlock(&hal->lock);
            fprintf(stderr, "  → node=%d OPERATION_ENABLED (auto)\n", m->node_id);
        } else {
            fprintf(stderr, "  → node=%d auto-startup failed (ret=%d)\n", m->node_id, ret);
        }
        started++;
    }

    return started;
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

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (hal->sync_running) {
        pdo_sync_send(hal->drv);

        /* 绝对时间基准, 无累积漂移 */
        next.tv_nsec += (long)(period_us * 1000UL);
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
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
 * TPDO/RPDO 配置 — 通用映射 (按巨蟹文档时序)
 *
 * 巨蟹文档 PDO 映射流程 (以 TPDO1 为例):
 *   1. 关闭不需要的 PDO 通道 (1801/1802/1803 sub01 bit31=1, best-effort)
 *   2. 设置传输类型 (1800 sub02)
 *   3. 清空映射 (1A00 sub00=0)
 *   4. 写入映射条目 (1A00 sub01/sub02/...)
 *   5. 保存映射数量 (1A00 sub00=N)
 *   6. 启用 PDO (1800 sub01=COB-ID)
 *
 * RPDO 同理, 使用 140x/160x 索引。
 * ===================================================== */

static int _pdo_map_core(motor_hal_t *hal, uint8_t node_id,
                         const pdo_map_entry_cfg_t *entries, uint8_t count,
                         uint16_t comm_idx, uint16_t map_idx,
                         uint32_t cob_id, uint8_t trans_type)
{
    int ret;

    /* 1. 设置传输类型 (trans_type, 1字节) */
    ret = sdo_write_simple(hal->drv, node_id, comm_idx, 0x02,
                           trans_type, 1);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d set trans_type=%d failed\n",
                node_id, trans_type);
        return ret;
    }

    /* 2. 清空映射 (map sub00=0, 1字节) */
    ret = sdo_write_simple(hal->drv, node_id, map_idx, 0x00, 0, 1);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d clear map failed\n", node_id);
        return ret;
    }

    /* 3. 写入映射条目 */
    for (uint8_t i = 0; i < count; i++) {
        /* 映射条目编码: Index[31:16] SubIdx[15:8] BitLen[7:0] */
        uint32_t entry = ((uint32_t)entries[i].index << 16)
                       | ((uint32_t)entries[i].subidx << 8)
                       | (uint32_t)entries[i].bitlen;
        ret = sdo_write_simple(hal->drv, node_id, map_idx,
                               (uint8_t)(i + 1), entry, 4);
        if (ret != 0) {
            fprintf(stderr, "[PDO] node=%d map[%d] 0x%04X.%02X@%db failed\n",
                    node_id, i, entries[i].index,
                    entries[i].subidx, entries[i].bitlen);
            return ret;
        }
    }

    /* 4. 保存映射数量 (1字节) */
    ret = sdo_write_simple(hal->drv, node_id, map_idx, 0x00, count, 1);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d set map count=%d failed\n",
                node_id, count);
        return ret;
    }

    /* 5. 启用 PDO + 设置 COB-ID */
    ret = sdo_write_simple(hal->drv, node_id, comm_idx, 0x01, cob_id, 4);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d set COB=0x%03X failed\n",
                node_id, cob_id);
        return ret;
    }

    return 0;
}

/* ---------- 关闭其他 PDO 通道 (best-effort) ---------- */

static void _pdo_disable_others(motor_hal_t *hal, uint8_t node_id,
                                pdo_type_t type, uint8_t keep_idx)
{
    /* 关闭 TPDO2/3/4 或 RPDO2/3/4 (跳过 keep_idx) */
    const uint16_t comm_base = (type == PDO_TYPE_RPDO)
                               ? OD_RPDO1_COMM : OD_TPDO1_COMM;

    for (uint8_t i = 1; i <= 3; i++) {  /* PDO2/PDO3/PDO4 */
        if (i == keep_idx) continue;
        /* COB-ID bit31=1 → 停用, 其他位保留原始 COB-ID (任意非零值) */
        uint16_t idx = comm_base + (uint16_t)i;
        sdo_write_simple(hal->drv, node_id, idx, 0x01, 0x80000000UL | (uint32_t)(0x80 + (i + 1)), 4);
    }
}

/* ---------- 公共 API ---------- */

int motor_hal_pdo_map(motor_hal_t *hal, uint8_t node_id,
                      const pdo_map_entry_cfg_t *entries, uint8_t count,
                      uint8_t pdo_idx, pdo_type_t type,
                      uint32_t cob_id, uint8_t trans_type)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (!entries || count == 0 || count > 8) return -EINVAL;
    if (pdo_idx > 1) return -EINVAL;

    /* 选择通信参数和映射表索引 */
    uint16_t comm_idx, map_idx;
    if (type == PDO_TYPE_RPDO) {
        comm_idx = (pdo_idx == 0) ? OD_RPDO1_COMM : OD_RPDO2_COMM;
        map_idx  = (pdo_idx == 0) ? OD_RPDO1_MAP  : OD_RPDO2_MAP;
    } else {
        comm_idx = (pdo_idx == 0) ? OD_TPDO1_COMM : OD_TPDO2_COMM;
        map_idx  = (pdo_idx == 0) ? OD_TPDO1_MAP  : OD_TPDO2_MAP;
    }

    const char *dir = (type == PDO_TYPE_RPDO) ? "RPDO" : "TPDO";

    /* 0. 关闭其他 PDO 通道 (best-effort, 失败不阻塞) */
    _pdo_disable_others(hal, node_id, type, pdo_idx);

    /* 1-5. 核心映射时序 */
    int ret = _pdo_map_core(hal, node_id, entries, count,
                            comm_idx, map_idx, cob_id, trans_type);
    if (ret == 0) {
        fprintf(stderr, "[%s] node=%d COB=0x%03X, %d entries, ttype=%d\n",
                dir, node_id, cob_id, count, trans_type);
    }
    return ret;
}

int motor_hal_tpdo_config(motor_hal_t *hal, uint8_t node_id, uint8_t sync_count)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (sync_count == 0 || sync_count > 240) return -EINVAL;

    pdo_map_entry_cfg_t entries[] = {
        {OD_STATUSWORD,      0x00, 16},  /* Statusword */
        {OD_POSITION_ACTUAL, 0x00, 32},  /* Position */
        {OD_VELOCITY_ACTUAL, 0x00, 32},  /* Velocity */
        {OD_CURRENT_ACTUAL,  0x00, 16},  /* Current */
    };

    return motor_hal_pdo_map(hal, node_id, entries, 4, 0,
                             PDO_TYPE_TPDO,
                             PDO_TPDO1_COB(node_id), sync_count);
}

/* =====================================================
 * 标准 RPDO 发送 — 用户自定义映射后发送控制帧
 * ===================================================== */

int motor_hal_rpdo_send(motor_hal_t *hal, uint8_t node_id,
                        const uint8_t *data, uint8_t dlc)
{
    if (!hal || !hal->drv || !data) return -ENODEV;
    if (dlc == 0 || dlc > 8) return -EINVAL;

    canfd_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id     = (uint32_t)(COB_RPDO1_BASE + node_id);  /* 0x200 + node */
    f.dlc    = dlc;
    f.is_fd  = false;   /* 标准 CAN 帧 */
    f.use_brs = false;
    memcpy(f.data, data, dlc);

    return can_driver_send(hal->drv, &f) >= 0 ? 0 : -errno;
}

/* =====================================================
 * 标准 TPDO 原始帧回调
 * ===================================================== */

void motor_hal_set_tpdo_cb(motor_hal_t *hal, uint8_t node_id,
                           motor_tpdo_raw_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->tpdo_raw_cb   = cb;
    m->tpdo_raw_ctx  = ctx;
    pthread_mutex_unlock(&hal->lock);
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
