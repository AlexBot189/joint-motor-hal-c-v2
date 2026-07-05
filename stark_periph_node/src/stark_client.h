/*
 * stark_client.h -- 算法控制接口 (Header-Only)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 依赖: stark_shm.h (共享内存布局)
 * 编译: gcc -O2 your_algo.c -lpthread -lrt -lm
 *
 * 数据方向:
 *   motor_node  -- fb_buffer -->  算法
 *   算法        -- mailbox  -->  motor_node
 *
 * 使用流程:
 *   1. stark_open   - 连接 SHM
 *   2. stark_ready  - 等待校准完成 (阻塞轮询)
 *   3. stark_enable - 使能电机
 *   4. 控制循环     - 读 stark_fb/imu, 写 stark_multi/torque/speed/position
 *   5. stark_estop  - 安全停机
 *   6. stark_close  - 断开
 *
 * 规则:
 *   - 管理命令 (enable/disable/estop/set_mode/clear_fault) 和实时控制命令
 *     不要在同一周期混合发送, 管理命令后至少间隔 5ms 再发控制命令
 *   - 双电机推荐 stark_multi, 一帧 CANFD 同时下发, 同步更好
 *   - 外骨骼场景: 算法不发命令时反馈正常, 关节自由
 *   - 定期调 stark_heartbeat + stark_rt_alive (建议 200ms)
 *   - stark_node 后启动时 stark_open 可重试等待
 *   - root 权限运行 (SHM 由 stark_periph_manager_node 以 root 创建)
 */
#pragma once

#include "stark_shm.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <linux/futex.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制模式常量, 对应 stark_set_mode */
#define STARK_MODE_PP       1   /* 轮廓位置, 驱动板侧梯形加减速 */
#define STARK_MODE_PV       2   /* 轮廓速度, 驱动板侧梯形加减速 */
#define STARK_MODE_CSP      3   /* 循环同步位置, SYNC 触发 */
#define STARK_MODE_CSV      4   /* 循环同步速度, SYNC 触发 */
#define STARK_MODE_CURRENT  5   /* Q轴电流直控 */
#define STARK_MODE_MIT      6   /* MIT 阻抗控制 */

/* 客户端句柄 */
typedef struct {
    int           fd;
    stark_shm_t*  shm;
} stark_client_t;

static inline void stark_close(stark_client_t* c);

/* -- 生命周期 ---------------------------------------------------- */

static inline int stark_open(stark_client_t* c)
{
    if (!c) return -1;
    stark_close(c);
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

/* -- 状态查询 ---------------------------------------------------- */

static inline int stark_ready(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return (c->shm->calib_state == 2);
}

static inline int stark_online(stark_client_t* c, int id)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return 0;
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

/* -- 反馈读取 (零拷贝, 读 SHM 双 Buffer active 端) --------------- */

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

/* -- 控制命令: 实时路径 (环形缓冲 mailbox, 不丢帧) ------------- */

/* SPSC 环形缓冲写入, 返回槽位索引. 缓冲满时短暂自旋等待. */
static inline int _stark_mbox_begin(stark_client_t* c)
{
    if (!c || !c->shm) return -1;
    uint64_t w, r;
    do {
        w = __atomic_load_n(&c->shm->mailbox.seq_write, __ATOMIC_RELAXED);
        r = __atomic_load_n(&c->shm->mailbox.seq_read,  __ATOMIC_ACQUIRE);
        if (w - r >= STARK_MBOX_DEPTH) usleep(50);  /* 缓冲满 */
    } while (w - r >= STARK_MBOX_DEPTH);
    return (int)(w % STARK_MBOX_DEPTH);
}

/* 提交写入: 递增 seq_write, 通知 RT */
static inline void _stark_mbox_commit(stark_client_t* c)
{
    __atomic_add_fetch(&c->shm->mailbox.seq_write, 1, __ATOMIC_RELEASE);
}

/* 力矩控制, 单位 mA */
static inline void stark_torque(stark_client_t* c, int id, int32_t ma)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_TORQUE;
    c->shm->mailbox.frames[slot].cmd[idx].value    = ma;

    _stark_mbox_commit(c);
}

/* 速度控制 (CSV), 单位 RPM */
static inline void stark_speed(stark_client_t* c, int id, float rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_SPEED;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(rpm * 100.0f);

    _stark_mbox_commit(c);
}

/* 轮廓速度 (PV), rpm=目标速度 accel=加速度 RPM/s */
static inline void stark_pv(stark_client_t* c, int id, float rpm, float accel)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_PV;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].value2   = (int32_t)(accel * 100.0f);

    _stark_mbox_commit(c);
}

/* 循环同步速度 (CSV), 单位 RPM */
static inline void stark_csv(stark_client_t* c, int id, float rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_CSV;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(rpm * 100.0f);

    _stark_mbox_commit(c);
}

/* 绝对位置控制 (CSP), 单位 deg, 范围 [-180, 180] */
static inline void stark_position(stark_client_t* c, int id, float deg)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_POS;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(deg * 100.0f);

    _stark_mbox_commit(c);
}

/* 相对位置控制, 自动读当前位置加偏移, 钳位到 [-180, 180] */
static inline void stark_rel_position(stark_client_t* c, int id, float delta_deg)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    motor_data_t fb = stark_fb(c, id);
    float cur_deg  = (float)fb.position * (360.0f / 65536.0f);
    float target   = cur_deg + delta_deg;

    if (target > 180.0f)  target -= 360.0f;
    if (target < -180.0f) target += 360.0f;

    stark_position(c, id, target);
}

/* 轮廓位置 (PP), deg=目标角度 accel=加速度RPM/s vel=轮廓速度RPM */
static inline void stark_pp(stark_client_t* c, int id,
                             float deg, float accel_rpm, float vel_rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd         = STARK_CMD_PP;
    c->shm->mailbox.frames[slot].cmd[idx].value       = (int32_t)(deg * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].value2      = (int32_t)(accel_rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].feedforward = (int32_t)(vel_rpm * 100.0f);

    _stark_mbox_commit(c);
}

/* MIT 阻抗控制, pos_deg=平衡点 vel_rpm=阻尼速度 kp=刚度 kd=阻尼 torque_ma=前馈 */
static inline void stark_mit(stark_client_t* c, int id,
                              float pos_deg, float vel_rpm,
                              float kp, float kd, float torque_ma)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id   = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd        = STARK_CMD_MIT;
    c->shm->mailbox.frames[slot].cmd[idx].mit_pos    = (uint16_t)((pos_deg + 180.0f) * 65535.0f / 360.0f);
    c->shm->mailbox.frames[slot].cmd[idx].mit_vel    = (uint16_t)(vel_rpm);
    c->shm->mailbox.frames[slot].cmd[idx].mit_kp     = (uint16_t)(kp * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].mit_kd     = (uint16_t)(kd * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].mit_torque = (uint16_t)(torque_ma);

    _stark_mbox_commit(c);
}

/*
 * 多轴广播, 一帧 64B CANFD 同时控制双电机.
 *
 * 参数: t1/t2=力矩(mA) v1/v2=速度前馈(RPM) p1/p2=位置(deg)
 * 实际生效的字段取决于当前控制模式:
 *   CURRENT 模式: 只用 t1/t2, v 和 p 填 0
 *   CSP 模式:     p1/p2 输入绝对角度
 *   CSV/PV 模式:  v1/v2 输入速度
 */
static inline void stark_multi(stark_client_t* c,
                                int32_t t1, int32_t v1, int32_t p1,
                                int32_t t2, int32_t v2, int32_t p2)
{
    if (!c || !c->shm) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;

    c->shm->mailbox.frames[slot].cmd[0].motor_id    = 1;
    c->shm->mailbox.frames[slot].cmd[0].cmd         = STARK_CMD_MULTI;
    c->shm->mailbox.frames[slot].cmd[0].value       = t1;
    c->shm->mailbox.frames[slot].cmd[0].value2      = v1;
    c->shm->mailbox.frames[slot].cmd[0].feedforward = p1;

    c->shm->mailbox.frames[slot].cmd[1].motor_id    = 2;
    c->shm->mailbox.frames[slot].cmd[1].cmd         = STARK_CMD_MULTI;
    c->shm->mailbox.frames[slot].cmd[1].value       = t2;
    c->shm->mailbox.frames[slot].cmd[1].value2      = v2;
    c->shm->mailbox.frames[slot].cmd[1].feedforward = p2;

    _stark_mbox_commit(c);
}

/* -- 管理命令 (per-motor mgmt slot, 不和算法 mailbox 竞争) --- */

/*
 * enable/disable/estop/recover/clear_fault 走 mgmt 通道.
 * 每电机独立 slot (mgmt_cmd[id-1] / mgmt_seq[id-1] / mgmt_ack[id-1]),
 * 不受其他电机或 mailbox 控制循环的 seq 覆盖影响.
 */
static inline void _stark_mgmt_cmd(stark_client_t* c, int id, int cmd)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int idx = id - 1;
    c->shm->mgmt_cmd[idx] = (uint8_t)cmd;
    /* 写屏障: cmd 必须在 seq 递增前对其他核心可见 */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_add_fetch(&c->shm->mgmt_seq[idx], 1, __ATOMIC_RELEASE);
}

/* 使能电机 */
static inline void stark_enable(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_ENABLE); }

/*
 * 失能电机: 先 PDO 失能, 再通过多轴广播发 disable+release_brake+电流0,
 * 确保电机停止后刹车松开.
 */
static inline void stark_disable(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_DISABLE); }

/*
 * 急停: PDO 失能 + bus=OFF (刹车抱死), 再发 disable+brake_hold+电流0.
 */
static inline void stark_estop(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_ESTOP); }

/* 从急停恢复 */
static inline void stark_recover(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_RECOVER); }

/*
 * 清除故障: PDO 清故障位 + 自动使能 + 切电流模式 target=0.
 * 清障后电机会恢复使能状态, 可直接进入控制循环.
 */
static inline void stark_clear_fault(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_CLEAR_FAULT); }

/* 切换控制模式, mode 取值见 STARK_MODE_* 常量.
 * set_mode 在控制循环启动前调用, 无 stark_multi 竞争, 走 mailbox 即可. */
static inline void stark_set_mode(stark_client_t* c, int id, int mode)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SET_MODE;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = mode;
    _stark_mbox_commit(c);
}

/* -- 双向心跳 -------------------------------------------------- */

/* 算法声明存活, 建议每 200ms 调一次 */
static inline void stark_heartbeat(stark_client_t* c)
{
    if (!c || !c->shm) return;
    __atomic_add_fetch(&c->shm->algo_heartbeat, 1, __ATOMIC_RELEASE);
}

/* stark_node 反向存活检测, 同频调用. 返回 0 需重连 */
static inline int stark_rt_alive(stark_client_t* c, uint32_t *last_cycle)
{
    if (!c || !c->shm) return 0;
    if (!stark_ready(c)) return 0;
    uint32_t cur = __atomic_load_n(&c->shm->rt_cycle, __ATOMIC_ACQUIRE);
    if (cur == *last_cycle) return 0;
    *last_cycle = cur;
    return 1;
}

/* -- 周期上报 (5ms 自动推送, 校准完成后自动开启) ------------------ */

/* 返回周期上报数据指针, 未开启时返回 NULL */
static inline const PeriodicUploadData* stark_report_data(stark_client_t* c)
{
    if (!c || !c->shm || !c->shm->periodic_enabled) return NULL;
    return &c->shm->periodic_data;
}

/* 上报版本号, 单调递增, 对比上次可检测数据更新 */
static inline uint32_t stark_report_version(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return __atomic_load_n(&c->shm->periodic_version, __ATOMIC_ACQUIRE);
}

/*
 * 尝试读取新数据. 封装版本号比对, 控制循环内一行调用.
 * 用法: stark_report_try_read(&c, &ver, &d), 返回 1 表示新数据, d 指向 SHM 零拷贝
 */
static inline int stark_report_try_read(stark_client_t* c, uint32_t *last_ver,
                                         const PeriodicUploadData** out)
{
    if (!c || !c->shm || !c->shm->periodic_enabled) return 0;
    uint32_t cur = __atomic_load_n(&c->shm->periodic_version, __ATOMIC_ACQUIRE);
    if (cur == *last_ver) return 0;
    *last_ver = cur;
    *out = &c->shm->periodic_data;
    return 1;
}

/*
 * 阻塞等待新的周期上报数据. 无新数据时通过 futex 让出 CPU,
 * 有新数据或超时后返回. 适合算法侧单独开一个数据接收线程被动等待,
 * 主控制循环不必轮询.
 *
 * 参数:
 *   last_ver   算法侧持有的版本号, 首次传入 0, 函数内部更新
 *   out        输出参数, 指向 SHM 内零拷贝数据, 仅返回 1 时有效
 *   timeout_ms 最长等待毫秒, 传 <0 表示无限等待
 * 返回:
 *   1  有新数据, *out 有效, *last_ver 已更新
 *   0  超时, 上报未开启, 或被信号打断, 无新数据
 *
 * 与 stark_report_try_read 共用 periodic_version, 两种取数方式可自由
 * 选择且互不影响. 唤醒由 stark 节点在每次上报后 futex_wake 触发.
 * 内部使用共享 futex (不带 FUTEX_PRIVATE_FLAG), 支持跨进程共享内存.
 */
static inline int stark_report_wait(stark_client_t* c, uint32_t *last_ver,
                                    const PeriodicUploadData** out,
                                    int timeout_ms)
{
    if (!c || !c->shm || !last_ver || !out) return 0;
    if (!c->shm->periodic_enabled) return 0;

    for (;;) {
        uint32_t cur = __atomic_load_n(&c->shm->periodic_version, __ATOMIC_ACQUIRE);
        if (cur != *last_ver) {
            *last_ver = cur;
            *out = &c->shm->periodic_data;
            return 1;
        }

        struct timespec  ts;
        struct timespec *pts = NULL;
        if (timeout_ms >= 0) {
            ts.tv_sec  = timeout_ms / 1000;
            ts.tv_nsec = (long)(timeout_ms % 1000) * 1000000L;
            pts = &ts;
        }

        /* 版本仍为 cur 时睡眠, 等待写端递增并 futex_wake.
         * FUTEX_WAIT 的 timeout 为相对时间. */
        long r = syscall(SYS_futex, &c->shm->periodic_version,
                         FUTEX_WAIT, cur, pts, NULL, 0);
        if (r == 0)                          continue;  /* 被唤醒, 重新比对版本 */
        if (errno == EAGAIN)                 continue;  /* 版本已变, 回头读到新数据 */
        if (errno == EINTR && timeout_ms < 0) continue; /* 无限等待被信号打断, 重进 */
        return 0;                                       /* 超时 / 被打断, 本次无新数据 */
    }
}

#ifdef __cplusplus
}
#endif
