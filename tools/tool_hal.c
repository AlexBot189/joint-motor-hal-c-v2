/**
 * @file tool_hal.c
 * @brief motor_hal 薄封装层实现
 *
 * 核心职责:
 *   1. ×100 换算: 命令行输入整数值 = 实际物理值 × 100
 *      例: 用户输入 2300 → 实际 23.00 度/RPM → tool_hal 内部 ÷100.0f 后调用 motor_hal API
 *   2. id=0 广播: 对已注册的所有电机依次执行操作
 *      例: tool_set_speed(0, 5000) → 对 ID=1,2 都设 50.00 RPM
 *   3. 格式化输出: read 命令使用统一 "[ID] key=val" 格式
 */

#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ================================================================
 * 全局状态
 * ================================================================ */

motor_hal_t *g_hal = NULL;

/* 当前已注册的电机 ID 列表 */
static int    g_motor_ids[TOOL_MAX_MOTORS];
static int    g_motor_count = 0;
static bool   g_initialized = false;

/* ================================================================
 * 初始化 / 清理
 * ================================================================ */

int tool_init(const char *iface)
{
    g_hal = motor_hal_create();
    if (!g_hal) {
        fprintf(stderr, "ERROR: motor_hal_create failed\n");
        return -1;
    }

    int ret = motor_hal_init(g_hal, iface, 1000000, 5000000);
    if (ret < 0) {
        fprintf(stderr, "ERROR: CANFD init failed on %s (ret=%d)\n", iface, ret);
        motor_hal_destroy(g_hal);
        g_hal = NULL;
        return -1;
    }

    printf("✓ CANFD interface %s opened (arb=1M, data=5M)\n", iface);
    g_initialized = true;
    return 0;
}

void tool_cleanup(void)
{
    if (!g_hal) return;
    motor_hal_destroy(g_hal);
    g_hal = NULL;
    g_initialized = false;
    g_motor_count = 0;
}

/* ================================================================
 * 内部: 注册电机 (startup 时调用)
 * ================================================================ */

int tool_register_motor(int node_id)
{
    if (g_motor_count >= TOOL_MAX_MOTORS) return -1;
    /* 避免重复注册 */
    for (int i = 0; i < g_motor_count; i++) {
        if (g_motor_ids[i] == node_id) return 0;
    }
    g_motor_ids[g_motor_count++] = node_id;
    return 0;
}

int tool_motor_count(void) { return g_motor_count; }

int tool_motor_id(int index) {
    if (index < 0 || index >= g_motor_count) return -1;
    return g_motor_ids[index];
}

/* ================================================================
 * 内部: ID 解析 (0 → 广播返回所有ID列表)
 * ================================================================ */

static int _parse_ids(int id, int *ids, int *count)
{
    if (id == TOOL_BROADCAST_ID) {
        if (g_motor_count == 0) return -1;
        memcpy(ids, g_motor_ids, g_motor_count * sizeof(int));
        *count = g_motor_count;
    } else {
        ids[0] = id;
        *count = 1;
    }
    return 0;
}

/* ================================================================
 * 内部: ×100 → float
 * ================================================================ */

static inline float _x100_to_float(int val_x100) {
    return (float)val_x100 / 100.0f;
}

static inline uint16_t _x100_to_u16(int val_x100) {
    return (uint16_t)(val_x100 / 100);
}

/* ================================================================
 * 控制封装
 * ================================================================ */

int tool_set_speed(int id, int rpm_x100)
{
    float rpm = _x100_to_float(rpm_x100);
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        /* 先切到速度模式 */
        motor_hal_set_mode(g_hal, (uint8_t)ids[i], MOTOR_MODE_PROFILE_VEL);
        int ret = motor_hal_set_velocity(g_hal, (uint8_t)ids[i], rpm);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d speed set failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_accel(int id, int acc_x100)
{
    uint16_t acc = _x100_to_u16(acc_x100);
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_accel_decel(g_hal, (uint8_t)ids[i], acc, acc);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d accel set failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_abs_pos(int id, int deg_x100)
{
    float deg = _x100_to_float(deg_x100);
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_position(g_hal, (uint8_t)ids[i], deg);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d abs_pos failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_rel_pos(int id, int delta_x100)
{
    float delta = _x100_to_float(delta_x100);
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        /* 1. 读当前位置 */
        int32_t curr_counts = motor_hal_get_position(g_hal, (uint8_t)ids[i]);
        float curr_deg = motor_counts_to_deg((int16_t)curr_counts);

        /* 2. 计算目标 */
        float target_deg = curr_deg + delta;

        /* 3. 设绝对位置 */
        int ret = motor_hal_set_position(g_hal, (uint8_t)ids[i], target_deg);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d rel_pos failed (curr=%.2f, delta=%.2f)\n",
                               ids[i], curr_deg, delta); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_max_vel(int id, int rpm_x100)
{
    uint16_t rpm_out = _x100_to_u16(rpm_x100);
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_profile_velocity(g_hal, (uint8_t)ids[i], rpm_out);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d max_vel set failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_stop(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_stop(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d stop failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_mode(int id, const char *mode_str)
{
    motor_mode_t mode;
    if (strcmp(mode_str, "pp") == 0)       mode = MOTOR_MODE_PROFILE_POS;
    else if (strcmp(mode_str, "pv") == 0)  mode = MOTOR_MODE_PROFILE_VEL;
    else if (strcmp(mode_str, "csp") == 0) mode = MOTOR_MODE_CSP;
    else if (strcmp(mode_str, "csv") == 0) mode = MOTOR_MODE_CSV;
    else if (strcmp(mode_str, "cur") == 0) mode = MOTOR_MODE_CURRENT;
    else {
        fprintf(stderr, "Unknown mode: %s (pp|pv|csp|csv|cur)\n", mode_str);
        return -1;
    }

    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_mode(g_hal, (uint8_t)ids[i], mode);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d set_mode failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * 新增: 力矩 / CSP / MIT / 抱闸 / 急停
 * ================================================================ */

int tool_set_torque(int id, int ma)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_torque(g_hal, (uint8_t)ids[i], (int16_t)ma);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d torque failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_csp(int id, int deg_x100)
{
    float deg = _x100_to_float(deg_x100);
    int16_t counts = motor_deg_to_counts(deg);
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_ctrl_raw(g_hal, (uint8_t)ids[i], MOTOR_MODE_CSP, counts, 0, 0);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d csp failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_mit(int id, float pos, float vel, float kp, float kd, float torque)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_mit_control(g_hal, (uint8_t)ids[i], pos, vel, kp, kd, torque);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d mit failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_brake(int id, bool release)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        /* 抱闸通过 PDO data[0] bit6 控制 */
        int ret = motor_hal_set_brake(g_hal, (uint8_t)ids[i], release);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d brake failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_quickstop(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_quick_stop(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d quickstop failed\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * 新增: 配置命令 (save / setzero / pid)
 * ================================================================ */

int tool_save_flash(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        printf("Saving motor %d to flash...\n", ids[i]);
        int ret = motor_hal_save_flash(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d save failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d saved\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_zero(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        printf("Zeroing motor %d...\n", ids[i]);
        int ret = motor_hal_set_zero(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d setzero failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d zeroed\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_pid(int id, uint16_t cp, uint16_t ci,
                 uint16_t vp, uint16_t vi, uint16_t pp, uint16_t pi)
{
    motor_pid_t pid = { .current_p=cp, .current_i=ci,
                        .velocity_p=vp, .velocity_i=vi,
                        .position_p=pp, .position_i=pi };
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_pid(g_hal, (uint8_t)ids[i], &pid);
        if (ret < 0) { fprintf(stderr, "WARN: motor %d pid failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d PID set\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * 新增: 通用 SDO 读写 (调试用)
 * ================================================================ */

int tool_sdo_read(int id, uint16_t index, uint8_t subidx)
{
    uint32_t val = 0;
    int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)id, index, subidx, &val);
    if (ret == 0) {
        printf("[%d] SDO 0x%04X:%02X = 0x%08X (%u)\n", id, index, subidx, val, val);
    } else {
        fprintf(stderr, "[%d] SDO 0x%04X:%02X read failed (ret=%d)\n", id, index, subidx, ret);
    }
    return ret;
}

int tool_sdo_write(int id, uint16_t index, uint8_t subidx, uint32_t value, uint8_t size)
{
    int ret = motor_hal_sdo_write(g_hal, (uint8_t)id, index, subidx, value, size);
    if (ret == 0) {
        printf("[%d] SDO 0x%04X:%02X ← 0x%08X (%u) size=%d OK\n",
               id, index, subidx, value, value, size);
    } else {
        fprintf(stderr, "[%d] SDO 0x%04X:%02X write failed (ret=%d)\n",
                id, index, subidx, ret);
    }
    return ret;
}

/* ================================================================
 * 读取封装 — 输出原始值, 不做换算
 * ================================================================ */

static int _read_one_angle(uint8_t id)
{
    int32_t val = motor_hal_get_position(g_hal, id);
    printf("[%d] angle=%d\n", id, val);
    return val >= 0 ? 0 : -1;
}

static int _read_one_speed(uint8_t id)
{
    int32_t val = motor_hal_get_velocity(g_hal, id);
    printf("[%d] speed=%d\n", id, val);
    return 0;
}

static int _read_one_current(uint8_t id)
{
    int32_t val = motor_hal_get_current(g_hal, id);
    printf("[%d] current=%d\n", id, val);
    return 0;
}

static int _read_one_temp(uint8_t id)
{
    motor_feedback_t fb;
    if (motor_hal_get_feedback(g_hal, id, &fb) == 0) {
        printf("[%d] temp=%d\n", id, fb.temperature);
    } else {
        printf("[%d] temp=N/A\n", id);
    }
    return 0;
}

static int _read_one_state(uint8_t id)
{
    motor_state_t st = motor_hal_get_state(g_hal, id);
    printf("[%d] state=%s\n", id, motor_state_str(st));
    return (st == MOTOR_STATE_UNKNOWN) ? -1 : 0;
}

static int _read_one_error(uint8_t id)
{
    motor_feedback_t fb;
    if (motor_hal_get_feedback(g_hal, id, &fb) == 0) {
        printf("[%d] error=0x%04X\n", id, fb.error_code);
    } else {
        printf("[%d] error=N/A\n", id);
    }
    return 0;
}

static int _read_one_version(uint8_t id)
{
    uint32_t ver = 0;
    int ret = motor_hal_sdo_read_u32(g_hal, id, OD_FIRMWARE_VER, 0x00, &ver);
    if (ret == 0) {
        printf("[%d] firmware=0x%08X\n", id, ver);
    } else {
        printf("[%d] firmware=N/A\n", id);
    }
    return ret;
}

static int _read_one_all(uint8_t id)
{
    motor_feedback_t fb;
    int fb_ok = (motor_hal_get_feedback(g_hal, id, &fb) == 0);
    motor_state_t st = motor_hal_get_state(g_hal, id);

    printf("[%d] angle=%d speed=%d cur=%dmA temp=%d state=%s err=0x%04X",
           id,
           fb_ok ? fb.position : -1,
           fb_ok ? fb.velocity : -1,
           fb_ok ? fb.current_iq : -1,
           fb_ok ? fb.temperature : -1,
           motor_state_str(st),
           fb_ok ? fb.error_code : 0xFFFF);

    if (st == MOTOR_STATE_OP_ENABLED && fb_ok) {
        printf(" %s%s%s%s",
               (fb.status_byte & 0x80) ? "ENABLED " : "",
               (fb.status_byte & 0x40) ? "BRAKE_RELEASED " : "",
               (fb.status_byte & 0x20) ? "ERROR " : "",
               (fb.status_byte & 0x10) ? "TARGET_REACHED" : "");
    }
    printf("\n");
    return 0;
}

/* ================================================================
 * 读取命令: 根据 item 分发
 * ================================================================ */

typedef int (*read_one_fn)(uint8_t id);

static int _read_broadcast(int id, read_one_fn fn)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        if (fn((uint8_t)ids[i]) < 0) errors++;
    }
    return errors > 0 ? -1 : 0;
}

int tool_read_angle(int id)   { return _read_broadcast(id, _read_one_angle); }
int tool_read_speed(int id)   { return _read_broadcast(id, _read_one_speed); }
int tool_read_current(int id) { return _read_broadcast(id, _read_one_current); }
int tool_read_temp(int id)    { return _read_broadcast(id, _read_one_temp); }
int tool_read_state(int id)   { return _read_broadcast(id, _read_one_state); }
int tool_read_error(int id)   { return _read_broadcast(id, _read_one_error); }
int tool_read_version(int id) { return _read_broadcast(id, _read_one_version); }
int tool_read_all(int id)     { return _read_broadcast(id, _read_one_all); }

/* ================================================================
 * 轮询封装
 * ================================================================ */

void tool_poll(int timeout_ms)
{
    motor_hal_poll(g_hal, timeout_ms);
}

/* ================================================================
 * watch 模式
 * ================================================================ */

#include <signal.h>
#include <time.h>

static volatile int g_watch_running = 0;

static uint64_t _now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

int tool_watch_start(int period_ms, int out_fd)
{
    if (!g_hal || g_motor_count == 0) {
        dprintf(out_fd >= 0 ? out_fd : STDERR_FILENO,
                "ERROR: no motors registered\n");
        return -1;
    }

    g_watch_running = 1;
    uint64_t t0 = _now_us();
    uint64_t last_print = 0;

    char buf[1024];
    int n = snprintf(buf, sizeof(buf), "motor_tool watch period=%dms motors=", period_ms);
    for (int i = 0; i < g_motor_count; i++)
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%d%s", g_motor_ids[i],
                      (i < g_motor_count - 1) ? "," : "");
    n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\n");
    write(out_fd >= 0 ? out_fd : STDOUT_FILENO, buf, (size_t)n);

    while (g_watch_running) {
        uint64_t now = _now_us();
        if (now - last_print >= (uint64_t)period_ms * 1000UL) {
            last_print = now;
            double elapsed = (double)(now - t0) / 1000000.0;

            n = snprintf(buf, sizeof(buf), "[%9.3f]", elapsed);
            for (int i = 0; i < g_motor_count; i++) {
                motor_feedback_t fb;
                if (motor_hal_get_feedback(g_hal, (uint8_t)g_motor_ids[i], &fb) == 0) {
                    n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                                  " [%d] pos=%6d vel=%4d cur=%4dmA temp=%3d err=0x%04X%s",
                                  g_motor_ids[i],
                                  fb.position, fb.velocity, fb.current_iq,
                                  fb.temperature, fb.error_code,
                                  (i < g_motor_count - 1) ? " |" : "");
                } else {
                    n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                                  " [%d] -%s", g_motor_ids[i],
                                  (i < g_motor_count - 1) ? " |" : "");
                }
            }
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\n");

            ssize_t wr = write(out_fd >= 0 ? out_fd : STDOUT_FILENO, buf, (size_t)n);
            if (wr < 0) break;
        }
    }

    return 0;
}

/* ================================================================
 * V2: ODS 协议扩展 — fault_reset / disable / reboot
 * ================================================================ */

int tool_fault_reset(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        printf("Resetting fault motor %d...\n", ids[i]);
        int ret = motor_hal_fault_reset(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "  ✗ motor %d fault_reset failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d fault cleared\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_disable(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        printf("Disabling motor %d...\n", ids[i]);
        int ret = motor_hal_disable(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "  ✗ motor %d disable failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d disabled\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_reboot(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        printf("Rebooting motor %d...\n", ids[i]);
        motor_hal_sdo_write(g_hal, (uint8_t)ids[i], 0x6040, 0, 0x06, 3);
        int ret = motor_hal_sdo_write(g_hal, (uint8_t)ids[i], 0x6040, 0, 0x81, 3);
        if (ret < 0) { fprintf(stderr, "  ✗ motor %d reboot failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d reboot sent\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * V2: posctrl / postarget / speedtarget
 * ================================================================ */

int tool_set_pos_ctrl(int id, bool start)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_pos_ctrl(g_hal, (uint8_t)ids[i], start);
        if (ret < 0) { fprintf(stderr, "  ✗ motor %d posctrl %s failed\n",
                               ids[i], start?"start":"stop"); errors++; }
        else         printf("  ✓ motor %d position %s\n", ids[i], start?"started":"stopped");
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_pos_target(int id, int counts)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_pos_target(g_hal, (uint8_t)ids[i], (int32_t)counts);
        if (ret < 0) { fprintf(stderr, "  ✗ motor %d postarget failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d target_pos=%d\n", ids[i], counts);
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_speed_target(int id, int rpm)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_speed_target(g_hal, (uint8_t)ids[i], (int32_t)rpm);
        if (ret < 0) { fprintf(stderr, "  ✗ motor %d speedtarget failed\n", ids[i]); errors++; }
        else         printf("  ✓ motor %d target_speed=%d RPM\n", ids[i], rpm);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * V2: 读取扩展 — voltage / bus_current / mode
 * ================================================================ */

int tool_read_voltage(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x2600, 0, &val);
        if (ret == 0) printf("[%d] voltage=%u\n", ids[i], val);
        else { printf("[%d] voltage=N/A\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_read_bus_current(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x6077, 0, &val);
        if (ret == 0) printf("[%d] bus_current=%d mA\n", ids[i], (int16_t)val);
        else { printf("[%d] bus_current=N/A\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}

int tool_read_mode(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x6061, 0, &val);
        if (ret == 0) {
            const char *names[] = {"?","PP","PV","CSP","CSV","CUR","MIT"};
            printf("[%d] mode=%s\n", ids[i], (val<7)?names[val]:"?");
        } else { printf("[%d] mode=N/A\n", ids[i]); errors++; }
    }
    return errors > 0 ? -1 : 0;
}
