/**
 * @file tool_hal.c
 * @brief motor_hal 薄封装层实现 — 全 SDO, ×100换算, id=0广播, SDO时序
 */

#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

/* ================================================================
 * 全局状态
 * ================================================================ */

motor_hal_t *g_hal = NULL;

static int    g_motor_ids[TOOL_MAX_MOTORS];
static int    g_motor_count = 0;

/* ================================================================
 * 初始化 / 清理
 * ================================================================ */

int tool_init(const char *iface)
{
    g_hal = motor_hal_create();
    if (!g_hal) { fprintf(stderr, "ERROR: motor_hal_create failed\n"); return -1; }
    int ret = motor_hal_init(g_hal, iface, 1000000, 5000000);
    if (ret < 0) {
        fprintf(stderr, "ERROR: CANFD init failed on %s (ret=%d)\n", iface, ret);
        motor_hal_destroy(g_hal); g_hal = NULL; return -1;
    }
    printf("✓ CANFD %s opened (arb=1M, data=5M)\n", iface);
    return 0;
}

void tool_cleanup(void)
{
    if (!g_hal) return;
    motor_hal_destroy(g_hal);
    g_hal = NULL;
    g_motor_count = 0;
}

int tool_register_motor(int node_id)
{
    if (g_motor_count >= TOOL_MAX_MOTORS) return -1;
    for (int i = 0; i < g_motor_count; i++)
        if (g_motor_ids[i] == node_id) return 0;
    g_motor_ids[g_motor_count++] = node_id;
    return 0;
}

int tool_unregister_motor(int node_id)
{
    for (int i = 0; i < g_motor_count; i++) {
        if (g_motor_ids[i] == node_id) {
            memmove(&g_motor_ids[i], &g_motor_ids[i + 1],
                    (g_motor_count - i - 1) * sizeof(int));
            g_motor_count--;
            return 0;
        }
    }
    return -1;
}

int tool_motor_count(void) { return g_motor_count; }
int tool_motor_id(int index) { return (index >= 0 && index < g_motor_count) ? g_motor_ids[index] : -1; }

/* ================================================================
 * 内部: id 解析 (0=广播到已注册列表)
 * ================================================================ */

static int _parse_ids(int id, int *ids, int *count)
{
    if (id == TOOL_BROADCAST_ID) {
        *count = g_motor_count;
        if (*count == 0) { fprintf(stderr, "No motors registered\n"); return -1; }
        for (int i = 0; i < *count; i++) ids[i] = g_motor_ids[i];
    } else {
        ids[0] = id; *count = 1;
    }
    return 0;
}

/* 过滤离线电机: 保留有反馈帧记录的, 剔除从未收到过反馈的 */
int tool_filter_online(int *ids, int n)
{
    int online = 0;
    for (int i = 0; i < n; i++) {
        motor_feedback_t fb;
        if (g_hal && motor_hal_get_feedback(g_hal, (uint8_t)ids[i], &fb) == 0
            && fb.timestamp_us > 0) {
            ids[online++] = ids[i];
        } else {
            fprintf(stderr, "WARN: motor %d offline, skipping\n", ids[i]);
        }
    }
    return online;
}

/* ================================================================
 * 系统命令
 * ================================================================ */

int tool_disable(int id)
{
    int ids[TOOL_MAX_MOTORS], n, errors = 0;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_disable(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d disable failed\n", ids[i]); errors++; }
        else printf("✓ Motor %d disabled\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_fault_reset(int id)
{
    int ids[TOOL_MAX_MOTORS], n, errors = 0;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        printf("Resetting fault motor %d...\n", ids[i]);
        int ret = motor_hal_fault_reset(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d fault_reset failed\n", ids[i]); errors++; }
        else printf("✓ Motor %d fault cleared\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_reboot(int id)
{
    int ids[TOOL_MAX_MOTORS], n, errors = 0;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_nmt_send(g_hal, (uint8_t)ids[i], NMT_CMD_RESET_NODE);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d reboot failed\n", ids[i]); errors++; }
        else printf("✓ Motor %d reboot sent\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * SDO 电流控制 — 完整时序: 使能, 切电流模式, 写目标电流
 *   torque <id> <mA>  范围 0~20000 mA
 * ================================================================ */

int tool_torque_sdo(int id, int ma)
{
    if (ma < -20000 || ma > 20000) {
        fprintf(stderr, "ERROR: current %d mA out of range (-20000~20000)\n", ma);
        return -1;
    }

    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        int ret;

        /* 切电流模式 (使能在 daemon startup 阶段完成, 控制命令只做模式切换) */
        ret = motor_hal_set_mode(g_hal, mid, MOTOR_MODE_CURRENT);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d set_mode(CUR) failed\n", mid); errors++; continue; }

        /* 写目标电流 (2字节, inst=0x2B) */
        ret = motor_hal_sdo_write(g_hal, mid, 0x6071, 0, (uint32_t)ma, 2);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d write current failed\n", mid); errors++; continue; }

        printf("✓ Motor %d: torque=%dmA (current mode)\n", mid, ma);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * SDO 速度控制 — 完整时序: 使能, 切PV模式, 设加减速, 写目标速度
 *   speed <id> <rpm> [acc] [dec]
 *   加减速范围 0~10000 RPM/s, 无上限速度
 * ================================================================ */

int tool_speed_sdo(int id, int rpm, int acc, int dec)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    uint16_t accel = (uint16_t)acc;
    uint16_t decel = (uint16_t)dec;

    if (accel > 10000 || decel > 10000) {
        fprintf(stderr, "ERROR: accel/decel %d/%d out of range (0~10000 RPM/s)\n", accel, decel);
        return -1;
    }

    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        int ret;

        /* 切 PV 模式 (使能在 daemon startup 阶段完成) */
        ret = motor_hal_set_mode(g_hal, mid, MOTOR_MODE_PROFILE_VEL);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d set_mode(PV) failed\n", mid); errors++; continue; }

        /* 加减速 0x6083/0x6084 (电机端 RPM/s) */
        ret = motor_hal_set_accel_decel(g_hal, mid, accel, decel);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d set_accel_decel failed\n", mid); errors++; continue; }

        /* 目标速度 0x60FF (4字节, 电机端 RPM) */
        ret = motor_hal_sdo_write(g_hal, mid, 0x60FF, 0, (uint32_t)rpm, 4);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d write speed target failed\n", mid); errors++; continue; }

        printf("✓ Motor %d: speed=%dRPM accel=%d decel=%d (PV mode)\n", mid, rpm, accel, decel);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * SDO 位置控制 — 完整时序: 使能, 切PP模式, 设加减速, 轨迹速度, 目标, 启动
 *   abs <id> <deg>
 *   加减速默认2000 RPM/s, 轨迹速度默认10 RPM, 目标范围 -180°~180°
 * ================================================================ */

static uint16_t g_abs_accel = 2000;   /* RPM/s, 默认2000, 范围0~10000 */
static uint16_t g_abs_speed = 10;     /* RPM 输出端, 默认10, 范围0~30 */

int tool_abs_set_accel(int id, int acc)
{
    (void)id;
    uint16_t a = (uint16_t)acc;
    if (a > 10000) { fprintf(stderr, "ERROR: accel %d out of range (0~10000 RPM/s)\n", a); return -1; }
    g_abs_accel = a;
    printf("✓ Position mode accel set to %d RPM/s\n", a);
    return 0;
}

int tool_abs_set_speed(int id, int rpm)
{
    (void)id;
    uint16_t s = (uint16_t)rpm;
    if (s > 30) { fprintf(stderr, "ERROR: speed %d out of range (0~30 RPM, output side)\n", s); return -1; }
    g_abs_speed = s;
    printf("✓ Position mode profile velocity set to %d RPM\n", s);
    return 0;
}

int tool_abs_sdo(int id, float deg)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int32_t counts = motor_deg_to_counts(deg);

    if (counts < -32767 || counts > 32768) {
        fprintf(stderr, "ERROR: position %d counts out of range (-32767~32768)\n", counts);
        return -1;
    }

    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        int ret;

        /* 切 PP 模式 (使能在 daemon startup 阶段完成) */
        ret = motor_hal_set_mode(g_hal, mid, MOTOR_MODE_PROFILE_POS);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d set_mode(PP) failed\n", mid); errors++; continue; }

        /* 加减速 0x6083/0x6084 (电机端 RPM/s) */
        motor_hal_set_accel_decel(g_hal, mid, g_abs_accel, g_abs_accel);

        /* 轨迹速度 0x6081 (输出端 RPM, 范围0~30) */
        motor_hal_set_profile_velocity(g_hal, mid, g_abs_speed);

        /* 目标位置 0x607A */
        ret = motor_hal_sdo_write(g_hal, mid, 0x607A, 0, (uint32_t)counts, 4);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d write pos target failed\n", mid); errors++; continue; }

        /* 启动绝对运动 CW=0x4F */
        ret = motor_hal_sdo_write(g_hal, mid, 0x6040, 0, 0x004F, 2);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d start motion failed\n", mid); errors++; continue; }

        printf("✓ Motor %d: abs %.2f° (counts=%d, accel=%d, vel=%d)\n",
               mid, deg, counts, g_abs_accel, g_abs_speed);
    }
    return errors > 0 ? -1 : 0;
}

int tool_abs_stop(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_sdo_write(g_hal, (uint8_t)ids[i], 0x6040, 0, 0x000F, 2);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d stop motion failed\n", ids[i]); errors++; }
        else printf("✓ Motor %d motion stopped\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * 单控 SDO: 零位标定 — 自动失能, 写0x2531
 * ================================================================ */

int tool_set_zero_auto(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        int ret;

        /* 失能 */
        printf("Disabling motor %d for zero calibration...\n", mid);
        ret = motor_hal_disable(g_hal, mid);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d disable failed\n", mid); errors++; continue; }

        /* 写零位 0x2531=1 */
        ret = motor_hal_set_zero(g_hal, mid);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d set_zero failed\n", mid); errors++; continue; }

        printf("✓ Motor %d zero position calibrated (re-enable manually)\n", mid);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * 单控 SDO: 限位 — 自动失能, 写限位, save_flash (用户手动enable)
 * ================================================================ */

int tool_limit_pos_set(int id, float deg)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int32_t counts = (int32_t)(deg * 65536.0f / 360.0f);

    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        int ret;

        /* 失能 */
        printf("Disabling motor %d for limit config...\n", mid);
        ret = motor_hal_disable(g_hal, mid);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d disable failed\n", mid); errors++; continue; }

        /* 写正限位 0x607D sub2 */
        ret = motor_hal_sdo_write(g_hal, mid, 0x607D, 0x02, (uint32_t)counts, 4);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d write pos limit failed\n", mid); errors++; continue; }

        /* 保存 Flash */
        ret = motor_hal_save_flash(g_hal, mid);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d save flash failed\n", mid); errors++; continue; }

        printf("✓ Motor %d: pos limit=%.2f° (counts=%d) saved to Flash (re-enable manually)\n",
               mid, deg, counts);
    }
    return errors > 0 ? -1 : 0;
}

int tool_limit_neg_set(int id, float deg)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int32_t counts = (int32_t)(deg * 65536.0f / 360.0f);

    int errors = 0;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        int ret;

        printf("Disabling motor %d for limit config...\n", mid);
        ret = motor_hal_disable(g_hal, mid);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d disable failed\n", mid); errors++; continue; }

        /* 写负限位 0x607D sub1 */
        ret = motor_hal_sdo_write(g_hal, mid, 0x607D, 0x01, (uint32_t)counts, 4);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d write neg limit failed\n", mid); errors++; continue; }

        ret = motor_hal_save_flash(g_hal, mid);
        if (ret != 0) { fprintf(stderr, "✗ Motor %d save flash failed\n", mid); errors++; continue; }

        printf("✓ Motor %d: neg limit=%.2f° (counts=%d) saved to Flash (re-enable manually)\n",
               mid, deg, counts);
    }
    return errors > 0 ? -1 : 0;
}

int tool_limit_pos_read(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    for (int i = 0; i < n; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x607D, 0x02, &val);
        if (ret == 0) {
            float deg = (float)(int32_t)val * 360.0f / 65536.0f;
            printf("[%d] pos_limit = %d counts = %.2f°\n", ids[i], (int32_t)val, deg);
        } else {
            fprintf(stderr, "[%d] limit_pos read failed\n", ids[i]);
        }
    }
    return 0;
}

int tool_limit_neg_read(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    for (int i = 0; i < n; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x607D, 0x01, &val);
        if (ret == 0) {
            float deg = (float)(int32_t)val * 360.0f / 65536.0f;
            printf("[%d] neg_limit = %d counts = %.2f°\n", ids[i], (int32_t)val, deg);
        } else {
            fprintf(stderr, "[%d] limit_neg read failed\n", ids[i]);
        }
    }
    return 0;
}

int tool_save_flash(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_save_flash(g_hal, (uint8_t)ids[i]);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d save failed\n", ids[i]); errors++; }
        else printf("✓ Motor %d parameters saved to Flash\n", ids[i]);
    }
    return errors > 0 ? -1 : 0;
}

int tool_set_pid(int id, uint16_t cp, uint16_t ci,
                 uint16_t vp, uint16_t vi, uint16_t pp, uint16_t pi)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    motor_pid_t pid = { cp, ci, vp, vi, pp, pi };
    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_set_pid(g_hal, (uint8_t)ids[i], &pid);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d set_pid failed\n", ids[i]); errors++; }
        else printf("✓ Motor %d PID: CP=%d CI=%d VP=%d VI=%d PP=%d PI=%d\n",
                    ids[i], cp, ci, vp, vi, pp, pi);
    }
    return errors > 0 ? -1 : 0;
}

int tool_read_pid(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    for (int i = 0; i < n; i++) {
        motor_pid_t pid;
        int ret = motor_hal_read_pid(g_hal, (uint8_t)ids[i], &pid);
        if (ret == 0) {
            printf("[%d] PID: CP=%d CI=%d | VP=%d VI=%d | PP=%d PI=%d\n",
                   ids[i], pid.current_p, pid.current_i,
                   pid.velocity_p, pid.velocity_i,
                   pid.position_p, pid.position_i);
        } else {
            fprintf(stderr, "[%d] pid read failed\n", ids[i]);
        }
    }
    return 0;
}

/* ================================================================
 * 通用 SDO 读写
 * ================================================================ */

int tool_sdo_read(int id, uint16_t index, uint8_t subidx)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    for (int i = 0; i < n; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], index, subidx, &val);
        if (ret == 0)
            printf("[%d] SDO 0x%04X:%02X = 0x%08X (%d)\n", ids[i], index, subidx, val, val);
        else
            fprintf(stderr, "[%d] SDO read 0x%04X:%02X failed (ret=%d)\n", ids[i], index, subidx, ret);
    }
    return 0;
}

int tool_sdo_write(int id, uint16_t index, uint8_t subidx, uint32_t value, uint8_t size)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;

    int errors = 0;
    for (int i = 0; i < n; i++) {
        int ret = motor_hal_sdo_write(g_hal, (uint8_t)ids[i], index, subidx, value, size);
        if (ret < 0) { fprintf(stderr, "✗ Motor %d write 0x%04X:%02X failed\n", ids[i], index, subidx); errors++; }
        else printf("✓ Motor %d: write 0x%04X:%02X = 0x%08X\n", ids[i], index, subidx, value);
    }
    return errors > 0 ? -1 : 0;
}

/* ================================================================
 * 读取命令 — 格式化输出
 * ================================================================ */

int tool_read_angle(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        int32_t pos = motor_hal_get_position(g_hal, (uint8_t)ids[i]);
        float deg = motor_counts_to_deg((int16_t)pos);
        printf("[%d] angle = %.2f° (raw: %d counts)\n", ids[i], deg, pos);
    }
    return 0;
}

int tool_read_speed(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        int32_t spd = motor_hal_get_velocity(g_hal, (uint8_t)ids[i]);
        printf("[%d] speed = %d (raw value)\n", ids[i], spd);
    }
    return 0;
}

int tool_read_current(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        int32_t cur = motor_hal_get_current(g_hal, (uint8_t)ids[i]);
        printf("[%d] current = %d mA\n", ids[i], cur);
    }
    return 0;
}

int tool_read_temp(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        int32_t t = 0;
        motor_hal_get_motor_temp(g_hal, (uint8_t)ids[i], &t);
        printf("[%d] motor_temp = %d (%.1f°C)\n", ids[i], t, motor_temp_to_c((int16_t)t));
    }
    return 0;
}

int tool_read_state(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        uint16_t sw = motor_hal_get_statusword(g_hal, (uint8_t)ids[i]);
        motor_state_t st = motor_hal_get_state(g_hal, (uint8_t)ids[i]);
        printf("[%d] state = %s (SW=0x%04X)\n", ids[i], motor_state_str(st), sw);
    }
    return 0;
}

int tool_read_error(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        uint16_t err = 0;
        int ret = motor_hal_get_fault_code(g_hal, (uint8_t)ids[i], &err);
        if (ret == 0) {
            const char *name = "Unknown";
            switch (err) {
                case 0x0000: name = "None"; break;
                case 0x0001: name = "OverVoltage"; break;
                case 0x0002: name = "UnderVoltage"; break;
                case 0x0004: name = "OverTemp"; break;
                case 0x0008: name = "Stall"; break;
                case 0x0010: name = "Overload"; break;
                case 0x0020: name = "CurrentSampleErr"; break;
                case 0x0040: name = "PosLimit"; break;
                case 0x0080: name = "NegLimit"; break;
                case 0x0100: name = "EncoderTimeout"; break;
                case 0x0200: name = "OverMaxSpeed"; break;
                case 0x0400: name = "ElecAngleInitFail"; break;
                case 0x1000: name = "PositionErrLarge"; break;
                case 0x2000: name = "EncoderFault"; break;
            }
            printf("[%d] error = 0x%04X (%s)\n", ids[i], err, name);
        } else {
            fprintf(stderr, "[%d] error read failed\n", ids[i]);
        }
    }
    return 0;
}

int tool_read_version(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        uint32_t ver = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x100A, 0x00, &ver);
        if (ret == 0)
            printf("[%d] firmware = 0x%08X\n", ids[i], ver);
        else
            fprintf(stderr, "[%d] version read failed\n", ids[i]);
    }
    return 0;
}

int tool_read_mode(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        uint32_t mode = 0;
        int ret = motor_hal_sdo_read_u32(g_hal, (uint8_t)ids[i], 0x6061, 0x00, &mode);
        if (ret == 0) {
            const char *name = "?";
            switch ((uint8_t)mode) {
                case 0x01: name = "PP"; break;
                case 0x03: name = "PV"; break;
                case 0x08: name = "CSP"; break;
                case 0x09: name = "CSV"; break;
                case 0x06: name = "MIT";     break;
                case 0x0A: name = "Current"; break;
            }
            printf("[%d] mode = 0x%02X (%s)\n", ids[i], (uint8_t)mode, name);
        }
    }
    return 0;
}

int tool_read_all(int id)
{
    int ids[TOOL_MAX_MOTORS], n;
    if (_parse_ids(id, ids, &n) < 0) return -1;
    n = tool_filter_online(ids, n);
    if (n == 0) return -1;
    for (int i = 0; i < n; i++) {
        uint8_t mid = (uint8_t)ids[i];
        printf("── Motor %d ──\n", mid);
        tool_read_angle(mid);
        tool_read_speed(mid);
        tool_read_current(mid);
        tool_read_temp(mid);
        tool_read_state(mid);
        tool_read_error(mid);
        tool_read_version(mid);
    }
    return 0;
}

/* ================================================================
 * watch 模式
 * ================================================================ */

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

    volatile int running = 1;
    uint64_t t0 = _now_us();
    uint64_t last_print = 0;

    char buf[1024];
    int n = snprintf(buf, sizeof(buf), "motor_tool watch period=%dms motors=", period_ms);
    for (int i = 0; i < g_motor_count; i++)
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%d%s", g_motor_ids[i],
                      (i < g_motor_count - 1) ? "," : "");
    n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\n");
    write(out_fd >= 0 ? out_fd : STDOUT_FILENO, buf, (size_t)n);

    while (running) {
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
 * 传感器看板 (后台线程)
 * ================================================================ */

static volatile int g_sensor_watch_running = 0;
static pthread_t    g_sensor_watch_thread;
static int          g_sensor_watch_id   = 0;
static int          g_sensor_watch_fd   = -1;

static void* _sensor_watch_thread_fn(void *arg)
{
    (void)arg;
    char buf[256];
    uint64_t last_ts = 0;
    uint64_t first_ts = 0;

    while (g_sensor_watch_running) {
        motor_sensor_t s;
        int ret = motor_hal_get_sensor(g_hal, (uint8_t)g_sensor_watch_id, &s);
        if (ret == 0 && s.timestamp_us != last_ts) {
            int64_t delta_us = (last_ts > 0) ? (int64_t)(s.timestamp_us - last_ts) : 0;
            if (first_ts == 0) first_ts = s.timestamp_us;
            last_ts = s.timestamp_us;

            double elapsed_s = (double)(s.timestamp_us - first_ts) / 1000000.0;
            (void)snprintf(buf, sizeof(buf),
                    "[%8.3fs] #%llu | Hall: %4d %4d %4d | Force: %5d%s | Knee: %4d | SW: %d | Δ=%lldus",
                    elapsed_s,
                    (unsigned long long)s.timestamp_us,
                    s.hall_adc0, s.hall_adc1, s.hall_adc2,
                    s.force_raw, s.data_valid ? "" : " (inv)",
                    s.knee_adc, s.hw_sw_pc9,
                    (long long)delta_us);
            if (g_sensor_watch_fd >= 0) {
                char json[512];
                int jn = snprintf(json, sizeof(json),
                        "{\"type\":\"watch\",\"data\":\"%s\"}\n", buf);
                ssize_t wr = write(g_sensor_watch_fd, json, (size_t)jn);
                if (wr < 0) { g_sensor_watch_running = 0; break; }
            }
        }
        usleep(500);
    }

    if (g_sensor_watch_fd >= 0) {
        close(g_sensor_watch_fd);
        g_sensor_watch_fd = -1;
    }
    return NULL;
}

int tool_sensor_watch_start(int id, int out_fd)
{
    if (g_sensor_watch_running) return -1;
    g_sensor_watch_id = id;
    g_sensor_watch_fd = out_fd;  /* 线程接管 fd 所有权, 退出时关闭 */
    g_sensor_watch_running = 1;
    if (pthread_create(&g_sensor_watch_thread, NULL, _sensor_watch_thread_fn, NULL) != 0) {
        g_sensor_watch_running = 0;
        close(g_sensor_watch_fd);
        g_sensor_watch_fd = -1;
        return -1;
    }
    return 0;
}

int tool_sensor_watch_stop(void)
{
    if (!g_sensor_watch_running) return 0;
    g_sensor_watch_running = 0;
    pthread_join(g_sensor_watch_thread, NULL);
    return 0;
}

/* ================================================================
 * PDO 映射 — 调用 motor_hal_pdo_map 通用接口
 * ================================================================ */

int tool_pdo_map(uint8_t id, pdo_type_t type,
                 const pdo_map_entry_cfg_t *entries, uint8_t count,
                 uint32_t cob_id, uint8_t trans_type)
{
    if (!g_hal) return -1;
    return motor_hal_pdo_map(g_hal, id, entries, count, 0,
                             type, cob_id, trans_type);
}

int tool_rpdo_send(uint8_t id, const uint8_t *data, uint8_t dlc)
{
    if (!g_hal) return -1;
    return motor_hal_rpdo_send(g_hal, id, data, dlc);
}
