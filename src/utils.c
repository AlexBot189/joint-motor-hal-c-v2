/**
 * @file utils.c
 * @brief 工具函数
 */

#include "motor_hal_types.h"
#include <sys/time.h>
#include <time.h>
#include <math.h>

/* ---------- 时间 ---------- */

uint64_t motor_utils_now_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000UL + (uint64_t)tv.tv_usec;
}

void motor_utils_sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* ---------- 编码器换算 ---------- */

float motor_utils_counts_to_deg(int16_t counts)
{
    return (float)counts * 360.0f / (float)ENCODER_LOAD_RES;
}

int16_t motor_utils_deg_to_counts(float degrees)
{
    /* 钳位 */
    if (degrees < LOAD_ANGLE_MIN) degrees = LOAD_ANGLE_MIN;
    if (degrees > LOAD_ANGLE_MAX) degrees = LOAD_ANGLE_MAX;
    return (int16_t)(degrees * ENCODER_LOAD_RES / 360.0f);
}

/* ---------- 错误码字符串 ---------- */

const char* motor_utils_error_str(uint16_t code)
{
    switch (code) {
        case 0x0000: return "OK";
        case ERR_OVER_VOLTAGE:    return "OVER_VOLTAGE";
        case ERR_UNDER_VOLTAGE:   return "UNDER_VOLTAGE";
        case ERR_OVER_TEMP:       return "OVER_TEMP";
        case ERR_STALL:           return "STALL";
        case ERR_OVERLOAD:        return "OVERLOAD";
        case ERR_CURRENT_SAMPLE:  return "CURRENT_SAMPLE_ERR";
        case ERR_POS_LIMIT:       return "POS_LIMIT";
        case ERR_NEG_LIMIT:       return "NEG_LIMIT";
        case ERR_ENCODER_TIMEOUT: return "ENCODER_TIMEOUT";
        case ERR_OVER_MAX_SPEED:  return "OVER_MAX_SPEED";
        case ERR_ELEC_ANGLE_FAIL: return "ELEC_ANGLE_INIT_FAIL";
        case ERR_POS_ERROR_LARGE: return "POS_ERROR_LARGE";
        case ERR_ENCODER_FAULT:   return "ENCODER_FAULT";
        default: return "UNKNOWN";
    }
}
