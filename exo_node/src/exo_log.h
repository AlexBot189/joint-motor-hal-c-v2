/*
 * @file exo_log.h
 * @brief 日志封装 — 透传 ECO_* 宏
 *
 * 非 RT 路径日志接口，直接映射到 log_helper 的 ECO_INFO/ECO_WARN/ECO_ERROR。
 * 未来如需增加 RT 路径判断（例如禁止在中断上下文中打印），在此处扩展。
 */
#pragma once

#include <log_helper/LogHelper.h>

/* ============================================================================
 *  日志宏 — 透传 ECO_*
 * ============================================================================ */

/**
 * @brief 信息级别日志
 */
#define EXO_INFO(fmt, ...)   ECO_INFO(fmt, ##__VA_ARGS__)

/**
 * @brief 警告级别日志
 */
#define EXO_WARN(fmt, ...)   ECO_WARN(fmt, ##__VA_ARGS__)

/**
 * @brief 错误级别日志
 */
#define EXO_ERROR(fmt, ...)  ECO_ERROR(fmt, ##__VA_ARGS__)
