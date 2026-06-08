/*
 * @file exo_state_machine.h
 * @brief 7 状态机 — 声明
 *
 * 状态流转:
 *   INIT(0) → DISCOVERY(1) → READY(2) → CALIBRATING(3) → ENABLED(4) → RUNNING(5)
 *   任意状态 → FAULT(6)   (异常)
 *   FAULT → READY         (人工恢复)
 *
 * 所有 enter_*() / exit_*() 仅声明，实现留空供后续填充。
 */
#pragma once

#include "exo_shm.h"

/* ============================================================================
 *  全局状态变量 (extern)
 * ============================================================================ */

/** @brief 系统当前状态，由 state_machine 模块维护 */
extern exo_state_t g_exo_state;

/* ============================================================================
 *  状态转换控制
 * ============================================================================ */

/**
 * @brief 执行状态转换
 *
 * 顺序: exit_hooks[旧状态] → 设置新状态 → 日志 → enter_hooks[新状态]
 *
 * @param new_state 目标状态
 * @return true 转换成功, false 非法转换（不执行任何操作）
 */
bool state_transition(exo_state_t new_state);

/**
 * @brief 检查状态转换是否合法
 *
 * @param from 当前状态
 * @param to   目标状态
 * @return true 允许转换
 */
bool state_transition_allowed(exo_state_t from, exo_state_t to);

/**
 * @brief 获取状态的可读名称字符串
 *
 * @param state 状态枚举值
 * @return C 风格字符串（静态常量，调用方不得释放）
 */
const char* state_name(exo_state_t state);

/* ============================================================================
 *  状态钩子函数声明 (enter / exit)
 *
 *  各函数实现留空，由业务模块按需填充。
 * ============================================================================ */

/* ---------- enter 钩子 ---------- */
void enter_init(exo_state_t prev);
void enter_discovery(exo_state_t prev);
void enter_ready(exo_state_t prev);
void enter_calibrating(exo_state_t prev);
void enter_enabled(exo_state_t prev);
void enter_running(exo_state_t prev);
void enter_fault(exo_state_t prev);

/* ---------- exit 钩子 ---------- */
void exit_init(void);
void exit_discovery(void);
void exit_ready(void);
void exit_calibrating(void);
void exit_enabled(void);
void exit_running(void);
void exit_fault(void);

/* ============================================================================
 *  钩子函数表类型 (内部使用，头文件暴露供测试)
 * ============================================================================ */

/** @brief enter 钩子函数指针类型 */
using enter_hook_t = void (*)(exo_state_t);

/** @brief exit 钩子函数指针类型 */
using exit_hook_t = void (*)(void);

/** @brief enter 钩子表，下标对应 exo_state_t 枚举值 */
extern enter_hook_t enter_hooks[EXO_STATE_COUNT];

/** @brief exit 钩子表，下标对应 exo_state_t 枚举值 */
extern exit_hook_t exit_hooks[EXO_STATE_COUNT];
