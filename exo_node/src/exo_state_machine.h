/*
 * @file exo_state_machine.h
 * @brief 7 状态机声明 — stark_periph_manager_node 状态管理
 *
 * 状态枚举定义在 exo_shm.h (跨进程共享):
 *   STATE_INIT → DISCOVERY → READY → CALIBRATING → ENABLED → RUNNING
 *   任意状态 → STATE_FAULT
 *
 * 使用方式:
 *   state_transition(STATE_READY);  // 一次调用完成 exit+enter
 */
#pragma once

#include "exo_shm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 函数指针类型 ── */
typedef void (*enter_fn)(void);
typedef void (*exit_fn)(void);

/* ── 全局状态 ── */
extern exo_state_t g_exo_state;

/* ── 状态名 ── */
const char* state_name(exo_state_t s);

/* ── 钩子函数声明 (实现可在不同 .cpp 中填充具体逻辑) ── */
extern void enter_init(void);
extern void enter_discovery(void);
extern void enter_ready(void);
extern void enter_calibrating(void);
extern void enter_enabled(void);
extern void enter_running(void);
extern void enter_fault(void);

extern void exit_init(void);
extern void exit_discovery(void);
extern void exit_ready(void);
extern void exit_calibrating(void);
extern void exit_enabled(void);
extern void exit_running(void);
extern void exit_fault(void);

/* ── 钩子函数表 (可被外部替换) ── */
extern enter_fn enter_hooks[EXO_STATE_COUNT];
extern exit_fn  exit_hooks[EXO_STATE_COUNT];

/* ── 状态操作 ── */

/*
 * 检查状态转换是否允许
 */
bool state_transition_allowed(exo_state_t from, exo_state_t to);

/*
 * 执行状态转换: exit_hooks[旧] → 设新状态 → enter_hooks[新]
 * 返回: true=转换成功, false=不允许的转换
 */
bool state_transition(exo_state_t to);

#ifdef __cplusplus
}
#endif
