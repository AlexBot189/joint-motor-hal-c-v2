/*
 * exo_state_machine.h — 7 状态机
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 状态: INIT → DISCOVERY → READY → CALIBRATING → ENABLED → RUNNING
 *       任意状态 → FAULT
 *
 * 使用: state_transition(new_state) 一次调用完成 exit+enter
 */
#pragma once

#include "exo_shm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*enter_fn)(void);
typedef void (*exit_fn)(void);

extern exo_state_t g_exo_state;

const char* state_name(exo_state_t s);

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

extern enter_fn enter_hooks[EXO_STATE_COUNT];
extern exit_fn  exit_hooks[EXO_STATE_COUNT];

bool state_transition_allowed(exo_state_t from, exo_state_t to);
bool state_transition(exo_state_t to);

#ifdef __cplusplus
}
#endif
