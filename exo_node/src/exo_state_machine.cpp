/*
 * @file exo_state_machine.cpp
 * @brief 7 状态机 — 实现
 *
 * 状态流转:
 *   INIT(0) → DISCOVERY(1) → READY(2) → CALIBRATING(3) → ENABLED(4) → RUNNING(5)
 *   任意状态 → FAULT(6)
 *   FAULT → READY (人工恢复)
 */
#include "exo_state_machine.h"
#include "exo_log.h"

/* ============================================================================
 *  全局变量定义
 * ============================================================================ */

exo_state_t g_exo_state = exo_state_t::STATE_INIT;

/* ============================================================================
 *  钩子函数表 — 初始化为空实现
 * ============================================================================ */

/* ---------- enter 钩子 ---------- */
void enter_init(exo_state_t prev)       { (void)prev; }
void enter_discovery(exo_state_t prev)  { (void)prev; }
void enter_ready(exo_state_t prev)      { (void)prev; }
void enter_calibrating(exo_state_t prev){ (void)prev; }
void enter_enabled(exo_state_t prev)    { (void)prev; }
void enter_running(exo_state_t prev)    { (void)prev; }
void enter_fault(exo_state_t prev)      { (void)prev; }

/* ---------- exit 钩子 ---------- */
void exit_init(void)       {}
void exit_discovery(void)  {}
void exit_ready(void)      {}
void exit_calibrating(void){}
void exit_enabled(void)    {}
void exit_running(void)    {}
void exit_fault(void)      {}

/* ---------- 钩子函数表 ---------- */
enter_hook_t enter_hooks[EXO_STATE_COUNT] = {
    enter_init,
    enter_discovery,
    enter_ready,
    enter_calibrating,
    enter_enabled,
    enter_running,
    enter_fault,
};

exit_hook_t exit_hooks[EXO_STATE_COUNT] = {
    exit_init,
    exit_discovery,
    exit_ready,
    exit_calibrating,
    exit_enabled,
    exit_running,
    exit_fault,
};

/* ============================================================================
 *  state_name — 状态名称字符串
 * ============================================================================ */

const char* state_name(exo_state_t state)
{
    switch (state) {
    case exo_state_t::STATE_INIT:        return "INIT";
    case exo_state_t::STATE_DISCOVERY:   return "DISCOVERY";
    case exo_state_t::STATE_READY:       return "READY";
    case exo_state_t::STATE_CALIBRATING: return "CALIBRATING";
    case exo_state_t::STATE_ENABLED:     return "ENABLED";
    case exo_state_t::STATE_RUNNING:     return "RUNNING";
    case exo_state_t::STATE_FAULT:       return "FAULT";
    default:                             return "UNKNOWN";
    }
}

/* ============================================================================
 *  state_transition_allowed — 合法性检查
 * ============================================================================ */

bool state_transition_allowed(exo_state_t from, exo_state_t to)
{
    /* 同一状态允许（幂等转换） */
    if (from == to) {
        return true;
    }

    /* 任意状态可进入 FAULT */
    if (to == exo_state_t::STATE_FAULT) {
        return true;
    }

    /* FAULT 只能恢复到 READY */
    if (from == exo_state_t::STATE_FAULT) {
        return (to == exo_state_t::STATE_READY);
    }

    /* 正常路径: 仅允许顺序推进 (0→1→2→3→4→5) */
    uint32_t from_idx = static_cast<uint32_t>(from);
    uint32_t to_idx   = static_cast<uint32_t>(to);

    /* 目标必须比当前大 1 */
    return (to_idx == from_idx + 1);
}

/* ============================================================================
 *  state_transition — 执行状态转换
 * ============================================================================ */

bool state_transition(exo_state_t new_state)
{
    exo_state_t old_state = g_exo_state;

    /* 合法性检查 */
    if (!state_transition_allowed(old_state, new_state)) {
        EXO_WARN("[state_machine] 非法状态转换: %s → %s",
                 state_name(old_state), state_name(new_state));
        return false;
    }

    /* 1. 执行 exit 钩子 */
    uint32_t old_idx = static_cast<uint32_t>(old_state);
    if (exit_hooks[old_idx] != nullptr) {
        exit_hooks[old_idx]();
    }

    /* 2. 设置新状态 */
    g_exo_state = new_state;

    /* 3. 日志 */
    EXO_INFO("[state_machine] 状态转换: %s → %s",
             state_name(old_state), state_name(new_state));

    /* 4. 执行 enter 钩子 */
    uint32_t new_idx = static_cast<uint32_t>(new_state);
    if (enter_hooks[new_idx] != nullptr) {
        enter_hooks[new_idx](old_state);
    }

    return true;
}
