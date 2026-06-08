/*
 * @file exo_state_machine.cpp
 * @brief 7 状态机实现
 *
 * 状态转换只需调 state_transition(new_state),
 * 内部自动执行 exit_hooks[旧] → 设状态 → enter_hooks[新]。
 *
 * enter_* / exit_* 钩子由外部模块填充具体逻辑,
 * 默认操作为空 (只打日志)。
 */
#include "exo_state_machine.h"
#include <log_helper/LogHelper.h>

#include <cstdio>

/* ================================================================
 * 全局状态
 * ================================================================ */

exo_state_t g_exo_state = STATE_INIT;

/* ================================================================
 * 状态名称
 * ================================================================ */

const char* state_name(exo_state_t s)
{
    switch (s) {
    case STATE_INIT:        return "INIT";
    case STATE_DISCOVERY:   return "DISCOVERY";
    case STATE_READY:       return "READY";
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_ENABLED:     return "ENABLED";
    case STATE_RUNNING:     return "RUNNING";
    case STATE_FAULT:       return "FAULT";
    default:                return "UNKNOWN";
    }
}

/* ================================================================
 * 默认 enter_* 钩子 (空实现, 由 main.cpp / CanDispatcher 后续填充)
 * ================================================================ */

void enter_init(void)
{
    ECO_INFO("[StateMachine] enter INIT — CAN/SHM initialized, waiting for motors");
}

void enter_discovery(void)
{
    ECO_INFO("[StateMachine] enter DISCOVERY — probing motor online status");
}

void enter_ready(void)
{
    ECO_INFO("[StateMachine] enter READY — motors online, waiting for calibration");
}

void enter_calibrating(void)
{
    ECO_INFO("[StateMachine] enter CALIBRATING — executing zero-position calibration");
}

void enter_enabled(void)
{
    ECO_INFO("[StateMachine] enter ENABLED — motors enabled (torque=0), waiting for algorithm cmd");
}

void enter_running(void)
{
    ECO_INFO("[StateMachine] enter RUNNING — algorithm controlling motors");
}

void enter_fault(void)
{
    ECO_INFO("[StateMachine] enter FAULT — emergency stop, waiting for manual recovery");
}

/* ================================================================
 * 默认 exit_* 钩子
 * ================================================================ */

void exit_init(void)
{
    /* INIT 退出时无需特殊操作 */
}

void exit_discovery(void)
{
    /* DISCOVERY 退出: 电机上线确认完毕 */
}

void exit_ready(void)
{
    /* READY 退出: 进入校准阶段 */
}

void exit_calibrating(void)
{
    /* CALIBRATING 退出: 校准完成或失败 */
}

void exit_enabled(void)
{
    /* ENABLED 退出: 算法已握手 */
}

void exit_running(void)
{
    /* RUNNING 退出: 正常停止或故障 */
    ECO_WARN("[StateMachine] exit RUNNING — motor control suspended");
}

void exit_fault(void)
{
    /* FAULT 退出: 故障恢复 */
    ECO_INFO("[StateMachine] exit FAULT — attempting recovery");
}

/* ================================================================
 * 钩子函数表
 * ================================================================ */

enter_fn enter_hooks[EXO_STATE_COUNT] = {
    enter_init,
    enter_discovery,
    enter_ready,
    enter_calibrating,
    enter_enabled,
    enter_running,
    enter_fault,
};

exit_fn exit_hooks[EXO_STATE_COUNT] = {
    exit_init,
    exit_discovery,
    exit_ready,
    exit_calibrating,
    exit_enabled,
    exit_running,
    exit_fault,
};

/* ================================================================
 * state_transition_allowed
 *
 * 允许的转换:
 *   - 正向: INIT→DISCOVERY→READY→CALIBRATING→ENABLED→RUNNING
 *   - 故障: 任意状态→FAULT
 *   - 恢复: FAULT→READY
 * ================================================================ */

bool state_transition_allowed(exo_state_t from, exo_state_t to)
{
    /* 自循环: 不操作 */
    if (from == to) {
        return false;
    }

    /* 任意状态可进 FAULT */
    if (to == STATE_FAULT) {
        return true;
    }

    /* FAULT 恢复 */
    if (from == STATE_FAULT && to == STATE_READY) {
        return true;
    }

    /* 正向: 顺序前进 */
    if ((int)to == (int)from + 1) {
        return true;
    }

    return false;
}

/* ================================================================
 * state_transition
 *
 * 1. 合法性检查
 * 2. exit_hooks[from]()
 * 3. g_exo_state = to
 * 4. enter_hooks[to]()
 * ================================================================ */

bool state_transition(exo_state_t to)
{
    exo_state_t from = g_exo_state;

    if (!state_transition_allowed(from, to)) {
        ECO_WARN("[StateMachine] invalid transition: %s → %s (denied)",
                 state_name(from), state_name(to));
        return false;
    }

    /* 离开旧状态 */
    if (exit_hooks[from]) {
        exit_hooks[from]();
    }

    /* 设置新状态 */
    g_exo_state = to;

    /* 进入新状态 */
    if (enter_hooks[to]) {
        enter_hooks[to]();
    }

    ECO_INFO("[StateMachine] %s → %s", state_name(from), state_name(to));
    return true;
}
