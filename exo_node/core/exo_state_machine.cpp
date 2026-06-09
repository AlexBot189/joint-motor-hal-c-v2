/*
 * exo_state_machine.cpp — 7 状态机实现
 *
 * enter/exit 钩子为日志+状态记录, 业务逻辑在 main.cpp 中驱动.
 */
#include "exo_state_machine.h"
#include <log_helper/LogHelper.h>

exo_state_t g_exo_state = STATE_INIT;

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

/* ── enter ── */

void enter_init(void) {
    ECO_INFO_NEW("[StateMachine] enter INIT");
}

void enter_discovery(void) {
    ECO_INFO_NEW("[StateMachine] enter DISCOVERY — probing motors");
}

void enter_ready(void) {
    ECO_INFO_NEW("[StateMachine] enter READY — motors online, waiting calib");
}

void enter_calibrating(void) {
    ECO_INFO_NEW("[StateMachine] enter CALIBRATING");
}

void enter_enabled(void) {
    ECO_INFO_NEW("[StateMachine] enter ENABLED — torque=0, waiting algorithm");
}

void enter_running(void) {
    ECO_INFO_NEW("[StateMachine] enter RUNNING");
}

void enter_fault(void) {
    ECO_INFO_NEW("[StateMachine] enter FAULT — emergency stop");
}

/* ── exit ── */

void exit_init(void) {}
void exit_discovery(void) {}
void exit_ready(void) {}
void exit_calibrating(void) {}
void exit_enabled(void) {}
void exit_running(void) {
    ECO_WARN_NEW("[StateMachine] exit RUNNING — control suspended");
}
void exit_fault(void) {
    ECO_INFO_NEW("[StateMachine] exit FAULT — attempting recovery");
}

/* ── 钩子表 ── */

enter_fn enter_hooks[EXO_STATE_COUNT] = {
    enter_init, enter_discovery, enter_ready,
    enter_calibrating, enter_enabled, enter_running, enter_fault,
};

exit_fn exit_hooks[EXO_STATE_COUNT] = {
    exit_init, exit_discovery, exit_ready,
    exit_calibrating, exit_enabled, exit_running, exit_fault,
};

/* ── state_transition_allowed ── */

bool state_transition_allowed(exo_state_t from, exo_state_t to)
{
    if (from == to) return false;
    if (to == STATE_FAULT) return true;
    if (from == STATE_FAULT && to == STATE_READY) return true;
    if ((int)to == (int)from + 1) return true;
    return false;
}

/* ── state_transition ── */

bool state_transition(exo_state_t to)
{
    exo_state_t from = g_exo_state;

    if (!state_transition_allowed(from, to)) {
        ECO_WARN_NEW("[StateMachine] denied: {} → {}",
                     state_name(from), state_name(to));
        return false;
    }

    if (exit_hooks[from]) exit_hooks[from]();
    g_exo_state = to;
    if (enter_hooks[to]) enter_hooks[to]();

    ECO_INFO_NEW("[StateMachine] {} → {}", state_name(from), state_name(to));
    return true;
}
