/*
 * exo_state_machine.cpp — 7 状态机实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * enter/exit 钩子为日志+状态记录, 业务逻辑在 main.cpp 中驱动.
 */
#include "exo_state_machine.h"
#include "exo_node_context.h"
#include "motor_calib.h"
#include <log_helper/LogHelper.h>
#include <unistd.h>

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

/* enter */

void enter_init(void) {
    ECO_INFO_NEW("[StateMachine] enter INIT");
}

void enter_discovery(void) {
    ECO_INFO_NEW("[StateMachine] enter DISCOVERY — waiting for motors...");

    if (!g_ctx || !g_ctx->hal || !g_ctx->shm) {
        ECO_ERROR_NEW("[StateMachine] DISCOVERY: no context, skip");
        return;
    }

    motor_hal_t* hal = g_ctx->hal;
    exo_shm_t*   shm = g_ctx->shm;
    int motor_count  = g_ctx->motor_count;
    int timeout_ms   = g_ctx->startup_timeout_sec * 1000;
    int elapsed_ms   = 0;
    const int interval_ms = 200;

    /* 阻塞轮询直到全部电机完成 auto-startup (DS402使能) */
    while (elapsed_ms < timeout_ms) {
        int started = motor_hal_process_pending_startups(hal);
        if (started > 0) {
            ECO_INFO_NEW("[StateMachine] auto-startup: {} motor(s) done", started);
        }

        /* 反馈缓存检测在线 (零延迟) */
        int online_count = 0;
        uint8_t online_mask = 0;
        for (uint8_t id = 1; id <= (uint8_t)motor_count; ++id) {
            motor_feedback_t fb;
            if (motor_hal_get_feedback(hal, id, &fb) == 0 && fb.status_byte != 0) {
                online_mask |= (1 << (id - 1));
                online_count++;
            }
        }
        shm->motor_online = online_mask;

        if (online_count == motor_count) {
            ECO_INFO_NEW("[StateMachine] all {} motors online (0x{:02X}) → READY",
                         online_count, online_mask);
            shm->node_state = STATE_READY;
            state_transition(STATE_READY);
            return;
        }

        usleep(interval_ms * 1000);
        elapsed_ms += interval_ms;

        /* 检测安全标志, RT 线程可能已触发 FAULT */
        if (shm->motor_severity == MOTOR_FAULT) {
            ECO_WARN_NEW("[StateMachine] FAULT detected during DISCOVERY, exit");
            shm->node_state = STATE_FAULT;
            state_transition(STATE_FAULT);
            return;
        }
    }

    ECO_ERROR_NEW("[StateMachine] DISCOVERY timeout ({}s), entering READY anyway",
                  g_ctx->startup_timeout_sec);
    state_transition(STATE_READY);
}

void enter_ready(void) {
    ECO_INFO_NEW("[StateMachine] enter READY — motors online, waiting calib");
}

void enter_calibrating(void) {
    ECO_INFO_NEW("[StateMachine] enter CALIBRATING");

    if (!g_ctx || !g_ctx->hal) {
        ECO_ERROR_NEW("[StateMachine] CALIBRATING: no context");
        return;
    }

    if (!g_ctx->calib_ctx) {
        g_ctx->calib_ctx = motor_calib_create(g_ctx->hal);
    }
    if (!g_ctx->calib_ctx) {
        ECO_ERROR_NEW("[StateMachine] CALIBRATING: create failed");
        return;
    }

    motor_calib_config_t calib_cfg = {0};
    calib_cfg.motor_id_r  = (g_ctx->motor_count >= 1) ? 1 : 0;
    calib_cfg.motor_id_l  = (g_ctx->motor_count >= 2) ? 2 : 0;
    calib_cfg.timeout_ms  = g_ctx->calib_timeout_ms;
    calib_cfg.angle_threshold_deg = 1.0f;
    calib_cfg.ctrl_mode   = MOTOR_MODE_CURRENT;

    int ret = motor_calib_start((motor_calib_t*)g_ctx->calib_ctx, &calib_cfg);
    if (ret != 0) {
        ECO_ERROR_NEW("[StateMachine] CALIBRATING: start failed");
        motor_calib_destroy((motor_calib_t*)g_ctx->calib_ctx);
        g_ctx->calib_ctx = nullptr;
        return;
    }

    g_ctx->calib_running = true;
    if (g_ctx->shm) g_ctx->shm->calib_state = 1;
}

void enter_enabled(void) {
    ECO_INFO_NEW("[StateMachine] enter ENABLED");
    /* PDO enable 由算法触发, sensor 透传在 auto-startup 完成时配置 */
}

void enter_running(void) {
    ECO_INFO_NEW("[StateMachine] enter RUNNING");
}

void enter_fault(void) {
    ECO_INFO_NEW("[StateMachine] enter FAULT — emergency stop");
}

/* exit */

void exit_init(void) {}
void exit_discovery(void) {}
void exit_ready(void) {}
void exit_calibrating(void) {
    if (g_ctx && g_ctx->calib_ctx) {
        motor_calib_destroy((motor_calib_t*)g_ctx->calib_ctx);
        g_ctx->calib_ctx = nullptr;
    }
    g_ctx->calib_running = false;
    if (g_ctx && g_ctx->shm) g_ctx->shm->calib_state = 0;
}
void exit_enabled(void) {
    ECO_INFO_NEW("[StateMachine] exit ENABLED — stop sensor passthrough");
    if (g_ctx && g_ctx->hal) {
        uint8_t online = g_ctx->shm ? g_ctx->shm->motor_online : 0;
        for (uint8_t id = 1; id <= (uint8_t)g_ctx->motor_count; id++) {
            if (!(online & (1 << (id - 1)))) continue;
            motor_hal_sensor_stop(g_ctx->hal, id);
        }
    }
}
void exit_running(void) {
    ECO_WARN_NEW("[StateMachine] exit RUNNING — control suspended");
}
void exit_fault(void) {
    ECO_INFO_NEW("[StateMachine] exit FAULT — attempting recovery");
}

/* 钩子表 */

enter_fn enter_hooks[EXO_STATE_COUNT] = {
    enter_init, enter_discovery, enter_ready,
    enter_calibrating, enter_enabled, enter_running, enter_fault,
};

exit_fn exit_hooks[EXO_STATE_COUNT] = {
    exit_init, exit_discovery, exit_ready,
    exit_calibrating, exit_enabled, exit_running, exit_fault,
};

/* state_transition_allowed */

bool state_transition_allowed(exo_state_t from, exo_state_t to)
{
    if (from == to) return true;  /* INIT→INIT etc allowed */
    if (to == STATE_FAULT) return true;
    if (from == STATE_FAULT && to == STATE_READY) return true;
    if (from == STATE_CALIBRATING && to == STATE_READY) return true;
    if (from == STATE_READY && to == STATE_ENABLED) return true;  /* 跳过校准直达使能 */
    if ((int)to == (int)from + 1) return true;
    return false;
}

/* state_transition */

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
