/*
 * exo_node_context.h — 全局上下文 (供状态机 enter/exit 钩子访问)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * enter/exit 钩子为 void(void) 无参数函数,
 * 通过此全局指针访问 HAL / SHM / 配置.
 */
#pragma once

extern "C" {
#include "motor_hal.h"
#include "exo_shm.h"
}

namespace stark_periph_manager_node {

struct ExoNodeContext {
    motor_hal_t* hal          = nullptr;
    exo_shm_t*   shm          = nullptr;
    int          motor_count  = 2;
    int          startup_timeout_sec = 30;

    /* 校准相关 */
    bool         auto_calib       = false;
    void*        calib_ctx        = nullptr;  /* motor_calib_t* opaque */
    bool         calib_running    = false;

    /* 传感器透传 */
    uint16_t     sensor_period_ms = 1;        /* ms */
    uint8_t      sensor_bus_format = 3;       /* 3=CANFD BRS, 0=Classic CAN */
    int          calib_timeout_ms = 10000;
};

}  /* namespace stark_periph_manager_node */

extern stark_periph_manager_node::ExoNodeContext* g_ctx;
