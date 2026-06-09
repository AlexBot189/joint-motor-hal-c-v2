/*
 * exo_node_context.h — 全局上下文 (供状态机 enter/exit 钩子访问)
 *
 * enter/exit 钩子是 void(void) 无参数函数,
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
};

}  /* namespace stark_periph_manager_node */

extern stark_periph_manager_node::ExoNodeContext* g_ctx;
