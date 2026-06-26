/*
 * main.cpp — 节点入口
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 启动流程 (无阻塞):
 *   1. CanDispatcher::InitDispatcher() — CANFD + 电机注册 + recv + SHM
 *   2. state_transition(STATE_BOOTING)
 *   3. 主循环:
 *      a) 电机发现检测 → BOOTING→READY
 *      b) RT线程启动 (至少1电机在线)
 *      c) SYNC启动 (至少1电机在线)
 *      d) READY: 校准+握手 → RUNNING
 *      e) RUNNING: 不做特殊处理
 *      f) FAULT: 算法失联降险 → 恢复 READY
 *   4. 退出: PDO estop → recv_stop → destroy
 */
#include <signal.h>
#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "motor_hal.h"
#include "motor_calib.h"
}

#include <log_helper/LogHelper.h>
#include "src/CanDispatcher.h"
#include "core/exo_rt_worker.h"
#include "core/exo_rt_log.h"
#include "core/exo_state_machine.h"
#include "core/exo_node_context.h"
#include "core/exo_imu_sensor.h"
#include "src/Factory.h"
#include "exo_shm.h"

using namespace stark_periph_manager_node;

/* 全局状态 */

static volatile int g_running = 1;
static volatile int g_log_running = 1;
static CanDispatcher*    g_dispatcher = nullptr;
static ExoRtWorker*      g_rt_worker  = nullptr;
static ExoRtLog          g_rt_log_instance;
ExoRtLog*                g_rt_log = &g_rt_log_instance;

/* 全局上下文 (状态机 enter/exit 钩子访问) */
static ExoNodeContext    g_node_ctx;
ExoNodeContext*          g_ctx = &g_node_ctx;

/* sensor 透传防重配置标记 */
static bool g_sensor_configured[EXO_MAX_MOTORS];

/* 校准触发门控 (防止重复进入校准) */
static bool g_calib_triggered = false;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/*
 * log_drain_thread — 从 ring buffer drain → ECO_INFO_NEW
 * RT 线程写 ring buffer (lock-free, <1μs), 此非 RT 线程负责 drain.
 */

static void rt_log_output_fn(const char* msg)
{
    ECO_INFO_NEW("[RT] {}", msg);
}

static void* log_drain_thread(void*)
{
    while (g_log_running) {
        if (g_rt_log) {
            g_rt_log->Drain(rt_log_output_fn);
        }
        usleep(50000);  /* 50ms drain */
    }
    if (g_rt_log) {
        g_rt_log->Drain(rt_log_output_fn);
    }
    return nullptr;
}

/*
 * update_shm_online — 更新 SHM motor_online 掩码
 */

static void update_shm_online(motor_hal_t* hal, exo_shm_t* shm, uint8_t motor_count)
{
    if (!hal || !shm) return;

    uint8_t mask = 0;
    for (uint8_t id = 1; id <= motor_count; ++id) {
        motor_state_t state = motor_hal_get_state(hal, id);
        if (state >= MOTOR_STATE_SWITCH_ON_DIS && state != MOTOR_STATE_UNKNOWN) {
            mask |= (1 << (id - 1));
        }
    }
    shm->motor_online = mask;
}

/*
 * all_motors_online — 检查是否所有已注册电机都已上线
 */

static bool all_motors_online(exo_shm_t* shm, int motor_count)
{
    if (!shm) return false;
    uint8_t expected = (uint8_t)((1 << motor_count) - 1);
    return (shm->motor_online & expected) == expected;
}

/*
 * any_motor_online — 检查是否至少1个电机在线
 */

static bool any_motor_online(exo_shm_t* shm, int motor_count)
{
    if (!shm) return false;
    for (int i = 0; i < motor_count; i++) {
        if (shm->motor_online & (1 << i)) return true;
    }
    return false;
}

/* main */

/*
 * poll_common — 每轮都执行的公共逻辑
 *   motor auto-startup 推进 + SHM online 掩码更新 + sensor 透传自动配置
 */

static void poll_common(motor_hal_t* hal, exo_shm_t* shm, uint8_t motor_count)
{
    if (!hal) return;

    motor_hal_process_pending_startups(hal);
    update_shm_online(hal, shm, motor_count);

    for (uint8_t id = 1; id <= motor_count; id++) {
        if (g_sensor_configured[id - 1]) continue;

        motor_state_t st = motor_hal_get_state(hal, id);
        if (st >= MOTOR_STATE_SWITCH_ON_DIS && st != MOTOR_STATE_UNKNOWN) {
            uint16_t period_div = (uint16_t)(g_node_ctx.sensor_period_ms * 4);
            int ret = motor_hal_sensor_config(hal, id, period_div,
                                              g_node_ctx.sensor_bus_format);
            if (ret == 0) {
                g_sensor_configured[id - 1] = true;
                ECO_INFO_NEW("[main] sensor passthrough: motor {} period={}ms bus={}",
                             id, g_node_ctx.sensor_period_ms,
                             g_node_ctx.sensor_bus_format == 3 ? "CANFD BRS" : "Classic CAN");
            }
        }
    }
}

/*
 * poll_booting — BOOTING 状态逻辑
 *   电机全在线 → READY, 懒启动 RT 线程和 SYNC
 */

static void poll_booting(exo_shm_t* shm, int motor_count,
                         bool enable_rt, bool& sync_started)
{
    if (all_motors_online(shm, motor_count)) {
        ECO_INFO_NEW("[main] all {} motors online (0x{:02X}) → READY",
                     motor_count, shm->motor_online);
        state_transition(STATE_READY);
        return;
    }

    /* 至少1个电机在线后启动 RT 线程和 SYNC */
    if (!any_motor_online(shm, motor_count)) return;

    if (g_rt_worker && !g_rt_worker->IsRunning()) {
        g_rt_worker->Start();
        ECO_INFO_NEW("[main] RT worker started (1KHz, {})",
                     enable_rt ? "SCHED_FIFO 90" : "SCHED_OTHER");
    }

    if (!sync_started) {
        motor_hal_t* hal = g_ctx->hal;
        if (hal) {
            int ret = motor_hal_sync_start(hal, 5000);
            ECO_INFO_NEW("[main] SYNC thread {} (ret={})",
                         ret == 0 ? "started" : "FAILED", ret);
        }
        sync_started = true;
    }
}

/*
 * poll_ready — READY 状态逻辑
 *   校准触发 / 校准轮询 / 校准完成+握手 → RUNNING
 */

static void poll_ready(motor_hal_t* hal, exo_shm_t* shm, int motor_count)
{
    /* 启动校准 */
    bool need_calib = !g_node_ctx.calib_done &&
                      (g_node_ctx.calib_requested || g_node_ctx.auto_calib);

    if (need_calib && !g_calib_triggered && !g_node_ctx.calib_running) {
        g_calib_triggered = true;
        g_node_ctx.calib_requested = false;

        ECO_INFO_NEW("[main] starting calibration (auto=%d)", g_node_ctx.auto_calib);

        if (!g_node_ctx.calib_ctx) {
            g_node_ctx.calib_ctx = motor_calib_create(hal);
        }

        if (g_node_ctx.calib_ctx) {
            motor_calib_config_t calib_cfg = {};
            calib_cfg.motor_id_r = (motor_count >= 1) ? 1 : 0;
            calib_cfg.motor_id_l = (motor_count >= 2) ? 2 : 0;
            calib_cfg.timeout_ms = g_node_ctx.calib_timeout_ms;
            calib_cfg.angle_threshold_deg = 1.0f;
            calib_cfg.ctrl_mode = MOTOR_MODE_CURRENT;

            int ret = motor_calib_start((motor_calib_t*)g_node_ctx.calib_ctx, &calib_cfg);
            if (ret == 0) {
                g_node_ctx.calib_running = true;
                if (shm) shm->calib_state = 1;
            } else {
                ECO_ERROR_NEW("[main] calib start failed");
                motor_calib_destroy((motor_calib_t*)g_node_ctx.calib_ctx);
                g_node_ctx.calib_ctx = nullptr;
            }
        }
    }

    /* 校准轮询 */
    if (g_node_ctx.calib_running && g_node_ctx.calib_ctx) {
        motor_calib_state_t result = motor_calib_poll(
            (motor_calib_t*)g_node_ctx.calib_ctx);

        if (result == MOTOR_CALIB_DONE) {
            ECO_INFO_NEW("[main] calibration DONE");
            g_node_ctx.calib_done = true;
            g_node_ctx.calib_running = false;
            if (shm) shm->calib_state = 2;
            g_calib_triggered = false;

            if (g_rt_worker) g_rt_worker->SetActive(true);
            if (g_rt_worker && g_rt_worker->IsHandshakeDone()) {
                ECO_INFO_NEW("[main] algo connected (post-calib) → RUNNING");
                state_transition(STATE_RUNNING);
            }
        } else if (result == MOTOR_CALIB_TIMEOUT) {
            ECO_WARN_NEW("[main] calibration TIMEOUT, entering RUNNING (degraded)");
            g_node_ctx.calib_done = true;
            g_node_ctx.calib_running = false;
            if (shm) shm->calib_state = 3;
            g_calib_triggered = false;

            if (g_rt_worker) g_rt_worker->SetActive(true);
            state_transition(STATE_RUNNING);
        }
    }

    /* 校准已完成不轮询: 检查算法握手直接进 RUNNING */
    if (g_node_ctx.calib_done && !g_node_ctx.calib_running) {
        if (g_rt_worker && !g_rt_worker->IsActive()) {
            g_rt_worker->SetActive(true);
        }
        if (g_rt_worker && g_rt_worker->IsHandshakeDone()) {
            ECO_INFO_NEW("[main] calib done + algo connected → RUNNING");
            state_transition(STATE_RUNNING);
        }
    }
}

/*
 * poll_rt_pending — 处理 RT 线程的状态切换请求
 *   FAULT 恢复 / 触发 FAULT
 */

static void poll_rt_pending(exo_shm_t* shm)
{
    if (!g_rt_worker) return;

    exo_state_t pending = g_rt_worker->GetPendingState();

    /* 算法重连: FAULT → READY, calib_done 保持 */
    if (pending == STATE_READY && g_exo_state == STATE_FAULT) {
        ECO_INFO_NEW("[main] FAULT → READY (algo reconnect, calib_done={})",
                     g_node_ctx.calib_done);
        state_transition(STATE_READY);
        return;
    }

    /* RT 线程触发 FAULT */
    if (pending == STATE_FAULT && g_exo_state != STATE_FAULT) {
        state_transition(STATE_FAULT);

        /* 补发 SDO DS402 Shutdown, RT 已通过 PDO 完成降险 */
        auto* ctrl = g_dispatcher->GetCtrl();
        if (ctrl) {
            for (uint8_t id = 1; id <= g_node_ctx.motor_count; id++) {
                if (shm->motor_online & (1 << (id - 1))) {
                    int ret = ctrl->SdoWrite(id, 0x6040, 0, 0x0006, 2);
                    ECO_INFO_NEW("[main] SDO Shutdown motor {}: {}",
                                 id, (ret == 0 ? "OK" : "FAIL"));
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    ECO_INFO_NEW("[main] stark_periph_manager_node starting...");

    /* 解析参数 */
    std::string config_path = "/data/config/stark/exo_config.json";
    bool enable_rt = true;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "--no-rt") == 0) {
            enable_rt = false;
        }
    }

    /* 步骤 1: 初始化 CANFD + 注册电机 + recv + SHM */

    g_dispatcher = new CanDispatcher();
    g_dispatcher->SetConfigPath(config_path);

    if (!g_dispatcher->InitDispatcher()) {
        ECO_ERROR_NEW("[main] CanDispatcher init failed");
        delete g_dispatcher;
        return 1;
    }

    motor_hal_t* hal = g_dispatcher->GetHal();
    exo_shm_t*   shm = g_dispatcher->GetShm();
    int motor_count  = g_dispatcher->GetMotorCount();

    state_transition(STATE_BOOTING);
    ECO_INFO_NEW("[main] CANFD ready, BOOTING");

    /* 步骤 2: 创建 RT 工作线程 (先不 Start, 等电机上线) */

    g_rt_worker = new ExoRtWorker(hal, shm,
                                  g_dispatcher->GetCtrl(),
                                  g_dispatcher->GetImuSensor(),
                                  motor_count);

    g_rt_worker->SetSafetyConfig(g_dispatcher->GetSafetyConfig());

    RtConfig rt_cfg = g_dispatcher->GetRtConfig();
    if (!enable_rt) {
        rt_cfg.enable_rt = false;
    }
    g_rt_worker->SetRtConfig(rt_cfg);

    ECO_INFO_NEW("[main] RT worker created (not started yet)");

    /* 启动日志 drain 线程 */
    pthread_t log_tid;
    pthread_create(&log_tid, nullptr, log_drain_thread, nullptr);
    ECO_INFO_NEW("[main] log drain thread started");

    /* 步骤 3: 注入全局上下文 */

    g_node_ctx.hal          = hal;
    g_node_ctx.shm          = shm;
    g_node_ctx.motor_count  = motor_count;

    /* 从 config.json 读取配置 */
    g_node_ctx.sensor_period_ms = g_dispatcher->GetSensorPeriodMs();
    g_node_ctx.sensor_bus_format = g_dispatcher->GetSensorBusFormat();
    g_node_ctx.auto_calib       = g_dispatcher->GetCalibAuto();
    g_node_ctx.calib_timeout_ms = g_dispatcher->GetCalibTimeoutMs();

    ECO_INFO_NEW("[main] config: motor_count={} sensor_period={}ms bus_fmt={} auto_calib={}",
                 motor_count, g_node_ctx.sensor_period_ms,
                 g_node_ctx.sensor_bus_format, g_node_ctx.auto_calib);

    bool sync_started = false;

    /* 步骤 4: 初始化 ROS (编译可选) */

#ifdef ENABLE_ROS
    ros::init(argc, argv, "stark_periph_manager_node");
    auto nh = std::make_shared<ros::NodeHandle>();
    auto ros_adapter = Factory::CreateRosListener(nh,
                        std::shared_ptr<IMsgInternalDispatcher>(g_dispatcher,
                        [](CanDispatcher*){}));
    g_dispatcher->RegisterObserver(ListenerType::ROS, ros_adapter);
    ECO_INFO_NEW("[main] ROS adapter enabled");
#endif

    /* 步骤 5: 主循环 (非阻塞, 状态分发) */

    ECO_INFO_NEW("[main] entering main loop (non-blocking)");

    while (g_running) {
#ifdef ENABLE_ROS
        ros::spinOnce();
#endif

        /* 公共: auto-startup + sensor 配置 */
        poll_common(hal, shm, (uint8_t)motor_count);

        /* 状态分发 */
        switch (g_exo_state) {
        case STATE_BOOTING:
            poll_booting(shm, motor_count, enable_rt, sync_started);
            break;
        case STATE_READY:
            poll_ready(hal, shm, motor_count);
            break;
        case STATE_RUNNING:
        case STATE_FAULT:
            break;
        default:
            break;
        }

        /* RT 线程状态切换请求 */
        poll_rt_pending(shm);

        /* 同步 SHM node_state */
        if (shm) {
            shm->node_state = g_exo_state;
        }

        usleep(100000);  /* 100ms 轮询 */
    }

    /* 步骤 6: 清理 */

    ECO_INFO_NEW("[main] shutting down...");

    /* 停止 SYNC 线程 */
    if (hal) {
        motor_hal_sync_stop(hal);
    }

    if (g_rt_worker) {
        g_rt_worker->Stop();
        delete g_rt_worker;
        g_rt_worker = nullptr;
    }

    /* 清理校准器 */
    if (g_node_ctx.calib_ctx) {
        motor_calib_destroy((motor_calib_t*)g_node_ctx.calib_ctx);
        g_node_ctx.calib_ctx = nullptr;
    }

    if (g_dispatcher) {
        g_dispatcher->DestroyDispatcher();
        delete g_dispatcher;
        g_dispatcher = nullptr;
    }

    /* 停日志 drain 线程 */
    g_log_running = 0;
    pthread_join(log_tid, nullptr);

    ECO_INFO_NEW("[main] stark_periph_manager_node stopped");
    return 0;
}
