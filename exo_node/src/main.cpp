/*
 * main.cpp — 节点入口
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 启动流程:
 *   1. CanDispatcher::InitDispatcher() — CANFD + 电机注册 + recv + SHM
 *   2. wait_bootup_and_startup() — 轮询电机上线
 *   3. ExoRtWorker::Start() — RT 线程 (torque=0, 等算法 cmd)
 *   4. 主循环: process_pending_startups + 状态检测 + ros::spinOnce
 *   5. 退出: PDO estop → recv_stop → destroy
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
    /* 最后再 drain 一次, 清空残留 */
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

/* main */

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

    state_transition(STATE_INIT);
    ECO_INFO_NEW("[main] CANFD ready, INIT done");

    /* 步骤 2: 启动 RT 工作线程 (IMU 在 CanDispatcher 中已初始化) */

    int motor_count = g_dispatcher->GetMotorCount();
    g_rt_worker = new ExoRtWorker(hal, shm,
                                  g_dispatcher->GetCtrl(),
                                  g_dispatcher->GetImuSensor(),
                                  motor_count);

    /* 注入 config.json 配置 (读失败则保持默认值) */
    g_rt_worker->SetSafetyConfig(g_dispatcher->GetSafetyConfig());

    RtConfig rt_cfg = g_dispatcher->GetRtConfig();
    if (!enable_rt) {
        rt_cfg.enable_rt = false;  /* --no-rt 覆盖 */
    }
    g_rt_worker->SetRtConfig(rt_cfg);
    g_rt_worker->Start();

    ECO_INFO_NEW("[main] RT worker started (1KHz, {})",
                 enable_rt ? "SCHED_FIFO 90" : "SCHED_OTHER");

    /* 启动日志 drain 线程 */
    pthread_t log_tid;
    pthread_create(&log_tid, nullptr, log_drain_thread, nullptr);
    ECO_INFO_NEW("[main] log drain thread started");

    /* 步骤 3: 状态机驱动 — enter_discovery 阻塞等待电机上线 */

    g_node_ctx.hal          = hal;
    g_node_ctx.shm          = shm;
    g_node_ctx.motor_count  = motor_count;
    g_node_ctx.startup_timeout_sec = (motor_count == 1) ? 5 : 30;

    /* 从 config.json 读取透传配置 */
    g_node_ctx.sensor_period_ms = g_dispatcher->GetSensorPeriodMs();
    g_node_ctx.sensor_bus_format = g_dispatcher->GetSensorBusFormat();

    /* 从 config.json 读取校准配置 */
    g_node_ctx.auto_calib       = g_dispatcher->GetCalibAuto();
    g_node_ctx.calib_timeout_ms = g_dispatcher->GetCalibTimeoutMs();

    ECO_INFO_NEW("[main] config: sensor_period={}ms, bus_fmt={}",
                 g_node_ctx.sensor_period_ms, g_node_ctx.sensor_bus_format);

    /* 启动 SYNC 线程: TPDO 依赖 SYNC 帧触发反馈上报 */
    if (hal) {
        int sync_ret = motor_hal_sync_start(hal, 5000);  /* 5ms = 200Hz */
        ECO_INFO_NEW("[main] SYNC thread {} (ret={})",
                     sync_ret == 0 ? "started" : "FAILED", sync_ret);
    }

    state_transition(STATE_DISCOVERY);  /* 阻塞: enter_discovery 做实际探测 */

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

    /* 步骤 5: 主循环 */

    ECO_INFO_NEW("[main] node ready");
    ECO_INFO_NEW("[main]   state: {}", state_name(g_exo_state));
    ECO_INFO_NEW("[main]   motors online: 0x{:02X}", shm ? shm->motor_online : 0);

    while (g_running) {
#ifdef ENABLE_ROS
        ros::spinOnce();
#endif

        /* 轮询 auto-startup (处理热插拔) + sensor 透传自动配置 */
        if (hal) {
            motor_hal_process_pending_startups(hal);
            update_shm_online(hal, shm, g_node_ctx.motor_count);

            /* startup 完成后自动配置 sensor 透传 */
            for (uint8_t id = 1; id <= g_node_ctx.motor_count; id++) {
                if (!g_sensor_configured[id - 1]) {
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
        }

        /* 自动推进状态 */
        if (g_rt_worker && shm) {
            /* 按键触发校准 (预留: 外部设置 g_node_ctx.calib_requested = true) */
            if (g_node_ctx.calib_requested && g_exo_state == STATE_READY) {
                g_node_ctx.calib_requested = false;
                ECO_INFO_NEW("[main] calib requested, entering CALIBRATING");
                state_transition(STATE_CALIBRATING);
            }
            if (g_exo_state == STATE_READY) {
                if (g_node_ctx.auto_calib) {
                    ECO_INFO_NEW("[main] auto-calib enabled, entering CALIBRATING");
                    state_transition(STATE_CALIBRATING);
                } else {
                    ECO_INFO_NEW("[main] motors ready, entering ENABLED");
                    state_transition(STATE_ENABLED);
                }
            }
            /* 校准轮询 (主循环 100ms 调用一次) */
            if (g_exo_state == STATE_CALIBRATING && g_node_ctx.calib_ctx) {
                motor_calib_state_t result = motor_calib_poll(
                    (motor_calib_t*)g_node_ctx.calib_ctx);
                if (result == MOTOR_CALIB_DONE) {
                    ECO_INFO_NEW("[main] calibration DONE, entering READY");
                    if (shm) shm->calib_state = 2;
                    state_transition(STATE_READY);
                } else if (result == MOTOR_CALIB_TIMEOUT) {
                    ECO_WARN_NEW("[main] calibration TIMEOUT, entering READY (degraded)");
                    if (shm) shm->calib_state = 3;
                    state_transition(STATE_READY);
                }
            }
            /* FAULT 恢复后, handshake 已在 RT 线程完成,
             * 这里补发 ENABLED→RUNNING */
            if (g_exo_state == STATE_ENABLED && g_rt_worker->IsHandshakeDone()) {
                ECO_INFO_NEW("[main] algo connected → RUNNING");
                state_transition(STATE_RUNNING);
            }
            /* 处理 RT 线程的状态切换请求 */
            exo_state_t pending = g_rt_worker->GetPendingState();
            if (pending == STATE_RUNNING && g_exo_state == STATE_FAULT) {
                /* 算法重连 → FAULT 自动恢复到 READY */
                ECO_INFO_NEW("[main] FAULT → READY (algo reconnect)");
                state_transition(STATE_READY);
            }
            else if (pending == STATE_RUNNING && g_exo_state == STATE_ENABLED) {
                state_transition(STATE_RUNNING);
            }
            else if (pending == STATE_FAULT && g_exo_state != STATE_FAULT) {
                state_transition(STATE_FAULT);

                /* 补发 SDO DS402 Shutdown (非 RT 路径, 可阻塞)
                 * RT 线程已通过 PDO enable=false + torque=0 完成降险. */
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

        /* 同步 SHM node_state (主线程权威, RT 线程不碰) */
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
