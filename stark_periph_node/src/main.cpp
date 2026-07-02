/*
 * main.cpp — 节点入口
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 启动流程 (无阻塞):
 *   1. CanDispatcher::InitDispatcher() — CANFD + 电机注册 + recv + SHM
 *   2. state_transition(STATE_BOOTING)
 *   3. main_loop_run() — 主循环 (状态分发)
 *   4. 退出: sync_stop ,  RT stop ,  calib destroy ,  dispatcher destroy
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
#include "motor/motor_init.h"
#include "motor/motor_rt_worker.h"
#include "motor/motor_state.h"
#include "motor/motor_context.h"
#include "imu/imu_sensor.h"
#include "utils/rt_log.h"
#include "utils/factory.h"
#include "main_loop/main_loop.h"
#include "stark_shm.h"

using namespace stark_periph_manager_node;

/* 全局状态 */

volatile int g_running = 1;
volatile int g_log_running = 1;
CanDispatcher*    g_dispatcher = nullptr;
StarkRtWorker*      g_rt_worker  = nullptr;
static StarkRtLog   g_rt_log_instance;
StarkRtLog*         g_rt_log = &g_rt_log_instance;

/* 全局上下文 (状态机 enter/exit 钩子访问) */
static StarkNodeContext g_node_ctx;
StarkNodeContext*       g_ctx = &g_node_ctx;

/* log_drain_thread — 在 main_loop.cpp 中定义 */
extern void* log_drain_thread(void*);

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    ECO_INFO_NEW("[main] stark_periph_manager_node starting...");

    /* 解析参数 */
    std::string config_path = "/data/config/stark/stark_config.json";
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
    stark_shm_t*   shm = g_dispatcher->GetShm();
    int motor_count  = g_dispatcher->GetMotorCount();

    state_transition(STATE_BOOTING);
    ECO_INFO_NEW("[main] CANFD ready, BOOTING");

    /* 步骤 2: 创建 RT 工作线程 (先不 Start, 等电机上线) */

    g_rt_worker = new StarkRtWorker(hal, shm,
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
    g_node_ctx.report_auto_enable = g_dispatcher->GetReportAutoEnable();
    g_node_ctx.report_period_ms   = g_dispatcher->GetReportPeriodMs();

    ECO_INFO_NEW("[main] config: motor_count={} sensor_period={}ms bus_fmt={} auto_calib={}",
                 motor_count, g_node_ctx.sensor_period_ms,
                 g_node_ctx.sensor_bus_format, g_node_ctx.auto_calib);

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

    main_loop_run(hal, shm, motor_count, g_dispatcher, g_rt_worker, enable_rt);

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
