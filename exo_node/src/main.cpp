/*
 * main.cpp — stark_periph_manager_node 入口
 *
 * ★ 启动流程完全对齐 motor_tool daemon:
 *
 *   1. CanDispatcher::InitDispatcher()
 *      ├─ motor_hal_create + motor_hal_init("can0", 1M, 5M)
 *      ├─ 注册电机 (ID 1,2, auto_enable=true, tpdo_sync_count=1)
 *      ├─ motor_hal_recv_start (★ 必须在 startup 之前)
 *      └─ exo_shm_mgr_open
 *
 *   2. wait_bootup_and_startup()
 *      ├─ 轮询 motor_hal_process_pending_startups
 *      ├─ 检测反馈缓存确认在线
 *      └─ 状态: INIT → DISCOVERY → READY
 *
 *   3. ExoRtWorker::Start() — RT 线程 (torque=0, 等算法 cmd)
 *
 *   4. 主循环: process_pending_startups + 状态检测 + ros::spinOnce
 *
 *   5. 退出: NMT Stop → recv_stop → destroy
 */
#include <signal.h>
#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "motor_hal.h"
}

#include <log_helper/LogHelper.h>
#include "src/CanDispatcher.h"
#include "core/exo_rt_worker.h"
#include "core/exo_rt_log.h"
#include "core/exo_state_machine.h"
#include "core/exo_node_context.h"
#include "core/exo_mock_sensor.h"
#include "src/Factory.h"
#include "exo_shm.h"

using namespace stark_periph_manager_node;

/* ════════════════════════════════════════════════════════════════════
 * 全局状态
 * ════════════════════════════════════════════════════════════════════ */

static volatile int g_running = 1;
static volatile int g_log_running = 1;
static CanDispatcher*    g_dispatcher = nullptr;
static ExoRtWorker*      g_rt_worker  = nullptr;
static ExoMockSensor     g_mock_sensor;
static ExoRtLog          g_rt_log_instance;
ExoRtLog*                g_rt_log = &g_rt_log_instance;

/* 全局上下文 (状态机 enter/exit 钩子访问) */
static ExoNodeContext    g_node_ctx;
ExoNodeContext*          g_ctx = &g_node_ctx;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ════════════════════════════════════════════════════════════════════
 * log_drain_thread — 从 ring buffer drain → ECO_INFO_NEW
 *
 * RT 线程写 ring buffer (lock-free, <1μs),
 * 这个非 RT 线程负责 drain 到 log_helper.
 * ════════════════════════════════════════════════════════════════════ */

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
        usleep(50000);  /* 50ms drain 一次 */
    }
    /* 最后再 drain 一次, 清空残留 */
    if (g_rt_log) {
        g_rt_log->Drain(rt_log_output_fn);
    }
    return nullptr;
}

/* ════════════════════════════════════════════════════════════════════
 * update_shm_online — 更新 SHM motor_online 掩码
 * ════════════════════════════════════════════════════════════════════ */

static void update_shm_online(motor_hal_t* hal, exo_shm_t* shm)
{
    if (!hal || !shm) return;

    uint8_t mask = 0;
    for (uint8_t id = 1; id <= EXO_MOTOR_COUNT; ++id) {
        motor_state_t state = motor_hal_get_state(hal, id);
        if (state >= MOTOR_STATE_SWITCH_ON_DIS && state != MOTOR_STATE_UNKNOWN) {
            mask |= (1 << (id - 1));
        }
    }
    shm->motor_online = mask;
}

/* ════════════════════════════════════════════════════════════════════
 * main
 * ════════════════════════════════════════════════════════════════════ */

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    ECO_INFO_NEW("[main] stark_periph_manager_node starting...");

    /* ── 解析参数 ── */
    std::string config_path = "/data/config/stark/exo_config.json";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    /* ════════════════════════════════════════════════════════════
     * 步骤 1: 初始化 CANFD + 注册电机 + recv + SHM
     * (对齐 motor_tool daemon 的 tool_init → add_motor → recv_start)
     * ════════════════════════════════════════════════════════════ */

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

    /* ════════════════════════════════════════════════════════════
     * 步骤 2: 初始化 mock 传感器 (IMU + 气压计)
     * ════════════════════════════════════════════════════════════ */

    g_mock_sensor.Init();
    ECO_INFO_NEW("[main] mock sensor (IMU+Baro) ready");

    /* ════════════════════════════════════════════════════════════
     * 步骤 3: 启动 RT 工作线程
     * 先发 torque=0, 等算法 cmd 后才真正切入控制
     * ════════════════════════════════════════════════════════════ */

    g_rt_worker = new ExoRtWorker(hal, shm,
                                  g_dispatcher->GetCtrl(),
                                  &g_mock_sensor);
    g_rt_worker->Start();

    ECO_INFO_NEW("[main] RT worker started (1KHz, SCHED_FIFO 90)");

    /* ── 启动日志 drain 线程 ── */
    pthread_t log_tid;
    pthread_create(&log_tid, nullptr, log_drain_thread, nullptr);
    ECO_INFO_NEW("[main] log drain thread started");

    /* ════════════════════════════════════════════════════════════
     * 步骤 4: 状态机驱动 — enter_discovery 阻塞等待全部电机上线
     *         成功后自动切 READY, 超时也会切 READY (允许部分在线)
     * ════════════════════════════════════════════════════════════ */

    g_node_ctx.hal          = hal;
    g_node_ctx.shm          = shm;
    g_node_ctx.motor_count  = EXO_MOTOR_COUNT;
    g_node_ctx.startup_timeout_sec = 30;

    state_transition(STATE_DISCOVERY);  /* 阻塞: enter_discovery 做实际探测 */

    /* ════════════════════════════════════════════════════════════
     * 步骤 5: 初始化 ROS (编译可选)
     * ════════════════════════════════════════════════════════════ */

#ifdef ENABLE_ROS
    ros::init(argc, argv, "stark_periph_manager_node");
    auto nh = std::make_shared<ros::NodeHandle>();
    auto ros_adapter = Factory::CreateRosListener(nh,
                        std::shared_ptr<IMsgInternalDispatcher>(g_dispatcher,
                        [](CanDispatcher*){}));
    g_dispatcher->RegisterObserver(ListenerType::ROS, ros_adapter);
    ECO_INFO_NEW("[main] ROS adapter enabled");
#endif

    /* ════════════════════════════════════════════════════════════
     * 步骤 6: 主循环
     * ════════════════════════════════════════════════════════════ */

    ECO_INFO_NEW("[main] node ready");
    ECO_INFO_NEW("[main]   state: {}", state_name(g_exo_state));
    ECO_INFO_NEW("[main]   motors online: 0x{:02X}", shm ? shm->motor_online : 0);

    while (g_running) {
#ifdef ENABLE_ROS
        ros::spinOnce();
#endif

        /* 轮询 auto-startup (处理热插拔) */
        if (hal) {
            motor_hal_process_pending_startups(hal);
            update_shm_online(hal, shm);
        }

        /* ── 处理 RT 线程的状态切换请求 ── */
        if (g_rt_worker && shm) {
            exo_state_t pending = g_rt_worker->GetPendingState();
            if (pending == STATE_RUNNING && g_exo_state == STATE_ENABLED) {
                state_transition(STATE_RUNNING);
            }
            else if (pending == STATE_FAULT && g_exo_state != STATE_FAULT) {
                state_transition(STATE_FAULT);

                /* ★ 补发 SDO DS402 Shutdown (非 RT 路径, 可阻塞)
                 * RT 线程已通过 PDO enable=false + torque=0 完成降险,
                 * 这里走标准 DS402 状态机退出到 READY_TO_SWITCH_ON. */
                auto* ctrl = g_dispatcher->GetCtrl();
                if (ctrl) {
                    for (uint8_t id = 1; id <= EXO_MOTOR_COUNT; id++) {
                        if (shm->motor_online & (1 << (id - 1))) {
                            /* SDO 0x6040=0x06: Shutdown → READY_TO_SWITCH_ON */
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

    /* ════════════════════════════════════════════════════════════
     * 步骤 7: 清理
     * ════════════════════════════════════════════════════════════ */

    ECO_INFO_NEW("[main] shutting down...");

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
