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
 * wait_bootup_and_startup() — 对齐 daemon _accept_loop 中的
 *   motor_hal_process_pending_startups 轮询
 *
 * 主线程轮询直到所有已注册电机 auto-startup 完成
 * (state >= SWITCH_ON_DIS 且 state != UNKNOWN).
 * ════════════════════════════════════════════════════════════════════ */

static bool wait_bootup_and_startup(motor_hal_t* hal, exo_shm_t* shm,
                                    int motor_count, int timeout_sec)
{
    int elapsed_ms = 0;
    const int interval_ms = 200;

    ECO_INFO_NEW("[main] waiting for motors to boot up ({}s timeout)...",
                 timeout_sec);

    while (elapsed_ms < timeout_sec * 1000) {
        int started = motor_hal_process_pending_startups(hal);
        if (started > 0) {
            ECO_INFO_NEW("[main] auto-startup: {} motor(s) initialized", started);
        }

        /* 用反馈缓存检测在线 (零延迟, 不触发 SDO) */
        int online_count = 0;
        uint8_t online_mask = 0;

        for (uint8_t id = 1; id <= (uint8_t)motor_count; ++id) {
            motor_feedback_t fb;
            if (motor_hal_get_feedback(hal, id, &fb) == 0 && fb.status_byte != 0) {
                online_mask |= (1 << (id - 1));
                online_count++;
            }
        }

        if (shm) {
            shm->motor_online = online_mask;
        }

        if (online_count > 0) {
            ECO_INFO_NEW("[main] {} motors online (mask=0x{:02X})",
                         online_count, online_mask);

            /* 状态转换: INIT → DISCOVERY → READY */
            state_transition(STATE_DISCOVERY);
            state_transition(STATE_READY);

            if (shm) {
                shm->node_state = STATE_READY;
            }
            return true;
        }

        usleep(interval_ms * 1000);
        elapsed_ms += interval_ms;
    }

    ECO_ERROR_NEW("[main] motor startup timeout ({}s)", timeout_sec);
    return false;
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
     * 步骤 4: 等待电机上电 → auto-startup → READY
     * ════════════════════════════════════════════════════════════ */

    bool motors_ready = wait_bootup_and_startup(hal, shm, EXO_MOTOR_COUNT, 30);

    if (!motors_ready) {
        ECO_WARN_NEW("[main] not all motors online, continuing...");
    }

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

        /* 状态切换检查 */
        if (g_exo_state == STATE_ENABLED && shm) {
            uint64_t seq = __atomic_load_n(&shm->mailbox.seq_begin,
                                          __ATOMIC_ACQUIRE);
            if (seq > 0) {
                state_transition(STATE_RUNNING);
            }
        }

        /* 同步 SHM 状态 */
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
