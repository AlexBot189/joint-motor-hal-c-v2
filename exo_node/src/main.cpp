/*
 * @file main.cpp
 * @brief stark_periph_manager_node — 外骨骼电机外设节点入口
 *
 * 启动流程 (五步框架):
 *   1. Factory 创建 CanDispatcher → 初始化 CAN + 注册电机 + 启动 recv
 *   2. 创建 ExoRtWorker → 启动 RT 工作线程 (先发 torque=0)
 *   3. 主循环轮询 process_pending_startups → auto-startup
 *   4. 状态机: DISCOVERY → READY → (等校准) → ENABLED → (等算法cmd) → RUNNING
 *   5. 主循环: ros::spinOnce + 等待退出信号
 *
 * 参考: motor_tool daemon 启动流程 + petrobot periph_manager_node 架构
 */
#include <signal.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "motor_hal.h"
}

#include <log_helper/LogHelper.h>
#include "src/CanDispatcher.h"
#include "src/exo_rt_worker.h"
#include "src/exo_state_machine.h"
#include "src/Factory.h"
#include "exo_shm.h"

using namespace stark_periph_manager_node;

/* ================================================================
 * 全局状态
 * ================================================================ */

static volatile int g_running = 1;
static CanDispatcher*    g_dispatcher = nullptr;
static ExoRtWorker*      g_rt_worker  = nullptr;

/* ================================================================
 * 信号处理
 * ================================================================ */

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ================================================================
 * 等待所有电机 bootup → 调 process_pending_startups
 *
 * 主线程定期调用 motor_hal_process_pending_startups,
 * 直到所有已注册电机完成 auto-startup (state != NOT_READY)。
 * 超时 30s 仍未全部在线则返回失败。
 * ================================================================ */

static bool wait_all_motors_online(motor_hal_t* hal, exo_shm_t* shm, int timeout_sec)
{
    int elapsed_ms = 0;
    const int poll_interval_ms = 200;

    ECO_INFO("Waiting for motor auto-startup...");

    while (elapsed_ms < timeout_sec * 1000) {
        int started = motor_hal_process_pending_startups(hal);

        if (started > 0) {
            ECO_INFO("auto-startup: %d motor(s) initialized", started);
        }

        /* 检查已注册电机的状态 */
        int online_count = 0;
        int total_motors = 0;
        uint8_t online_mask = 0;

        for (uint8_t id = 1; id <= 4; ++id) {
            motor_state_t state = motor_hal_get_state(hal, id);
            if (state == MOTOR_STATE_UNKNOWN || state == MOTOR_STATE_NOT_READY) {
                continue;  /* 电机未注册或未启动 */
            }
            total_motors++;

            if (state == MOTOR_STATE_OP_ENABLED ||
                state == MOTOR_STATE_SWITCH_ON_DIS ||
                state == MOTOR_STATE_READY_TO_SW_ON) {
                online_count++;
                online_mask |= (1 << (id - 1));
            }
        }

        /* 更新 SHM 状态区 */
        if (shm) {
            shm->motor_online = online_mask;
        }

        /* 所有已注册电机在线 → 完成 */
        if (total_motors > 0 && online_count == total_motors) {
            ECO_INFO("All %d motors online (mask=0x%02X)", total_motors, online_mask);
            state_transition(STATE_DISCOVERY);
            /* DISCOVERY → READY (电机全部在线) */
            state_transition(STATE_READY);
            return true;
        }

        usleep(poll_interval_ms * 1000);
        elapsed_ms += poll_interval_ms;
    }

    ECO_ERROR("Motor startup timeout (%ds), %d motors still offline",
              timeout_sec, 0);
    return false;
}

/* ================================================================
 * main
 * ================================================================ */

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    ECO_INFO("stark_periph_manager_node starting...");

    /* ── 1. 创建 CanDispatcher (初始化 CAN + 注册电机 + 启动 recv) ── */
    g_dispatcher = new CanDispatcher();
    if (!g_dispatcher->InitDispatcher()) {
        ECO_ERROR("CanDispatcher init failed");
        delete g_dispatcher;
        return 1;
    }

    motor_hal_t* hal = g_dispatcher->GetHal();
    exo_shm_t*   shm = g_dispatcher->GetShm();

    ECO_INFO("CANFD interface ready");


    /* ── 2. 状态机: INIT → (同步初始化完成) ── */
    state_transition(STATE_INIT);


    /* ── 3. 启动 RT 工作线程 (先发 torque=0, 等算法 cmd) ── */
    g_rt_worker = new ExoRtWorker(hal, shm);
    g_rt_worker->Start();

    ECO_INFO("RT worker started (1KHz, SCHED_FIFO 90)");


    /* ── 4. 等待电机上电 → auto-startup ── */
    /* 主线程轮询 motor_hal_process_pending_startups,
     * 直到所有已注册电机进入 READY/ENABLED 状态 */
    bool all_online = wait_all_motors_online(hal, shm, 30);

    if (!all_online) {
        ECO_WARN("Not all motors online, continuing with partial set");
        /* 不退出 — 部分电机在线也可以运行 */
    }


    /* ── 5. 初始化 ROS (编译可选) ── */
#ifdef ENABLE_ROS
    ros::init(argc, argv, "stark_periph_manager_node");
    auto nh = std::make_shared<ros::NodeHandle>();
    auto ros_adapter = Factory::CreateRosListener(nh,
                        std::shared_ptr<IMsgInternalDispatcher>(g_dispatcher,
                        [](CanDispatcher*){}));  /* noop deleter */
    g_dispatcher->RegisterObserver(ListenerType::ROS, ros_adapter);
    ECO_INFO("ROS adapter enabled");
#endif


    /* ── 6. 主循环 ── */
    ECO_INFO("stark_periph_manager_node ready");
    ECO_INFO("  State: %s", state_name(g_exo_state));
    ECO_INFO("  Motors online: 0x%02X", shm ? shm->motor_online : 0);

    while (g_running) {
#ifdef ENABLE_ROS
        ros::spinOnce();
#endif

        /* 轮询 auto-startup (处理热插拔电机) */
        if (hal) {
            int started = motor_hal_process_pending_startups(hal);
            if (started > 0 && shm) {
                /* 更新 motor_online 掩码 */
                uint8_t mask = 0;
                for (uint8_t id = 1; id <= 4; ++id) {
                    motor_state_t state = motor_hal_get_state(hal, id);
                    if (state >= MOTOR_STATE_SWITCH_ON_DIS &&
                        state != MOTOR_STATE_UNKNOWN) {
                        mask |= (1 << (id - 1));
                    }
                }
                shm->motor_online = mask;
            }
        }

        /* 检查状态切换条件 */
        if (g_exo_state == STATE_ENABLED && shm) {
            /* ENABLED → RUNNING: 检测到第一个算法 cmd */
            uint64_t seq = __atomic_load_n(&shm->mailbox.seq_begin,
                                          __ATOMIC_ACQUIRE);
            if (seq > 0) {
                state_transition(STATE_RUNNING);
            }
        }

        usleep(100000);  /* 100ms 轮询间隔 */
    }


    /* ── 7. 清理 ── */
    ECO_INFO("Shutting down...");

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

    ECO_INFO("stark_periph_manager_node stopped");
    return 0;
}
