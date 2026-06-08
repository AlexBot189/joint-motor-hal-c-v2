/*
 * @file main.cpp
 * @brief stark_periph_manager_node — 外骨骼电机外设节点入口
 *
 * 启动流程:
 *   1. 加载配置 (exo_config.json)
 *   2. Factory 创建 CanDispatcher (打开 CAN / 注册电机 / 启动 recv)
 *   3. Factory 创建 RosAdapter (ROS 话题 + pull 线程)
 *   4. 状态机同步执行: INIT → DISCOVERY → READY
 *   5. 进入主循环: ros::spinOnce + 等待事件
 */
#include <signal.h>
#include <ros/ros.h>
#include <log_helper/LogHelper.h>
#include "src/Factory.h"
#include "src/exo_state_machine.h"
#include "src/exo_log.h"

using namespace stark_periph_manager_node;

static volatile int g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    EXO_INFO("stark_periph_manager_node starting...");

    /* 1. 创建 CanDispatcher (初始化 CAN + 注册电机 + 启动接收线程) */
    auto dispatcher = Factory::CreateSingletonDispatcher();
    if (!dispatcher) {
        EXO_ERROR("Failed to create CanDispatcher");
        return 1;
    }

    /* 2. 状态机: INIT → DISCOVERY → READY */
    state_transition(STATE_INIT);
    /* DISCOVERY 阶段由 auto-startup 在线程池中异步完成 */
    /* 主线程等待所有电机进入 READY 状态 */

    /* 3. 初始化 ROS (可选) */
#ifdef ENABLE_ROS
    ros::init(argc, argv, "stark_periph_manager_node");
    auto nh = std::make_shared<ros::NodeHandle>();
    auto rosAdapter = Factory::CreateRosListener(nh, dispatcher);
    dispatcher->RegisterObserver(ListenerType::ROS, rosAdapter);
#endif

    /* 4. 主循环 */
    EXO_INFO("stark_periph_manager_node ready");
    while (g_running) {
#ifdef ENABLE_ROS
        ros::spinOnce();
#endif
        usleep(10000);  /* 10ms 轮询, 主要是等待事件 */
    }

    /* 5. 清理 */
    dispatcher->DestroyDispatcher();
    EXO_INFO("stark_periph_manager_node stopped");
    return 0;
}
