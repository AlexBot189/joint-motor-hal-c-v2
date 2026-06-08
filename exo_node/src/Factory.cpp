/*
 * @file Factory.cpp
 * @brief 对象工厂实现
 *
 * CreateSingletonDispatcher:
 *   1. new CanDispatcher
 *   2. CanDispatcher::InitDispatcher (打开 CAN / 注册电机 / 启动接收线程)
 *   3. 返回 shared_ptr<IMsgInternalDispatcher>
 *
 * CreateRosListener:
 *   1. 从 dispatcher 中获取 SHM (CanDispatcher::GetShm)
 *   2. new RosAdapter(nh, shm)
 *   3. RosAdapter::SetDispatcher (注入控制命令转发器)
 *   4. RosAdapter::Start (启动 pull 线程)
 *   5. 返回 shared_ptr<IListener>
 */
#include "Factory.h"
#include "CanDispatcher.h"
#include "ros/RosAdapter.h"
#include "exo_log.h"

namespace stark_periph_manager_node {

/* ============================================================================
 *  CreateSingletonDispatcher
 * ============================================================================ */

std::shared_ptr<IMsgInternalDispatcher>
Factory::CreateSingletonDispatcher()
{
    EXO_INFO("[Factory] 创建 CanDispatcher 单例...");

    auto dispatcher = std::make_shared<CanDispatcher>();
    if (!dispatcher) {
        EXO_ERROR("[Factory] CanDispatcher 内存分配失败");
        return nullptr;
    }

    if (!dispatcher->InitDispatcher()) {
        EXO_ERROR("[Factory] CanDispatcher::InitDispatcher 失败");
        return nullptr;
    }

    EXO_INFO("[Factory] CanDispatcher 创建并初始化完成");
    return dispatcher;
}

/* ============================================================================
 *  CreateRosListener
 * ============================================================================ */

std::shared_ptr<IListener>
Factory::CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                           std::shared_ptr<IMsgInternalDispatcher> dispatcher)
{
    EXO_INFO("[Factory] 创建 RosAdapter...");

    if (!nh) {
        EXO_ERROR("[Factory] CreateRosListener: NodeHandle 为空");
        return nullptr;
    }

    if (!dispatcher) {
        EXO_ERROR("[Factory] CreateRosListener: Dispatcher 为空");
        return nullptr;
    }

    /* ── 从 CanDispatcher 获取 SHM ── */
    auto* can = dynamic_cast<CanDispatcher*>(dispatcher.get());
    if (!can) {
        EXO_ERROR("[Factory] CreateRosListener: dispatcher 不是 CanDispatcher 类型");
        return nullptr;
    }

    exo_shm_t* shm = can->GetShm();
    if (!shm) {
        EXO_ERROR("[Factory] CreateRosListener: SHM 未初始化");
        return nullptr;
    }

    /* ── 创建 RosAdapter ── */
    auto rosAdapter = std::make_shared<RosAdapter>(nh, shm);
    if (!rosAdapter) {
        EXO_ERROR("[Factory] RosAdapter 内存分配失败");
        return nullptr;
    }

    /* ── 注入 Dispatcher (ROS Subscriber 回调中发控制命令) ── */
    rosAdapter->SetDispatcher(dispatcher);

    /* ── 启动 pull 线程 ── */
    rosAdapter->Start();

    EXO_INFO("[Factory] RosAdapter 创建并启动完成");
    return rosAdapter;
}

}  /* namespace stark_periph_manager_node */
