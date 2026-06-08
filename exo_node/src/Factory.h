/*
 * @file Factory.h
 * @brief 对象工厂 — 创建 stark_periph_manager_node 核心组件
 *
 * 参考 petrobot Factory 模式, 集中管理组件创建和依赖注入:
 *   - CanDispatcher (HAL + SHM 管理)
 *   - RosAdapter   (ROS Topic/Service)
 *   - WebServer    (WebSocket, 预留)
 */
#pragma once

#include <memory>
#include <ros/ros.h>
#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"

namespace stark_periph_manager_node
{

class CanDispatcher;

class Factory
{
public:
    /*
     * 创建 CanDispatcher 单例
     *
     * 内部完成:
     *   - motor_hal_create + motor_hal_init (CANFD)
     *   - 从 config.json 读取电机列表并注册
     *   - motor_hal_recv_start (CAN 接收线程)
     *   - SHM 创建 + 初始化
     */
    static std::shared_ptr<IMsgInternalDispatcher>
    CreateSingletonDispatcher();

    /*
     * 创建 RosAdapter
     *
     * @param nh          ROS NodeHandle
     * @param dispatcher  CanDispatcher 引用 (控制命令下发路径)
     * @return            RosAdapter (IListener 接口)
     *
     * 内部: 从 dispatcher 获取 SHM 指针 → RosAdapter 构造 → RosAdapter->Start()
     */
    static std::shared_ptr<IListener>
    CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<IMsgInternalDispatcher> dispatcher);
};

}  /* namespace stark_periph_manager_node */
