/*
 * @file Factory.h
 * @brief 对象工厂 — 创建 stark_periph_manager_node 核心组件
 *
 * 参考 petrobot Factory 模式, 集中管理组件创建和依赖注入。
 * ROS 相关接口仅在 ENABLE_ROS 宏定义时编译。
 */
#pragma once

#include <memory>
#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"

#ifdef ENABLE_ROS
#include <ros/ros.h>
#endif

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

#ifdef ENABLE_ROS
    /*
     * 创建 RosAdapter
     *
     * @param nh          ROS NodeHandle
     * @param dispatcher  CanDispatcher 引用 (控制命令下发路径)
     * @return            RosAdapter (IListener 接口)
     */
    static std::shared_ptr<IListener>
    CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<IMsgInternalDispatcher> dispatcher);
#endif
};

}  /* namespace stark_periph_manager_node */
