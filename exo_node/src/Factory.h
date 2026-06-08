/*
 * @file Factory.h
 * @brief 对象工厂 — CanDispatcher + RosAdapter 创建/组装
 *
 * 职责:
 *   1. CreateSingletonDispatcher  → 创建单例 CanDispatcher + 初始化
 *   2. CreateRosListener           → 创建 RosAdapter + 注入 Dispatcher + 启动
 *
 * 与 petrobot 的 Factory 模式一致:
 *   main() 通过 Factory 创建组件，解耦构造逻辑与启动流程。
 */
#pragma once

#include <memory>
#include <ros/ros.h>
#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"

namespace stark_periph_manager_node {

class CanDispatcher;
class RosAdapter;

class Factory {
public:
    /* 创建单例 CanDispatcher (打开 CAN / 注册电机 / 启动 recv) */
    static std::shared_ptr<IMsgInternalDispatcher>
    CreateSingletonDispatcher();

    /* 创建 RosAdapter (需要 Dispatcher 引用用于控制下发) */
    static std::shared_ptr<IListener>
    CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<IMsgInternalDispatcher> dispatcher);
};

}  /* namespace stark_periph_manager_node */
