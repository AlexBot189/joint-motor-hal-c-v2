/*
 * @file Factory.cpp
 * @brief 对象工厂实现
 *
 * 参考 petrobot Factory::CreateSingletonDispatcher / CreateWebListener 模式。
 * stark_periph_manager_node 简化:
 *   - 不需要 DeviceOption (配置走 config.json)
 *   - 不需要 CreateTouchAdapter (外骨骼没有触摸传感器)
 */
#include "Factory.h"
#include "CanDispatcher.h"

#ifdef ENABLE_ROS
#include "ros/RosAdapter.h"
#endif

#include <cstdio>

namespace stark_periph_manager_node
{

std::shared_ptr<IMsgInternalDispatcher>
Factory::CreateSingletonDispatcher()
{
    auto dispatcher = std::make_shared<CanDispatcher>();

    if (!dispatcher->InitDispatcher()) {
        fprintf(stderr, "[Factory] CanDispatcher::InitDispatcher() failed\n");
        return nullptr;
    }

    printf("[Factory] CanDispatcher created successfully\n");
    return dispatcher;
}

#ifdef ENABLE_ROS
std::shared_ptr<IListener>
Factory::CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                           std::shared_ptr<IMsgInternalDispatcher> dispatcher)
{    if (!nh || !dispatcher) {
        fprintf(stderr, "[Factory] CreateRosListener: invalid arguments\n");
        return nullptr;
    }

    /* 从 CanDispatcher 获取 SHM 指针 */
    auto can_disp = std::dynamic_pointer_cast<CanDispatcher>(dispatcher);
    if (!can_disp) {
        fprintf(stderr, "[Factory] dispatcher is not CanDispatcher\n");
        return nullptr;
    }

    exo_shm_t* shm = can_disp->GetShm();
    if (!shm) {
        fprintf(stderr, "[Factory] SHM not available for RosAdapter\n");
        return nullptr;
    }

    auto adapter = std::make_shared<RosAdapter>(nh, shm);
    adapter->SetDispatcher(dispatcher);
    adapter->Start();

    printf("[Factory] RosAdapter created (pull from SHM)\n");
    return adapter;
}
#endif  /* ENABLE_ROS */

}  /* namespace stark_periph_manager_node */
