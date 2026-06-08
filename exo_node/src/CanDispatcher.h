/*
 * @file CanDispatcher.h
 * @brief 电机数据调度中心 — IMsgInternalDispatcher 实现
 *
 * CanDispatcher 实现 IMsgInternalDispatcher 接口，负责:
 *   - 电机 HAL 生命周期管理 (创建/初始化/销毁)
 *   - 共享内存 (SHM) 管理
 *   - IListener 注册/注销/通知
 *   - 接收上层 JSON 控制命令并转发到 HAL
 *
 * 线程模型:
 *   主线程 (SCHED_OTHER):  CanDispatcher 自身 + Listener 管理
 *   RT 工作线程:            ExoRtWorker 独占, 控制+上报+安全
 *   CAN 接收线程:           motor_hal 内部, SCHED_FIFO 85
 */
#pragma once

#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"
#include "interface/Defines.hpp"

#include <unordered_map>
#include <memory>
#include <string>
#include <mutex>

/* ── C 头文件包裹 ── */
extern "C" {
#include "exo_shm.h"
#include "exo_shm_mgr.h"
#include "motor_hal.h"
}

namespace stark_periph_manager_node
{

class CanDispatcher : public IMsgInternalDispatcher
{
public:
    CanDispatcher();
    ~CanDispatcher();

    /* ── IMsgInternalDispatcher 接口 ── */
    bool InitDispatcher() override;
    bool DestroyDispatcher() override;
    void Send(const std::string& data) override;
    void RegisterObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void RemoveObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void NotifyObserver(const boost::any& data) override;

    /* ── 获取 HAL / SHM 实例 (ExoRtWorker 使用) ── */
    motor_hal_t* GetHal() { return m_hal; }
    exo_shm_t*   GetShm() { return m_shm; }

    /* ── 运行状态查询 ── */
    bool IsRunning() const { return m_running; }

private:
    /* ── 内部辅助 ── */
    bool LoadMotorConfig();                         /* 读 config.json 注册电机 */

    motor_hal_t*     m_hal;                         /* HAL 实例 (PDO/SDO/CAN) */
    exo_shm_mgr_t*   m_shm_mgr;                     /* SHM 管理器 (用于清理)    */
    exo_shm_t*       m_shm;                         /* 共享内存数据指针         */
    bool             m_running;                     /* 调度器运行标志           */

    /* ── Listener 管理 ── */
    std::mutex m_listener_mutex;
    std::unordered_map<ListenerType, std::shared_ptr<IListener>> m_listeners;

    /* ── 配置路径 ── */
    std::string m_config_path;
};

}  /* namespace stark_periph_manager_node */
