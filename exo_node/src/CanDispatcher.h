/*
 * CanDispatcher.h — 电机数据调度中心
 *
 * 对齐 motor_tool daemon 的初始化流程:
 *   1. motor_hal_create + init (CANFD)
 *   2. 注册电机 (ID 1,2)
 *   3. fork 后台化 (★ 在 recv_start 之前)
 *   4. recv_start (子进程)
 *   5. 创建 SHM
 *   6. 主循环 (事件驱动)
 *
 * 所有 SDO/PDO/OD 控制通过 ExoMotorCtrl 封装.
 * RT 控制走 ExoRtWorker → SHM mailbox.
 */
#pragma once

#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"
#include "interface/Defines.hpp"

#include <unordered_map>
#include <memory>
#include <string>
#include <mutex>

extern "C" {
#include "exo_shm.h"
#include "exo_shm_mgr.h"
#include "motor_hal.h"
}

#include "core/exo_motor_ctrl.h"

namespace stark_periph_manager_node {

class CanDispatcher : public IMsgInternalDispatcher {
public:
    CanDispatcher();
    ~CanDispatcher();

    /* ── IMsgInternalDispatcher ── */
    bool InitDispatcher() override;
    bool DestroyDispatcher() override;
    void Send(const std::string& data) override;
    void RegisterObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void RemoveObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void NotifyObserver(const boost::any& data) override;

    /* ── 获取内部实例 ── */
    motor_hal_t*   GetHal()  { return m_hal; }
    exo_shm_t*     GetShm()  { return m_shm; }
    ExoMotorCtrl*  GetCtrl() { return m_ctrl.get(); }

    bool IsRunning() const { return m_running; }
    void SetConfigPath(const std::string& path) { m_config_path = path; }

private:
    bool LoadMotorConfig();
    void _dispatch_command(const std::string& cmd, uint8_t id, int value);

    motor_hal_t*    m_hal;
    exo_shm_mgr_t*  m_shm_mgr;
    exo_shm_t*      m_shm;
    bool            m_running;

    std::unique_ptr<ExoMotorCtrl> m_ctrl;

    std::mutex m_listener_mutex;
    std::unordered_map<ListenerType, std::shared_ptr<IListener>> m_listeners;
    std::string m_config_path;
};

}  /* namespace stark_periph_manager_node */
