/*
 * CanDispatcher.cpp — 电机数据调度中心实现
 *
 * 对齐 motor_tool daemon: fork→recv→SHM→主循环
 * 所有 SDO/PDO/OD 通过 ExoMotorCtrl 封装.
 */
#include "CanDispatcher.h"
#include "nlohmann/json.hpp"

#include <cstring>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <log_helper/LogHelper.h>

namespace stark_periph_manager_node {

#include "exo_shm_mgr.h"
extern "C" {
exo_shm_mgr_t* exo_shm_mgr_open(const char* name, bool create, size_t size);
void            exo_shm_mgr_close(exo_shm_mgr_t* mgr);
}

CanDispatcher::CanDispatcher()
    : m_hal(nullptr), m_shm(nullptr), m_running(false)
    , m_config_path("exo_node/config/exo_config.json")
{
}

CanDispatcher::~CanDispatcher()
{
    if (m_running) DestroyDispatcher();
}

/* ════════════════════════════════════════════════════════════════════
 * InitDispatcher() — 对齐 daemon_start() 流程
 * ════════════════════════════════════════════════════════════════════ */

bool CanDispatcher::InitDispatcher()
{
    if (m_running) return false;

    /* 1. 创建 HAL */
    m_hal = motor_hal_create();
    if (!m_hal) {
        ECO_ERROR_NEW("[CanDispatcher] motor_hal_create() failed");
        return false;
    }

    /* 2. 打开 CANFD — 对齐 tool_init */
    int ret = motor_hal_init(m_hal, "can0", 1000000, 5000000);
    if (ret < 0) {
        ECO_ERROR_NEW("[CanDispatcher] motor_hal_init() failed: {}", ret);
        motor_hal_destroy(m_hal); m_hal = nullptr;
        return false;
    }
    ECO_INFO_NEW("[CanDispatcher] CANFD can0: arb=1M data=5M ✓");

    /* 3. 注册电机 (走配置或硬编码默认) */
    if (!LoadMotorConfig()) {
        ECO_ERROR_NEW("[CanDispatcher] LoadMotorConfig() failed");
        motor_hal_destroy(m_hal); m_hal = nullptr;
        return false;
    }

    /* 4. 后台化 ★ 必须在 recv_start 之前 fork */
    /* (exo_node 不 fork, 但流程顺序对齐 daemon) */

    /* 5. 启动接收线程 */
    ret = motor_hal_recv_start(m_hal);
    if (ret < 0) {
        ECO_ERROR_NEW("[CanDispatcher] motor_hal_recv_start() failed: {}", ret);
        motor_hal_destroy(m_hal); m_hal = nullptr;
        return false;
    }

    /* 6. 创建 ExoMotorCtrl 封装 */
    m_ctrl = std::make_unique<ExoMotorCtrl>(m_hal);

    /* 7. 打开共享内存 */
    m_shm_mgr = exo_shm_mgr_open(EXO_SHM_NAME, true, EXO_SHM_SIZE);
    if (!m_shm_mgr) {
        ECO_ERROR_NEW("[CanDispatcher] exo_shm_mgr_open() failed");
        motor_hal_recv_stop(m_hal);
        motor_hal_destroy(m_hal); m_hal = nullptr;
        return false;
    }
    m_shm = (exo_shm_t*)m_shm_mgr->ptr;

    /* 初始化 SHM */
    memset(m_shm, 0, EXO_SHM_SIZE);

    m_running = true;
    ECO_INFO_NEW("[CanDispatcher] ready");
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 * DestroyDispatcher()
 * ════════════════════════════════════════════════════════════════════ */

bool CanDispatcher::DestroyDispatcher()
{
    if (!m_running) return true;
    m_running = false;

    /* 1. NMT 广播 Stop */
    if (m_hal) {
        motor_hal_nmt_broadcast(m_hal, NMT_CMD_STOP);
    }

    /* 2. 停止 recv + 销毁 HAL */
    if (m_hal) {
        motor_hal_recv_stop(m_hal);
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
    }
    m_ctrl.reset();

    /* 3. 关闭 SHM */
    if (m_shm) {
        exo_shm_mgr_close(m_shm_mgr);
        m_shm_mgr = nullptr;
        m_shm = nullptr;
    }

    ECO_INFO_NEW("[CanDispatcher] stopped");
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 * Send() — JSON 控制命令 → ExoMotorCtrl
 *
 * 格式:
 *   {"cmd":"torque","motor_id":1,"value":500}
 *   {"cmd":"speed","motor_id":2,"value":3000}
 *   {"cmd":"position","motor_id":1,"value":9000}
 *   {"cmd":"enable","motor_id":1}
 *   {"cmd":"disable","motor_id":1}
 *   {"cmd":"stop","motor_id":0}
 *   {"cmd":"setzero","motor_id":1}
 *   {"cmd":"pid","motor_id":1,"cp":100,"ci":10,...}
 *   {"cmd":"save","motor_id":1}
 *   {"cmd":"fault_reset","motor_id":1}
 *
 * 非实时, SDO 路径.
 * ════════════════════════════════════════════════════════════════════ */

void CanDispatcher::Send(const std::string& data)
{
    if (!m_hal || !m_running || !m_ctrl) return;

    try {
        auto j = nlohmann::json::parse(data);

        std::string cmd = j.value("cmd", std::string{});
        int motor_id    = j.value("motor_id", 0);
        int value       = j.value("value", 0);

        if (cmd.empty()) {
            ECO_WARN_NEW("[CanDispatcher] Send: missing 'cmd'");
            return;
        }

        _dispatch_command(cmd, (uint8_t)motor_id, value);

    } catch (const nlohmann::json::parse_error& e) {
        ECO_ERROR_NEW("[CanDispatcher] JSON parse: {}", e.what());
    } catch (const std::exception& e) {
        ECO_ERROR_NEW("[CanDispatcher] Send exception: {}", e.what());
    }
}

void CanDispatcher::_dispatch_command(const std::string& cmd, uint8_t id, int val)
{
    if (cmd == "torque") {
        m_ctrl->Torque(id, val);
    }
    else if (cmd == "speed") {
        m_ctrl->Speed(id, val);
    }
    else if (cmd == "position") {
        m_ctrl->AbsPosition(id, val);
    }
    else if (cmd == "stop") {
        if (id == 0) {
            for (uint8_t i = 1; i <= EXO_MOTOR_COUNT; i++) m_ctrl->AbsStop(i);
        } else {
            m_ctrl->AbsStop(id);
        }
    }
    else if (cmd == "enable") {
        m_ctrl->Enable(id);
    }
    else if (cmd == "disable") {
        m_ctrl->Disable(id);
    }
    else if (cmd == "fault_reset") {
        m_ctrl->FaultReset(id);
    }
    else if (cmd == "setzero") {
        m_ctrl->SetZero(id);
    }
    else if (cmd == "save") {
        m_ctrl->SaveFlash(id);
    }
    else if (cmd == "reboot") {
        m_ctrl->Reboot(id);
    }
    else if (cmd == "pid") {
        /* 简易版: 需从 JSON 额外解析 PID 参数 */
        ECO_WARN_NEW("[CanDispatcher] pid command needs full JSON fields, use exo_motor_ctrl API directly");
    }
    else {
        ECO_WARN_NEW("[CanDispatcher] unknown cmd: {}", cmd);
    }
}

/* ════════════════════════════════════════════════════════════════════
 * Observer
 * ════════════════════════════════════════════════════════════════════ */

void CanDispatcher::RegisterObserver(ListenerType type,
                                     std::shared_ptr<IListener> listener)
{
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    m_listeners[type] = listener;
}

void CanDispatcher::RemoveObserver(ListenerType type,
                                   std::shared_ptr<IListener> listener)
{
    (void)listener;
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    m_listeners.erase(type);
}

void CanDispatcher::NotifyObserver(const boost::any& data)
{
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    for (auto& kv : m_listeners) {
        if (kv.second) kv.second->Update(data);
    }
}

/* ════════════════════════════════════════════════════════════════════
 * LoadMotorConfig() — 对齐 daemon 硬编码默认值
 * ════════════════════════════════════════════════════════════════════ */

bool CanDispatcher::LoadMotorConfig()
{
    std::ifstream ifs(m_config_path);
    if (ifs.is_open()) {
        nlohmann::json cfg;
        try { ifs >> cfg; }
        catch (const nlohmann::json::parse_error& e) {
            ECO_ERROR_NEW("[CanDispatcher] config parse: {}", e.what());
            return false;
        }
        ifs.close();

        if (cfg.contains("motors") && cfg["motors"].is_array()) {
            for (const auto& m : cfg["motors"]) {
                motor_config_t mc = {};
                mc.node_id           = m.value("id", 0);
                mc.heartbeat_ms      = m.value("heartbeat_ms", 2000);
                mc.profile_accel     = m.value("profile_accel", 5000u);
                mc.profile_decel     = m.value("profile_decel", 5000u);
                mc.profile_velocity  = m.value("profile_velocity", 20u);
                mc.disable_watchdog  = m.value("disable_watchdog", true);
                mc.auto_enable       = m.value("auto_enable", true);
                mc.bootup_timeout_ms = m.value("bootup_timeout_ms", 3000);
                mc.tpdo_sync_count   = m.value("tpdo_sync_count", (uint8_t)1);

                if (mc.node_id == 0) continue;
                int ret = motor_hal_add_motor(m_hal, &mc);
                if (ret < 0) {
                    ECO_ERROR_NEW("[CanDispatcher] motor id={} add failed: {}", mc.node_id, ret);
                    return false;
                }
            }
            ECO_INFO_NEW("[CanDispatcher] loaded {} motors from {}",
                         cfg["motors"].size(), m_config_path);
            return true;
        }
    }

    /* 配置文件不存在 → 硬编码默认值 (对齐 daemon) */
    ECO_INFO_NEW("[CanDispatcher] config not found, using hardcoded defaults");

    motor_config_t def = {};
    def.heartbeat_ms      = 2000;
    def.profile_accel     = 5000;
    def.profile_decel     = 5000;
    def.profile_velocity  = 20;
    def.disable_watchdog  = true;
    def.auto_enable       = true;
    def.bootup_timeout_ms = 5000;
    def.tpdo_sync_count   = 1;

    for (uint8_t id = 1; id <= EXO_MOTOR_COUNT; id++) {
        def.node_id = id;
        int ret = motor_hal_add_motor(m_hal, &def);
        if (ret != 0 && ret != -EEXIST) {
            ECO_ERROR_NEW("[CanDispatcher] motor id={} add failed: {}", id, ret);
            return false;
        }
    }

    ECO_INFO_NEW("[CanDispatcher] registered {} motors (defaults)", EXO_MOTOR_COUNT);
    return true;
}

}  /* namespace stark_periph_manager_node */
