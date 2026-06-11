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
    int ret = motor_hal_init(m_hal, m_can_iface.c_str(),
                              m_can_arb_rate, m_can_data_rate);
    if (ret < 0) {
        ECO_ERROR_NEW("[CanDispatcher] motor_hal_init({}) failed: {}", m_can_iface, ret);
        motor_hal_destroy(m_hal); m_hal = nullptr;
        return false;
    }
    ECO_INFO_NEW("[CanDispatcher] CANFD {}: arb={}bps data={}bps ✓",
                 m_can_iface, m_can_arb_rate, m_can_data_rate);

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
    m_shm_mgr = exo_shm_mgr_open(m_shm_name.c_str(), true, m_shm_size_bytes);
    if (!m_shm_mgr) {
        ECO_ERROR_NEW("[CanDispatcher] exo_shm_mgr_open() failed");
        motor_hal_recv_stop(m_hal);
        motor_hal_destroy(m_hal); m_hal = nullptr;
        return false;
    }
    m_shm = (exo_shm_t*)m_shm_mgr->ptr;

    /* 全量清零 SHM — 防止跨进程残留触发假 FAULT
     * 启动顺序: exo_node 先启 → algo_sim 后启, 不存在 algo 正在写的情况
     * 即使 algo 在跑, 清零后 algo 检测到 seq 回零会重新握手, 安全 */
    memset(m_shm, 0, EXO_SHM_SIZE);
    m_shm->node_state     = STATE_INIT;

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

    /* 1. PDO 安全停机: 先设 pdo_byte0=estop, 再发 PDO 帧让电机停止
     *    (替代 NMT_STOP — 避免驱动板进入不可恢复的 Stopped 状态) */
    if (m_hal) {
        for (uint8_t id = 1; id <= EXO_MOTOR_COUNT; id++) {
            motor_hal_pdo_estop(m_hal, id);        // pdo_byte0 enable=0, bus=0
            motor_hal_set_torque(m_hal, id, 0);     // 发 PDO 帧: enable=0, torque=0
        }
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

        /* ── 解析 CAN ── */
        if (cfg.contains("can")) {
            m_can_iface    = cfg["can"].value("interface",        "can0");
            m_can_arb_rate = cfg["can"].value("arbitration_rate", 1000000);
            m_can_data_rate= cfg["can"].value("data_rate",        5000000);
        }

        /* ── 解析 motors ── */
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
                mc.bootup_timeout_ms = 5000;
                mc.tpdo_sync_count   = m.value("tpdo_sync_count", (uint8_t)1);

                if (mc.node_id == 0) continue;

                std::string name = m.value("name", std::string{});
                ECO_INFO_NEW("[CanDispatcher] motor id={} name='{}'",
                             mc.node_id, name.empty() ? "(none)" : name);

                int ret = motor_hal_add_motor(m_hal, &mc);
                if (ret < 0) {
                    ECO_ERROR_NEW("[CanDispatcher] motor id={} add failed: {}", mc.node_id, ret);
                    return false;
                }
            }
            ECO_INFO_NEW("[CanDispatcher] loaded {} motors from {}",
                         cfg["motors"].size(), m_config_path);
        }

        /* ── 解析 safety ── */
        if (cfg.contains("safety")) {
            auto& s = cfg["safety"];
            m_safety_cfg.algo_timeout_ms   = s.value("algo_timeout_ms",   200u);
            m_safety_cfg.algo_shutdown_ms  = s.value("algo_shutdown_ms",  500u);
            m_safety_cfg.overtemp_celsius  = s.value("overtemp_celsius",  80);
            m_safety_cfg.can_offline_ms    = s.value("can_offline_ms",    2000u);
            m_safety_cfg.encoder_stall_s   = s.value("encoder_stall_s",   3u);
        }

        /* ── 解析 rt ── */
        if (cfg.contains("rt")) {
            auto& r = cfg["rt"];
            m_rt_cfg.priority      = r.value("control_priority",  90);
            m_rt_cfg.period_us     = r.value("control_period_us", 1000u);
            m_rt_cfg.report_divider = r.value("report_divider",    5);
            if (r.contains("cpu_affinity") && r["cpu_affinity"].is_array()
                && r["cpu_affinity"].size() > 0) {
                m_rt_cfg.cpu_affinity[0] = r["cpu_affinity"][0].get<int>();
                m_rt_cfg.cpu_affinity[1] = (r["cpu_affinity"].size() > 1)
                    ? r["cpu_affinity"][1].get<int>() : -1;
            }
        }

        /* ── 解析 shm ── */
        if (cfg.contains("shm")) {
            m_shm_name = cfg["shm"].value("name", std::string(EXO_SHM_NAME));
            size_t kb  = cfg["shm"].value("size_kb", (size_t)(EXO_SHM_SIZE / 1024));
            m_shm_size_bytes = kb * 1024;
            if (m_shm_size_bytes < sizeof(exo_shm_t)) {
                ECO_WARN_NEW("[CanDispatcher] shm.size_kb={} < struct size={}, clamping",
                             kb, sizeof(exo_shm_t));
                m_shm_size_bytes = sizeof(exo_shm_t);
            }
        }

        return true;  /* 文件解析完成, 缺失字段用默认值 */
    }

    /* 配置文件不存在 → 全部默认值 (对齐 daemon) */
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
