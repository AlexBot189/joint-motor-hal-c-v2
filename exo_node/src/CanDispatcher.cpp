/*
 * @file CanDispatcher.cpp
 * @brief 电机数据调度中心 — 实现
 *
 * 负责:
 *   1. 初始化 motor_hal, 读取配置注册电机, 启动 CAN 接收
 *   2. 打开共享内存 (SHM), 初始化状态区
 *   3. 接收上层 JSON 控制命令 (Send), 转发到 HAL
 *   4. 管理 IListener 观察者 (Register/Remove/Notify)
 *
 * 非 RT 路径: Send() 走 SDO, 可阻塞
 * RT 路径:    ExoRtWorker 直接操作 m_hal/m_shm, 不经过 CanDispatcher
 */

#include "CanDispatcher.h"
#include "nlohmann/json.hpp"

#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace stark_periph_manager_node
{

/* ════════════════════════════════════════════════════════════════════
 * extern — exo_shm_mgr.cpp 提供的函数
 * ════════════════════════════════════════════════════════════════════ */
#include "exo_shm_mgr.h"

extern "C" {
exo_shm_mgr_t* exo_shm_mgr_open(const char* name, bool create, size_t size);
void            exo_shm_mgr_close(exo_shm_mgr_t* mgr);
}

/* ════════════════════════════════════════════════════════════════════
 * CanDispatcher
 * ════════════════════════════════════════════════════════════════════ */

CanDispatcher::CanDispatcher()
    : m_hal(nullptr)
    , m_shm(nullptr)
    , m_running(false)
    , m_config_path("exo_node/config/exo_config.json")
{
}

CanDispatcher::~CanDispatcher()
{
    if (m_running) {
        DestroyDispatcher();
    }
}

/* ────────────────────────────────────────────────────────────────────
 * InitDispatcher()
 *
 * 步骤:
 *   1. 创建 HAL 实例
 *   2. 打开 CANFD 接口
 *   3. 从 config.json 加载电机列表并注册
 *   4. 启动 CAN 接收线程
 *   5. 打开共享内存, 初始化状态区
 *
 * 返回: true=成功发起, false=失败
 * ──────────────────────────────────────────────────────────────────── */
bool CanDispatcher::InitDispatcher()
{
    if (m_running) {
        return false;   /* 已初始化 */
    }

    /* 1. 创建 HAL */
    m_hal = motor_hal_create();
    if (!m_hal) {
        fprintf(stderr, "[CanDispatcher] motor_hal_create() failed\n");
        return false;
    }

    /* 2. 打开 CANFD 接口 */
    int ret = motor_hal_init(m_hal, "can0", 1000000, 5000000);
    if (ret < 0) {
        fprintf(stderr, "[CanDispatcher] motor_hal_init() failed: %d\n", ret);
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }

    /* 3. 从配置文件加载电机 */
    if (!LoadMotorConfig()) {
        fprintf(stderr, "[CanDispatcher] LoadMotorConfig() failed\n");
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }

    /* 4. 启动 CAN 接收线程 (必须在 startup 之前) */
    ret = motor_hal_recv_start(m_hal);
    if (ret < 0) {
        fprintf(stderr, "[CanDispatcher] motor_hal_recv_start() failed: %d\n", ret);
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }

    /* 5. 打开共享内存 */
    m_shm_mgr = exo_shm_mgr_open(EXO_SHM_NAME, true, EXO_SHM_SIZE);
    if (!m_shm_mgr) {
        fprintf(stderr, "[CanDispatcher] exo_shm_mgr_open() failed\n");
        motor_hal_recv_stop(m_hal);
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }
    m_shm = (exo_shm_t*)m_shm_mgr->ptr;

    /* 初始化 SHM 状态区为 0 */
    memset(&m_shm->fb_buffer,    0, sizeof(m_shm->fb_buffer));
    m_shm->active_idx     = 0;
    m_shm->motor_online   = 0;
    m_shm->calib_state    = 0;
    m_shm->motor_enabled  = 0;
    m_shm->motor_severity = 0;
    m_shm->fault_reason   = 0;
    m_shm->node_state     = STATE_INIT;
    memset(&m_shm->mailbox, 0, sizeof(m_shm->mailbox));

    m_running = true;
    return true;
}

/* ────────────────────────────────────────────────────────────────────
 * DestroyDispatcher()
 *
 * 逆序清理:
 *   1. 停止 RT 工作线程 (ExoRtWorker 先 Stop)
 *   2. 停止 CAN 接收线程
 *   3. 销毁 HAL (自动脱使能)
 *   4. 关闭 SHM
 * ──────────────────────────────────────────────────────────────────── */
bool CanDispatcher::DestroyDispatcher()
{
    if (!m_running) {
        return true;
    }

    m_running = false;

    /* 1. 停止 CAN 接收 (HAL 内部线程) */
    if (m_hal) {
        motor_hal_recv_stop(m_hal);
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
    }

    /* 2. 关闭 SHM */
    if (m_shm) {
        exo_shm_mgr_close(m_shm_mgr);
        m_shm_mgr = nullptr;
        m_shm = nullptr;
    }

    return true;
}

/* ────────────────────────────────────────────────────────────────────
 * Send(const string& data)
 *
 * 解析 JSON 控制命令字符串, 转发到 HAL:
 *
 *   { "cmd": "torque",  "motor_id": 1, "value": 500 }
 *   { "cmd": "speed",   "motor_id": 2, "value": 3000 }
 *   { "cmd": "position","motor_id": 1, "value": 9000 }
 *   { "cmd": "stop",    "motor_id": 0 }              → 所有电机
 *
 * 非实时路径, 可以阻塞 (内部走 SDO)。
 * RT 控制走 SHM mailbox → ExoRtWorker → motor_hal_multi_ctrl。
 * ──────────────────────────────────────────────────────────────────── */
void CanDispatcher::Send(const std::string& data)
{
    if (!m_hal || !m_running) {
        return;
    }

    try {
        auto j = nlohmann::json::parse(data);

        std::string cmd  = j.value("cmd", std::string{});
        int  motor_id    = j.value("motor_id", 0);
        int  value       = j.value("value", 0);

        if (cmd.empty()) {
            fprintf(stderr, "[CanDispatcher] Send: missing 'cmd' field\n");
            return;
        }

        /* 双电机模式: motor_id=0 → 遍历所有在线电机 */
        if (motor_id == 0) {
            uint8_t online = (m_shm) ? m_shm->motor_online : 0;
            for (uint8_t id = 1; id <= 4; ++id) {
                if (online & (1 << (id - 1))) {
                    /* 递归调用简化 —— 分别发到每个电机 */
                }
            }
        }

        if (cmd == "torque") {
            /* 电流模式 */
            motor_hal_set_mode(m_hal, (uint8_t)motor_id, MOTOR_MODE_CURRENT);
            motor_hal_sdo_write(m_hal, (uint8_t)motor_id,
                                OD_TARGET_TORQUE, 0x00,
                                (uint32_t)(int32_t)value, 2);
        }
        else if (cmd == "speed") {
            /* 速度模式 — value 单位 RPM×100 */
            motor_hal_set_mode(m_hal, (uint8_t)motor_id, MOTOR_MODE_PROFILE_VEL);
            motor_hal_sdo_write(m_hal, (uint8_t)motor_id,
                                OD_TARGET_VELOCITY, 0x00,
                                (uint32_t)value, 4);
        }
        else if (cmd == "position") {
            /* 位置模式 — value 单位 °×100 */
            float angle = value / 100.0f;
            motor_hal_set_position(m_hal, (uint8_t)motor_id, angle);
        }
        else if (cmd == "stop") {
            motor_hal_stop(m_hal, (uint8_t)motor_id);
        }
        else if (cmd == "enable") {
            motor_hal_enable(m_hal, (uint8_t)motor_id);
        }
        else if (cmd == "disable") {
            motor_hal_disable(m_hal, (uint8_t)motor_id);
        }
        else if (cmd == "mode") {
            /* 切换控制模式: { "cmd":"mode", "motor_id":1, "value":1 }
             * value = motor_mode_t 枚举值 */
            motor_hal_set_mode(m_hal, (uint8_t)motor_id, (motor_mode_t)value);
        }
        else if (cmd == "fault_reset") {
            motor_hal_fault_reset(m_hal, (uint8_t)motor_id);
        }
        else {
            fprintf(stderr, "[CanDispatcher] Send: unknown cmd '%s'\n",
                    cmd.c_str());
        }

    } catch (const nlohmann::json::parse_error& e) {
        fprintf(stderr, "[CanDispatcher] JSON parse error: %s\n", e.what());
    } catch (const std::exception& e) {
        fprintf(stderr, "[CanDispatcher] Send exception: %s\n", e.what());
    }
}

/* ────────────────────────────────────────────────────────────────────
 * RegisterObserver
 * ──────────────────────────────────────────────────────────────────── */
void CanDispatcher::RegisterObserver(ListenerType type,
                                     std::shared_ptr<IListener> listener)
{
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    m_listeners[type] = listener;
}

/* ────────────────────────────────────────────────────────────────────
 * RemoveObserver
 * ──────────────────────────────────────────────────────────────────── */
void CanDispatcher::RemoveObserver(ListenerType type,
                                   std::shared_ptr<IListener> listener)
{
    (void)listener;  /* 用 type 索引, listener 参数保留接口兼容 */
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    m_listeners.erase(type);
}

/* ────────────────────────────────────────────────────────────────────
 * NotifyObserver
 *
 * 遍历所有注册的 Listener, 调用 Update(data)。
 * 数据源: SHM 反馈帧 或 状态变更事件。
 * 注意: petrobot 用 push 模式 (recv 线程直接推), 
 *       exo_node 改为 pull (RosAdapter/WebServer 独立线程读 SHM),
 *       此函数保留用于状态变更通知 (state transition / fault)。
 * ──────────────────────────────────────────────────────────────────── */
void CanDispatcher::NotifyObserver(const boost::any& data)
{
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    for (auto& kv : m_listeners) {
        if (kv.second) {
            kv.second->Update(data);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════
 * 内部辅助
 * ════════════════════════════════════════════════════════════════════ */

/*
 * LoadMotorConfig()
 *
 * 从 config/exo_config.json 读取电机列表,
 * 填充 motor_config_t 并调用 motor_hal_add_motor 注册。
 *
 * 配置路径: m_config_path (默认 "exo_node/config/exo_config.json")
 */
bool CanDispatcher::LoadMotorConfig()
{
    std::ifstream ifs(m_config_path);
    if (!ifs.is_open()) {
        fprintf(stderr, "[CanDispatcher] Cannot open config: %s\n",
                m_config_path.c_str());
        return false;
    }

    nlohmann::json cfg;
    try {
        ifs >> cfg;
    } catch (const nlohmann::json::parse_error& e) {
        fprintf(stderr, "[CanDispatcher] Config JSON parse error: %s\n",
                e.what());
        return false;
    }

    /* 读取电机列表 */
    if (!cfg.contains("motors") || !cfg["motors"].is_array()) {
        fprintf(stderr, "[CanDispatcher] Config: missing 'motors' array\n");
        return false;
    }

    for (const auto& m : cfg["motors"]) {
        motor_config_t motor_cfg = {};
        motor_cfg.node_id           = m.value("id",           0);
        motor_cfg.heartbeat_ms      = m.value("heartbeat_ms", 2000);
        motor_cfg.profile_accel     = m.value("profile_accel", 5000u);
        motor_cfg.profile_decel     = m.value("profile_decel", 5000u);
        motor_cfg.profile_velocity  = m.value("profile_velocity", 20u);
        motor_cfg.disable_watchdog  = m.value("disable_watchdog",  true);
        motor_cfg.auto_enable       = m.value("auto_enable",       true);
        motor_cfg.bootup_timeout_ms = m.value("bootup_timeout_ms", 3000);
        motor_cfg.tpdo_sync_count   = m.value("tpdo_sync_count",   (uint8_t)1);
        motor_cfg.pos_limit_pos     = m.value("pos_limit_pos", 180.0f);
        motor_cfg.pos_limit_neg     = m.value("pos_limit_neg", -180.0f);

        if (motor_cfg.node_id == 0) {
            fprintf(stderr, "[CanDispatcher] Invalid motor id=0, skip\n");
            continue;
        }

        int ret = motor_hal_add_motor(m_hal, &motor_cfg);
        if (ret < 0) {
            fprintf(stderr, "[CanDispatcher] add_motor(id=%d) failed: %d\n",
                    motor_cfg.node_id, ret);
            return false;
        }

        /* 读取 SHM 安全参数 (仅首次有效) */
        if (cfg.contains("safety")) {
            /* 参数由 ExoRtWorker 在构造时读取, 此处仅记录可用 */
        }
    }

    printf("[CanDispatcher] Loaded %zu motors from config\n",
           cfg["motors"].size());
    return true;
}

}  /* namespace stark_periph_manager_node */
