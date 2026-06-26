/*
 * @file WebServer.cpp
 * @brief WebSocket 调试服务器实现
 *
 * ## 当前状态: 占位实现
 *
 *   crow WebSocket 库未引入项目, 当前 PullLoop 仅从 SHM 读取反馈帧并计数。
 *   每 100 帧打印统计信息, 验证数据通路。crow 完整实现见下方注释代码框架。
 *
 * ## 启用 crow 完整实现的步骤
 *
 *   1. 将 crow_all.h 放入 exo_node/3rd/crow/ 或系统 include 路径
 *   2. 在 CMakeLists.txt 取消注释 crow 相关行
 *   3. 去掉本文件 #define WEBSERVER_PLACEHOLDER 宏
 */

#include "web/WebServer.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <unistd.h>

/* ── 编译开关: 定义此宏使用占位实现, 注释掉则使用完整 crow 实现 ── */
#define WEBSERVER_PLACEHOLDER

namespace stark_periph_manager_node {

/* ════════════════════════════════════════════════════════════════════
 * 构造 / 析构
 * ════════════════════════════════════════════════════════════════════ */

WebServer::WebServer(exo_shm_t* shm, uint16_t port)
    : m_shm(shm)
    , m_port(port)
    , m_running(false)
    , m_frame_count(0)
    , m_fail_count(0)
{
}

WebServer::~WebServer()
{
    if (m_running.load(std::memory_order_acquire)) {
        Stop();
    }
}

/* ════════════════════════════════════════════════════════════════════
 * Update() — IListener 接口 (预留)
 * ════════════════════════════════════════════════════════════════════ */

void WebServer::Update(const boost::any& data)
{
    /* 预留: 未来可用于接收 CanDispatcher 推送的增量数据 */
    (void)data;
}

/* ════════════════════════════════════════════════════════════════════
 * Start() / Stop()
 * ════════════════════════════════════════════════════════════════════ */

void WebServer::Start()
{
    if (m_running.load(std::memory_order_acquire)) {
        return;
    }

    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&WebServer::PullLoop, this);

    printf("[WebServer] Started on port %u (placeholder mode)\n", m_port);
}

void WebServer::Stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    printf("[WebServer] Stopped. Total frames: %lu, failures: %lu\n",
           m_frame_count, m_fail_count);
}

/* ════════════════════════════════════════════════════════════════════
 * PullLoop() — 200Hz 读 SHM → JSON → WebSocket 推送
 *
 * ## 占位实现
 *
 *   当前不依赖任何第三方 WebSocket 库。
 *   - 读 SHM fb_buffer[active] 快照
 *   - 计数, 每 100 帧打印进度
 *   - usleep(5000) 维持 200Hz
 *
 * ## 完整 crow 实现代码框架 (注释)
 *   取消下方 WEBSERVER_PLACEHOLDER 宏后启用。
 * ════════════════════════════════════════════════════════════════════ */

#ifdef WEBSERVER_PLACEHOLDER

void WebServer::PullLoop()
{
    uint64_t local_count = 0;

    while (m_running.load(std::memory_order_acquire)) {

        /* ── 1. 从 SHM 双 Buffer 读反馈帧快照 ── */
        if (m_shm) {
            uint32_t active = __atomic_load_n(&m_shm->active_idx,
                                              __ATOMIC_ACQUIRE);
            /* 值拷贝 fb_buffer[active], 防止 RT 线程切换期间数据撕裂 */
            feedback_frame_t snapshot = m_shm->fb_buffer[active];
            (void)snapshot;  /* 占位: 未序列化 */

            /* ── 2. (TODO) JSON 序列化 ── */
            /* 参考: 下方完整实现中的 SerializeFrame() */

            /* ── 3. (TODO) push 到所有 WebSocket 客户端 ── */
            /* 参考: 下方完整实现中的 BroadcastFrame() */
        }

        local_count++;
        m_frame_count++;

        /* 每 100 帧输出进度 */
        if (local_count % 100 == 0) {
            printf("[WebServer] sent %lu frames (approx %.1f s)\n",
                   m_frame_count, (double)m_frame_count / 200.0);
        }

        /* 200Hz 周期: 5ms */
        usleep(5000);
    }
}

#else  /* ── 完整 crow WebSocket 实现 (需 crow_all.h) ── */

/*
 * ════════════════════════════════════════════════════════════════════
 * 完整实现: 基于 crow 的 WebSocket 实时数据推送服务器
 *
 * 使用方式:
 *   1. 编译: cmake -DENABLE_WEBSERVER=ON ..
 *   2. 运行后浏览器访问 http://<ip>:8080
 *   3. 页面通过 WebSocket 接收实时 JSON 数据流
 *
 * 依赖:
 *   - crow_all.h (单头文件 HTTP/WebSocket C++ 库)
 *   - 见 https://github.com/CrowCpp/Crow
 * ════════════════════════════════════════════════════════════════════
#include "crow_all.h"
#include <mutex>
#include <unordered_set>
#include <nlohmann/json.hpp>  // 或自定义 JSON 序列化

static std::mutex g_ws_mutex;
static std::unordered_set<crow::websocket::connection*> g_ws_clients;

// ── feedback_frame_t → JSON ──
static std::string SerializeFrame(const feedback_frame_t* fb)
{
    nlohmann::json j;

    // 电机数据
    for (int i = 0; i < 4; i++) {
        std::string prefix = std::string("motor_") + std::to_string(i);
        j[prefix]["pos"]    = fb->motor[i].position;
        j[prefix]["vel"]    = fb->motor[i].velocity;
        j[prefix]["iq"]     = fb->motor[i].current_iq;
        j[prefix]["temp"]   = fb->motor[i].temperature;
        j[prefix]["status"] = fb->motor[i].status_byte;
        j[prefix]["mode"]   = fb->motor[i].mode;
        j[prefix]["err"]    = fb->motor[i].error_code;
    }

    // 传感器数据
    for (int i = 0; i < 4; i++) {
        std::string prefix = std::string("sensor_") + std::to_string(i);
        j[prefix]["hall0"] = fb->sensor[i].hall_adc0;
        j[prefix]["hall1"] = fb->sensor[i].hall_adc1;
        j[prefix]["hall2"] = fb->sensor[i].hall_adc2;
        j[prefix]["force"] = fb->sensor[i].force_raw;
        j[prefix]["knee"]  = fb->sensor[i].knee_adc;
        j[prefix]["land"]  = fb->sensor[i].key_landing;
        j[prefix]["valid"] = fb->sensor[i].data_valid;
    }

    // IMU 数据
    j["imu"]["roll"]  = fb->imu.roll;
    j["imu"]["pitch"] = fb->imu.pitch;
    j["imu"]["yaw"]   = fb->imu.yaw;
    j["imu"]["ax"]    = fb->imu.acc_x;
    j["imu"]["ay"]    = fb->imu.acc_y;
    j["imu"]["az"]    = fb->imu.acc_z;
    j["imu"]["gx"]    = fb->imu.gyro_x;
    j["imu"]["gy"]    = fb->imu.gyro_y;
    j["imu"]["gz"]    = fb->imu.gyro_z;
    j["imu"]["ts"]    = fb->imu.timestamp_us;

    j["ts_us"] = fb->timestamp_us;

    return j.dump();
}

// ── 广播 JSON 到所有 WebSocket 客户端 ──
static void BroadcastFrame(const std::string& json)
{
    std::lock_guard<std::mutex> lock(g_ws_mutex);
    for (auto* conn : g_ws_clients) {
        try {
            conn->send_text(json);
        } catch (...) {
            // 客户端已断开, 下次 onclose 会清理
        }
    }
}

void WebServer::PullLoop()
{
    // ── 启动 crow HTTP 服务器 (独立线程) ──
    crow::SimpleApp app;

    // 静态文件: 调试页面
    CROW_ROUTE(app, "/")
    ([]{
        return R"(
<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Exo Debug</title></head>
<body>
<h1>Exo Node WebSocket Debug</h1>
<pre id="log"></pre>
<script>
var ws = new WebSocket('ws://' + location.host + '/ws');
ws.onmessage = function(e) {
    var data = JSON.parse(e.data);
    var el = document.getElementById('log');
    el.textContent = JSON.stringify(data, null, 2);
    // 只保留最新帧
};
ws.onclose = function() { console.log('WS closed'); };
</script>
</body></html>
        )";
    });

    // ── WebSocket 路由 ──
    CROW_WEBSOCKET_ROUTE(app, "/ws")
    .onopen([&](crow::websocket::connection& conn){
        std::lock_guard<std::mutex> lock(g_ws_mutex);
        g_ws_clients.insert(&conn);
        printf("[WebServer] WS client connected, total=%zu\n",
               g_ws_clients.size());
    })
    .onclose([&](crow::websocket::connection& conn, const std::string& /*reason*/){
        std::lock_guard<std::mutex> lock(g_ws_mutex);
        g_ws_clients.erase(&conn);
        printf("[WebServer] WS client disconnected, total=%zu\n",
               g_ws_clients.size());
    })
    .onmessage([&](crow::websocket::connection& /*conn*/,
                   const std::string& data, bool /*is_binary*/){
        // 上行: 可用于发送调试命令 (预留)
        printf("[WebServer] WS recv: %s\n", data.c_str());
    });

    // ── crow 在后台线程运行 ──
    std::thread crow_thread([&](){
        app.port(m_port).multithreaded().run();
    });
    crow_thread.detach();

    // ── 主循环: 200Hz 推数据 ──
    uint64_t local_count = 0;
    while (m_running.load(std::memory_order_acquire)) {

        if (m_shm) {
            uint32_t active = __atomic_load_n(&m_shm->active_idx,
                                              __ATOMIC_ACQUIRE);
            feedback_frame_t snapshot = m_shm->fb_buffer[active];

            std::string json = SerializeFrame(&snapshot);
            BroadcastFrame(json);
        }

        local_count++;
        m_frame_count++;

        if (local_count % 200 == 0) {  // 每秒一次
            printf("[WebServer] %.1f s, %lu frames\n",
                   (double)m_frame_count / 200.0, m_frame_count);
        }

        usleep(5000);
    }

    // ── 清理 ──
    app.stop();
}
*/

#endif  /* WEBSERVER_PLACEHOLDER */

}  /* namespace stark_periph_manager_node */
