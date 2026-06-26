/*
 * @file WebServer.h
 * @brief WebSocket 调试服务器 — 从 SHM 读取反馈帧, 推送到 WebSocket 客户端
 *
 * ## 定位
 *
 *   WebServer 是一个调试/监控组件, 提供机器人关节数据的实时可视化。
 *   不参与控制闭环, 仅消费 SHM fb_buffer 数据。
 *
 * ## 数据流
 *
 *   motor_node → SHM fb_buffer → PullLoop (200Hz) → JSON → WebSocket → 浏览器
 *
 * ## 依赖
 *
 *   - exo_shm_t* (共享内存)
 *   - crow WebSocket 库 (crow_all.h)
 *   - ENABLE_WEBSERVER 编译开关
 *
 * ## 线程模型
 *
 *   PullLoop 在独立线程中运行, 以 200Hz 周期从 SHM 双 Buffer 读数据,
 *   序列化为 JSON 后推送到所有连接的 WebSocket 客户端。
 */
#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <cstdint>
#include "interface/IListener.hpp"
#include "exo_shm.h"

namespace stark_periph_manager_node {

class WebServer : public IListener {
public:
    /**
     * @param shm  共享内存指针 (exo_shm_mgr_t->ptr)
     * @param port HTTP/WebSocket 端口, 默认 8080
     */
    WebServer(exo_shm_t* shm, uint16_t port = 8080);
    ~WebServer();

    /* ── IListener 接口 ── */
    void Update(const boost::any& data) override;

    /* ── 生命周期 ── */
    void Start();
    void Stop();
    bool IsRunning() const { return m_running.load(std::memory_order_acquire); }

private:
    /**
     * @brief 主循环 — 200Hz 从 SHM 读反馈帧, JSON 序列化, push 到 WebSocket
     *
     * 流程:
     *   1. atomic_load(active_idx, acquire)  →  读活跃 Buffer 索引
     *   2. memcpy fb_buffer[active]  →  防撕裂快照
     *   3. 序列化 feedback_frame_t → JSON 字符串
     *   4. 遍历 websocket_clients, push(json)
     *   5. usleep(5000)  →  200Hz
     */
    void PullLoop();

    /* ── 外部依赖 ── */
    exo_shm_t*      m_shm;
    uint16_t        m_port;

    /* ── 线程控制 ── */
    std::atomic<bool> m_running;
    std::thread     m_thread;

    /* ── 统计 ── */
    uint64_t m_frame_count;         /* 总推送帧数 */
    uint64_t m_fail_count;          /* 推送失败次数 */
};

}  /* namespace stark_periph_manager_node */
