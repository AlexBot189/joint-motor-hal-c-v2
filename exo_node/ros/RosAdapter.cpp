/*
 * @file RosAdapter.cpp
 * @brief ROS 适配器实现 — pull SHM 反馈 + Subscriber 控制
 *
 * 与 petrobot RosAdapter 的关键差异:
 *   - petrobot: push 模式 (UartDispatcher::NotifyObserver → Update → Pub)
 *   - stark:     pull 模式 (独立线程定期读 SHM fb_buffer → Pub)
 *
 * Pull 优势: 消费者独立, ROS crash 不影响实时控制路径。
 * Subscriber 路径: ROS → OnMotorCtrl → dispatcher->Send (JSON) → SDO → CAN
 */
#include "RosAdapter.h"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sstream>

#include "interface/IMsgInternalDispatcher.hpp"

namespace stark_periph_manager_node
{

/* ================================================================
 * 构造 / 析构
 * ================================================================ */

RosAdapter::RosAdapter(std::shared_ptr<ros::NodeHandle> nh, exo_shm_t* shm)
    : m_nh(nh)
    , m_shm(shm)
    , m_running(false)
{
    /* Publisher */
    m_feedbackPub = nh->advertise<std_msgs::String>(
        "/stark/motor/feedback", 10);
    m_statePub = nh->advertise<std_msgs::String>(
        "/stark/motor/state", 10);

    /* Subscriber: 接收上层控制命令 */
    m_motorCtrlSub = nh->subscribe<std_msgs::String>(
        "/stark/motor/ctrl", 10, &RosAdapter::OnMotorCtrl, this);

    ECO_INFO("[RosAdapter] publishers/subscribers created");
}

RosAdapter::~RosAdapter()
{
    Stop();
}

/* ================================================================
 * Update — IListener 接口 (保留, 用于状态事件通知)
 * ================================================================ */

void RosAdapter::Update(const boost::any& data)
{
    /* 主数据流通过 PullLoop 从 SHM 拉取,
     * Update 仅用于状态变更事件 (state transition / fault) */
    (void)data;
}

/* ================================================================
 * SetDispatcher
 * ================================================================ */

void RosAdapter::SetDispatcher(std::shared_ptr<IMsgInternalDispatcher> dispatcher)
{
    m_dispatcher = dispatcher;
}

/* ================================================================
 * Start / Stop
 * ================================================================ */

void RosAdapter::Start()
{
    if (m_running) return;

    m_running = true;
    m_pullThread = std::thread(&RosAdapter::PullLoop, this);
    ECO_INFO("[RosAdapter] pull thread started");
}

void RosAdapter::Stop()
{
    m_running = false;
    if (m_pullThread.joinable()) {
        m_pullThread.join();
    }
}

/* ================================================================
 * PullLoop — SHM → ROS Topic
 *
 * 周期: 5ms (200Hz)
 * 从 SHM fb_buffer[active_idx] 读取最新反馈帧,
 * 序列化为 JSON 发布到 /stark/motor/feedback 和 /stark/motor/state。
 * ================================================================ */

void RosAdapter::PullLoop()
{
    ECO_INFO("[RosAdapter] PullLoop running (200Hz)");

    while (m_running) {
        if (!m_shm) {
            usleep(5000);
            continue;
        }

        /* 读双 Buffer 活跃帧 (acquire 语义, 保证完整性) */
        uint32_t active = __atomic_load_n(&m_shm->active_idx,
                                          __ATOMIC_ACQUIRE);
        const feedback_frame_t& fb = m_shm->fb_buffer[active];

        /* 发布反馈帧 */
        PubFeedback(fb);

        /* 发布状态 (100ms 一次, 降低频率) */
        static int state_counter = 0;
        if (++state_counter >= 20) {
            state_counter = 0;
            PubState((exo_state_t)m_shm->node_state);
        }

        usleep(5000);  /* 5ms → 200Hz */
    }
}

/* ================================================================
 * PubFeedback — 反馈帧 → JSON → ROS Topic
 *
 * 格式:
 * {
 *   "type": "feedback",
 *   "ts": 1234567890,
 *   "motor": [
 *     {"id":1, "pos":0, "vel":0, "cur":0, "temp":0, "status":0}
 *   ],
 *   "imu": {"roll":0, "pitch":0, "yaw":0}
 * }
 * ================================================================ */

void RosAdapter::PubFeedback(const feedback_frame_t& fb)
{
    if (!m_shm) return;

    /* 用 snprintf 拼 JSON (轻量, 避免 nlohmann 在 ros 线程中分配内存) */
    char buf[2048];
    int off = 0;

    off += snprintf(buf + off, sizeof(buf) - off,
        "{\"type\":\"feedback\",\"ts\":%lu,\"motor\":[",
        (unsigned long)fb.timestamp_us);

    for (int i = 0; i < 2; ++i) {  /* 只发送已配置的髋关节电机 */
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        uint8_t online = m_shm->motor_online;
        if (!(online & (1 << i))) {
            off += snprintf(buf + off, sizeof(buf) - off, "null");
            continue;
        }
        off += snprintf(buf + off, sizeof(buf) - off,
            "{\"id\":%d,\"pos\":%d,\"vel\":%d,\"cur\":%d,"
            "\"temp\":%d,\"status\":%d}",
            i + 1,
            (int)fb.motor[i].position,
            (int)fb.motor[i].velocity,
            (int)fb.motor[i].current_iq,
            (int)fb.motor[i].temperature,
            (int)fb.motor[i].status_byte);
    }

    off += snprintf(buf + off, sizeof(buf) - off,
        "],\"imu\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"severity\":%d,\"fault\":%d}",
        (double)fb.imu.roll, (double)fb.imu.pitch, (double)fb.imu.yaw,
        (int)m_shm->motor_severity, (int)m_shm->fault_reason);

    std_msgs::String msg;
    msg.data = buf;
    m_feedbackPub.publish(msg);
}

/* ================================================================
 * PubState — 节点状态 → ROS Topic
 * ================================================================ */

void RosAdapter::PubState(exo_state_t state)
{
    if (!m_shm) return;

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"state\",\"state\":\"%s\",\"severity\":%d,"
        "\"reason\":%d,\"online\":%d}",
        state_name(state),
        (int)m_shm->motor_severity,
        (int)m_shm->fault_reason,
        (int)m_shm->motor_online);

    std_msgs::String msg;
    msg.data = buf;
    m_statePub.publish(msg);
}

/* ================================================================
 * OnMotorCtrl — ROS Subscriber 回调 → dispatcher->Send
 *
 * 接收 JSON 控制命令:
 *   {"motor_id":1,"cmd":"torque","value":500}
 *   {"motor_id":2,"cmd":"position","value":9000}
 *
 * 透传到 CanDispatcher::Send → SDO → CANFD (非实时路径)。
 * ================================================================ */

void RosAdapter::OnMotorCtrl(const std_msgs::String::ConstPtr& msg)
{
    if (!m_dispatcher) {
        ECO_WARN("[RosAdapter] OnMotorCtrl: no dispatcher");
        return;
    }

    m_dispatcher->Send(msg->data);
}

}  /* namespace stark_periph_manager_node */
