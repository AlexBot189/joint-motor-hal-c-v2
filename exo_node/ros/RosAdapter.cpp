/*
 * @file RosAdapter.cpp
 * @brief ROS 适配器实现 — pull 线程 + publish/subscribe
 *
 * 线程模型:
 *   PullLoop 线程 (SCHED_OTHER):     从 SHM 读反馈帧 → serialize JSON → publish
 *   ROS 回调线程:                     接收 /stark/motor/ctrl → dispatcher->Send
 *   主线程:                           构造/Start/Stop
 *
 * JSON 序列化使用 snprintf，不引入 nlohmann/json 依赖。
 */
#include "ros/RosAdapter.h"
#include "interface/IMsgInternalDispatcher.hpp"
#include "src/exo_log.h"

#include <unistd.h>     /* usleep */
#include <cstdio>       /* snprintf */
#include <cstring>      /* strlen, strncmp */
#include <chrono>

namespace stark_periph_manager_node {

/* ============================================================================
 *  构造 / 析构
 * ============================================================================ */

RosAdapter::RosAdapter(std::shared_ptr<ros::NodeHandle> nh, exo_shm_t* shm)
    : m_nh(std::move(nh))
    , m_shm(shm)
    , m_dispatcher(nullptr)
    , m_running(false)
{
    if (!m_nh || !m_shm) {
        EXO_ERROR("[RosAdapter] 构造参数无效: nh=%p shm=%p",
                  static_cast<const void*>(m_nh.get()),
                  static_cast<const void*>(m_shm));
        return;
    }

    /* 创建 Publishers */
    m_feedbackPub = m_nh->advertise<std_msgs::String>("/stark/motor/feedback", 10);
    m_statePub    = m_nh->advertise<std_msgs::String>("/stark/motor/state",    10);

    /* 创建 Subscriber */
    m_motorCtrlSub = m_nh->subscribe("/stark/motor/ctrl", 10,
                                     &RosAdapter::OnMotorCtrl, this);

    EXO_INFO("[RosAdapter] 构造完成: feedback pub=%d sub=%d",
             m_feedbackPub.getNumSubscribers(),
             m_motorCtrlSub.getNumPublishers());
}

RosAdapter::~RosAdapter()
{
    Stop();
    EXO_INFO("[RosAdapter] 析构完成");
}

/* ============================================================================
 *  Start / Stop — pull 线程控制
 * ============================================================================ */

void RosAdapter::Start()
{
    if (m_running.load(std::memory_order_acquire)) {
        EXO_WARN("[RosAdapter] Start: pull 线程已在运行");
        return;
    }

    m_running.store(true, std::memory_order_release);
    m_pullThread = std::thread(&RosAdapter::PullLoop, this);
    EXO_INFO("[RosAdapter] pull 线程已启动");
}

void RosAdapter::Stop()
{
    if (!m_running.load(std::memory_order_acquire)) {
        return;
    }

    m_running.store(false, std::memory_order_release);
    if (m_pullThread.joinable()) {
        m_pullThread.join();
    }
    EXO_INFO("[RosAdapter] pull 线程已停止");
}

/* ============================================================================
 *  IListener::Update — 保留接口兼容性 (主要数据走 pull)
 * ============================================================================ */

void RosAdapter::Update(const boost::any& /*data*/)
{
    /* 保留接口兼容性 — exo_node 使用 pull 模式，此方法为空 */
}

/* ============================================================================
 *  SetDispatcher — 注入控制命令转发器
 * ============================================================================ */

void RosAdapter::SetDispatcher(std::shared_ptr<IMsgInternalDispatcher> dispatcher)
{
    m_dispatcher = std::move(dispatcher);
    EXO_INFO("[RosAdapter] Dispatcher 已注入: %p",
             static_cast<const void*>(m_dispatcher.get()));
}

/* ============================================================================
 *  PullLoop — 独立线程循环: 读 SHM → Publish
 * ============================================================================ */

void RosAdapter::PullLoop()
{
    EXO_INFO("[RosAdapter] PullLoop 开始");

    uint32_t last_active_idx = m_shm->active_idx;

    while (m_running.load(std::memory_order_acquire)) {
        /* 5ms 周期 — 与 exo_rt_worker 的 1kHz 控制周期匹配 */
        usleep(5000);

        /* ── 读取当前活跃 Buffer 索引 (atomic acquire) ── */
        uint32_t idx = __atomic_load_n(&m_shm->active_idx, __ATOMIC_ACQUIRE);

        /* 如果 SHM 没有更新，跳过本次发布 */
        if (idx == last_active_idx) {
            continue;
        }
        last_active_idx = idx;

        /* ── 读取反馈帧 ── */
        const feedback_frame_t& fb = m_shm->fb_buffer[idx];

        /* ── 发布 ── */
        PubFeedback(fb);
        PubState(static_cast<exo_state_t>(m_shm->node_state));
    }

    EXO_INFO("[RosAdapter] PullLoop 结束");
}

/* ============================================================================
 *  PubFeedback — 序列化 feedback_frame_t → JSON → Publish
 *
 *  JSON 格式:
 *  {
 *    "type": "feedback",
 *    "ts": 1234567890,
 *    "motor": [
 *      {"id":1,"pos":1234,"vel":56,"iq":789,"temp":45.0,"status":"ENABLED","mode":8,"err":0},
 *      ...
 *    ],
 *    "imu": {
 *      "roll":0.12,"pitch":0.34,"yaw":1.56,
 *      "acc_x":0.01,"acc_y":0.02,"acc_z":9.81,
 *      "gyro_x":0.1,"gyro_y":0.2,"gyro_z":0.3,
 *      "ts_us":1234567890
 *    }
 *  }
 * ============================================================================ */

static const char* motor_status_str(uint8_t status_byte)
{
    /* bit7:使能 bit6:抱闸 bit5:错误 bit4:到位 */
    if (status_byte & 0x20) return "ERROR";
    if ((status_byte & 0xF0) == 0x90) return "ENABLED+BRAKE";
    if ((status_byte & 0xF0) == 0x80) return "ENABLED";
    return "DISABLED";
}

void RosAdapter::PubFeedback(const feedback_frame_t& fb)
{
    /* 预估 JSON 最大长度:
     *   header:      ~100
     *   motor ×4:    ~200 each = 800
     *   imu:         ~300
     *   total:       ~1300 → buffer 2048 够用
     */
    char buf[2048];
    char* p = buf;
    char* end = buf + sizeof(buf);
    int written = 0;

    /* ── JSON 头部 ── */
    written = snprintf(p, end - p,
        "{\"type\":\"feedback\",\"ts\":%lu,\"motor\":[",
        (unsigned long)fb.timestamp_us);
    if (written <= 0 || written >= end - p) goto fail;
    p += written;

    /* ── 电机数组 (4 个电机) ── */
    for (int i = 0; i < 4; ++i) {
        const motor_data_t& m = fb.motor[i];
        if (i > 0) {
            written = snprintf(p, end - p, ",");
            if (written <= 0) goto fail;
            p += written;
        }
        written = snprintf(p, end - p,
            "{\"id\":%d,\"pos\":%d,\"vel\":%d,\"iq\":%d,"
            "\"temp\":%.1f,\"status\":\"%s\",\"mode\":%d,\"err\":%d}",
            i + 1,                               /* motor id: 1-4 */
            m.position, m.velocity, m.current_iq,
            m.temperature * 0.1f,                /* 0.1°C → °C   */
            motor_status_str(m.status_byte),
            m.mode, m.error_code);
        if (written <= 0 || written >= end - p) goto fail;
        p += written;
    }

    /* ── 电机数组结束 + IMU 开始 ── */
    written = snprintf(p, end - p, "],\"imu\":{");
    if (written <= 0 || written >= end - p) goto fail;
    p += written;

    /* ── IMU 数据 ── */
    const imu_data_t& imu = fb.imu;
    written = snprintf(p, end - p,
        "\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,"
        "\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,"
        "\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f,"
        "\"ts_us\":%lu}",
        imu.roll, imu.pitch, imu.yaw,
        imu.acc_x, imu.acc_y, imu.acc_z,
        imu.gyro_x, imu.gyro_y, imu.gyro_z,
        (unsigned long)imu.timestamp_us);
    if (written <= 0 || written >= end - p) goto fail;
    p += written;

    /* ── JSON 尾部 ── */
    written = snprintf(p, end - p, "}");
    if (written <= 0 || written >= end - p) goto fail;
    p += written;

    /* ── Publish ── */
    std_msgs::String msg;
    msg.data = std::string(buf, p - buf);
    m_feedbackPub.publish(msg);
    return;

fail:
    EXO_ERROR("[RosAdapter] PubFeedback: snprintf 缓冲区溢出 (剩余 %td 字节)", end - p);
}

/* ============================================================================
 *  PubState — 序列化状态信息 → JSON → Publish
 *
 *  JSON 格式:
 *  {"type":"state","value":"STATE_READY","severity":0,"reason":"OK"}
 * ============================================================================ */

static const char* exo_state_name(exo_state_t state)
{
    switch (state) {
    case STATE_INIT:        return "STATE_INIT";
    case STATE_DISCOVERY:   return "STATE_DISCOVERY";
    case STATE_READY:       return "STATE_READY";
    case STATE_CALIBRATING: return "STATE_CALIBRATING";
    case STATE_ENABLED:     return "STATE_ENABLED";
    case STATE_RUNNING:     return "STATE_RUNNING";
    case STATE_FAULT:       return "STATE_FAULT";
    default:                return "UNKNOWN";
    }
}

static const char* fault_reason_name(uint8_t reason)
{
    switch (reason) {
    case FAULT_NONE:              return "OK";
    case FAULT_ALGO_TIMEOUT:      return "ALGO_TIMEOUT";
    case FAULT_CMD_STALL:         return "CMD_STALL";
    case FAULT_CAN_OFFLINE:       return "CAN_OFFLINE";
    case FAULT_ENCODER_FAULT:     return "ENCODER_FAULT";
    case FAULT_OVERTEMP:          return "OVERTEMP";
    case FAULT_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
    case FAULT_CALIB_TIMEOUT:     return "CALIB_TIMEOUT";
    default:                      return "UNKNOWN_FAULT";
    }
}

static const char* severity_name(uint8_t severity)
{
    switch (severity) {
    case MOTOR_OK:    return "OK";
    case MOTOR_WARN:  return "WARN";
    case MOTOR_FAULT: return "FAULT";
    default:          return "UNKNOWN";
    }
}

void RosAdapter::PubState(exo_state_t state)
{
    char buf[256];
    int written = snprintf(buf, sizeof(buf),
        "{\"type\":\"state\",\"value\":\"%s\",\"severity\":\"%s\",\"reason\":\"%s\"}",
        exo_state_name(state),
        severity_name(m_shm->motor_severity),
        fault_reason_name(m_shm->fault_reason));

    if (written <= 0 || written >= (int)sizeof(buf)) {
        EXO_ERROR("[RosAdapter] PubState: snprintf 缓冲区溢出");
        return;
    }

    std_msgs::String msg;
    msg.data = std::string(buf, written);
    m_statePub.publish(msg);
}

/* ============================================================================
 *  OnMotorCtrl — ROS Subscriber 回调
 *
 *  接收 JSON 控制命令，格式:
 *  {"motor_id":1,"cmd":"torque","value":500}
 *
 *  cmd 取值: "torque" (力矩), "speed" (速度), "pos" (位置)
 *
 *  将 JSON 原样转发到 dispatcher->Send() (CanDispatcher 负责解析)
 * ============================================================================ */

void RosAdapter::OnMotorCtrl(const std_msgs::String::ConstPtr& msg)
{
    if (!m_dispatcher) {
        EXO_WARN("[RosAdapter] OnMotorCtrl: dispatcher 未注入, 丢弃命令: %s",
                 msg->data.c_str());
        return;
    }

    /* 简单校验 JSON 格式 (含 motor_id / cmd / value 基本字段) */
    const std::string& data = msg->data;
    if (data.find("\"motor_id\"") == std::string::npos ||
        data.find("\"cmd\"") == std::string::npos ||
        data.find("\"value\"") == std::string::npos) {
        EXO_WARN("[RosAdapter] OnMotorCtrl: JSON 格式非法: %s", data.c_str());
        return;
    }

    /* 透传到内部调度器 */
    m_dispatcher->Send(data);
}

}  /* namespace stark_periph_manager_node */
