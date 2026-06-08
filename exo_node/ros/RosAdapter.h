/*
 * @file RosAdapter.h
 * @brief ROS 适配器 — 独立线程 pull SHM 反馈帧 → 发布 ROS Topic
 *
 * 与 petrobot 的 RosAdapter 区别:
 *   petrobot: Update(data) 由 UartDispatcher::NotifyObserver 触发 (push)
 *   exo_node:  内部启动独立 PullLoop 线程, 定期从 SHM 读反馈帧 (pull)
 *
 * ROS Topics:
 *   Pub:  /stark/motor/feedback  (反馈帧 JSON)
 *   Pub:  /stark/motor/state     (节点状态 JSON)
 *   Sub:  /stark/motor/ctrl      (控制命令 JSON → dispatcher->Send)
 */
#pragma once

#include <ros/ros.h>
#include <memory>
#include <thread>
#include <atomic>
#include <boost/any.hpp>
#include "interface/IListener.hpp"

/* ROS 消息类型 — 先不引入自定义 msg，用 std_msgs/String 占位 */
#include <std_msgs/String.h>

/* ── SHM 数据布局 (motordata_t, feedback_frame_t, exo_state_t 等) ── */
extern "C" {
#include "exo_shm.h"
}

namespace stark_periph_manager_node {

class IMsgInternalDispatcher;

class RosAdapter : public IListener {
public:
    RosAdapter(std::shared_ptr<ros::NodeHandle> nh, exo_shm_t* shm);
    ~RosAdapter();

    /* IListener 接口 — 保留以兼容 Observer 模式, 但主要数据走 pull */
    void Update(const boost::any& data) override;

    /* 启动/停止 pull 线程 (独立线程从 SHM 读反馈帧) */
    void Start();
    void Stop();

    /* 设置 Dispatcher 引用 (用于 ROS Subscriber 回调中发控制命令) */
    void SetDispatcher(std::shared_ptr<IMsgInternalDispatcher> dispatcher);

private:
    /* pull 线程: 读 SHM → publish */
    void PullLoop();

    /* Publisher */
    void PubFeedback(const feedback_frame_t& fb);
    void PubState(exo_state_t state);

    /* Subscriber 回调 — 接收上层控制命令 */
    void OnMotorCtrl(const std_msgs::String::ConstPtr& msg);

    std::shared_ptr<ros::NodeHandle> m_nh;
    exo_shm_t*              m_shm;
    std::shared_ptr<IMsgInternalDispatcher> m_dispatcher;
    std::thread             m_pullThread;
    std::atomic<bool>       m_running;

    /* Publishers */
    ros::Publisher m_feedbackPub;       /* /stark/motor/feedback */
    ros::Publisher m_statePub;          /* /stark/motor/state   */

    /* Subscribers */
    ros::Subscriber m_motorCtrlSub;     /* /stark/motor/ctrl     */
};

}  /* namespace stark_periph_manager_node */
