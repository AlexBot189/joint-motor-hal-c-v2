/*
 * RosAdapter.h — ROS 适配器
 *
 * pull 模式: 独立线程从 SHM 读反馈帧 → publish ROS Topic
 * Subscriber 路径: ROS → dispatcher->Send (JSON) → SDO → CAN
 */
#pragma once

#include <ros/ros.h>
#include <memory>
#include <thread>
#include <atomic>
#include <boost/any.hpp>
#include "interface/IListener.hpp"
#include <std_msgs/String.h>

extern "C" {
#include "exo_shm.h"
}

namespace stark_periph_manager_node {

class IMsgInternalDispatcher;

class RosAdapter : public IListener {
public:
    RosAdapter(std::shared_ptr<ros::NodeHandle> nh, exo_shm_t* shm);
    ~RosAdapter();

    void Update(const boost::any& data) override;

    void Start();
    void Stop();
    void SetDispatcher(std::shared_ptr<IMsgInternalDispatcher> dispatcher);

private:
    void PullLoop();
    void PubFeedback(const feedback_frame_t& fb);
    void PubState(exo_state_t state);
    void OnMotorCtrl(const std_msgs::String::ConstPtr& msg);

    std::shared_ptr<ros::NodeHandle>        m_nh;
    exo_shm_t*                              m_shm;
    std::shared_ptr<IMsgInternalDispatcher> m_dispatcher;
    std::thread                             m_pullThread;
    std::atomic<bool>                       m_running;

    ros::Publisher  m_feedbackPub;
    ros::Publisher  m_statePub;
    ros::Subscriber m_motorCtrlSub;
};

}  /* namespace stark_periph_manager_node */
