/*
 * @file IListener.hpp
 * @brief 观察者基类 — 接收 CanDispatcher 推送的数据
 *
 * 所有需要接收电机反馈/传感器/状态数据的模块实现此接口。
 * RosAdapter / WebServer 通过 Update() 接收 boost::any 数据包。
 */
#pragma once

#include <memory>
#include <boost/any.hpp>
#include "interface/Defines.hpp"

namespace stark_periph_manager_node
{

class IMsgInternalDispatcher;

class IListener
{
public:
    virtual ~IListener() = default;

    virtual void
    Update(const boost::any& data) = 0;
};

}  /* namespace stark_periph_manager_node */
