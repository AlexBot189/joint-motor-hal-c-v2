/*
 * CanDispatcher.h — 电机数据调度中心
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 初始化流程:
 *   1. motor_hal_create + init (CANFD)
 *   2. 注册电机 (ID 1,2)
 *   3. recv_start
 *   4. 创建 SHM
 *   5. 主循环 (事件驱动)
 *
 * 所有 SDO/PDO/OD 控制通过 StarkMotorCtrl 封装.
 * RT 控制走 StarkRtWorker ,  SHM mailbox.
 */
#pragma once

#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"
#include "interface/Defines.hpp"

#include <unordered_map>
#include <memory>
#include <string>
#include <mutex>

extern "C" {
#include "stark_shm.h"
#include "shm/shm_mgr.h"
#include "motor_hal.h"
}

#include "motor/motor_ctrl.h"
#include "imu/imu_sensor.h"
#include "motor/motor_rt_worker.h"

namespace stark_periph_manager_node {

class CanDispatcher : public IMsgInternalDispatcher {
public:
    CanDispatcher();
    ~CanDispatcher();

    /* IMsgInternalDispatcher */
    bool InitDispatcher() override;
    bool DestroyDispatcher() override;
    void Send(const std::string& data) override;
    void RegisterObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void RemoveObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void NotifyObserver(const boost::any& data) override;

    /* 获取内部实例 */
    motor_hal_t*   GetHal()       { return m_hal; }
    stark_shm_t*     GetShm()       { return m_shm; }
    StarkMotorCtrl*  GetCtrl()      { return m_ctrl.get(); }
    ImuHALSensor*  GetImuSensor() { return m_imu_sensor.get(); }

    /* 配置 (从 config.json 读取, 或默认值) */
    const SafetyConfig& GetSafetyConfig()  { return m_safety_cfg; }
    const RtConfig&     GetRtConfig()      { return m_rt_cfg; }
    const std::string&  GetShmName()       { return m_shm_name; }
    const std::string&  GetCanIface()      { return m_can_iface; }

    /* 配置获取 (主线程读取, 设置 g_ctx) */
    int  GetMotorCount()     const { return m_motor_count; }
    bool GetCalibAuto()      const { return m_calib_auto; }
    int  GetCalibTimeoutMs() const { return m_calib_timeout_ms; }
    uint16_t GetSensorPeriodMs()   const { return m_sensor_period_ms; }
    uint16_t GetSensorPeriodDiv()  const { return m_sensor_period_div; }
    uint8_t  GetSensorBusFormat()  const { return m_sensor_bus_format; }
    uint8_t  GetSensorMode()        const { return m_sensor_mode; }
    uint8_t  GetSensorForceModule() const { return m_sensor_force_module; }
    bool     GetReportAutoEnable() const { return m_report_auto_enable; }
    uint32_t GetReportPeriodMs()   const { return m_report_period_ms; }
    bool     GetMotorAutoEnable()  const { return m_motor_auto_enable; }

    bool IsRunning() const { return m_running; }
    void SetConfigPath(const std::string& path) { m_config_path = path; }

private:
    bool LoadMotorConfig();
    void _dispatch_command(const std::string& cmd, uint8_t id, int value);

    motor_hal_t*    m_hal;
    stark_shm_mgr_t*  m_shm_mgr;
    stark_shm_t*      m_shm;
    bool            m_running;

    std::unique_ptr<StarkMotorCtrl> m_ctrl;

    std::mutex m_listener_mutex;
    std::unordered_map<ListenerType, std::shared_ptr<IListener>> m_listeners;
    std::string m_config_path;

    /* 配置 (优先 config.json, 读失败则默认值) */
    SafetyConfig m_safety_cfg;
    RtConfig     m_rt_cfg;
    std::string  m_shm_name  = STARK_SHM_NAME;
    size_t       m_shm_size_bytes = STARK_SHM_SIZE;
    std::string  m_can_iface = "can0";
    int          m_can_arb_rate  = 1000000;
    int          m_can_data_rate = 5000000;

    /* IMU HAL */
    std::unique_ptr<ImuHALSensor> m_imu_sensor;
    std::string  m_imu_i2c_dev    = "/dev/i2c-3";
    std::string  m_imu_gpio_chip  = "gpiochip4";
    unsigned int m_imu_gpio_line  = 6;
    int          m_imu_op_mode    = 5;

    /* 电机数量 (从 config.json motors 数组长度读取) */
    int          m_motor_count    = 2;

    /* 校准/透传配置 (来自 config.json) */
    bool         m_calib_auto       = false;
    int          m_calib_timeout_ms = 10000;
    uint16_t     m_sensor_period_ms = 1;
    uint16_t     m_sensor_period_div = 1;   /* 0.5ms 基准分频, 默认 1 */
    uint8_t      m_sensor_bus_format = 3;  /* CANFD BRS */
    uint8_t      m_sensor_mode = 2;         /* 0=关 1=仅传感器帧 2=全部帧 */
    uint8_t      m_sensor_force_module = 1; /* 0=CAN力矩 1=SPI力矩 */
    bool         m_report_auto_enable = true;  /* 校准后自动开启周期上报 */
    uint32_t     m_report_period_ms   = 5;    /* 上报周期 ms */
    bool         m_motor_auto_enable  = false; /* 任意电机 auto_enable=true 则置 true */
};

}  /* namespace stark_periph_manager_node */
