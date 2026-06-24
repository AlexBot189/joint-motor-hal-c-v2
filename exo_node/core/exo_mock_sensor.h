/*
 * exo_mock_sensor.h — IMU + 气压计 Mock 数据源
 *
 * 真实硬件 (ICM45608 / QMP6990) 拿到之前,
 * 用固定基准值 + 可控随机抖动模拟传感器读数,
 * 验证完整的端到端数据链路.
 *
 * 使用:
 *   ExoMockSensor sensor;
 *   sensor.Init();
 *   sensor.Tick();  // 每周期 (1KHz) 更新内部状态
 *
 *   imu_data_t       imu  = sensor.GetImu();
 *   barometer_data_t baro = sensor.GetBaro();
 */
#pragma once

#include "exo_shm.h"
#include <cstdint>

namespace stark_periph_manager_node {

class ExoMockSensor {
public:
    ExoMockSensor();

    /* ── 初始化 (设置基准值) ── */
    void Init();

    /* ── 每周期更新 (内部 sin 波形 + 随机抖动) ── */
    void Tick(uint64_t timestamp_us);

    /* ── 读取当前值 ── */
    imu_data_t       GetImu()       const { return m_imu; }
    barometer_data_t GetBaro()      const { return m_baro; }

    /* ── 配置噪声幅度 ── */
    void SetImuNoise(float gyro_noise, float acc_noise);
    void SetBaroNoise(float press_noise);

    /* ── 重置相位 (sin 波形归零) ── */
    void ResetPhase();

private:
    imu_data_t       m_imu;
    barometer_data_t m_baro;

    /* ── 基准值 ── */
    float m_base_roll;        /* 横滚基准, °              */
    float m_base_pitch;       /* 俯仰基准, °              */
    float m_base_yaw;         /* 偏航基准, °              */
    float m_base_pressure;    /* 气压基准, hPa            */
    float m_base_temp;        /* 温度基准, °C             */

    /* ── 噪声幅度 ── */
    float m_gyro_noise;       /* 角速度噪声, °/s peak-to-peak  */
    float m_acc_noise;        /* 加速度噪声, m/s² p-p          */
    float m_press_noise;      /* 气压噪声, hPa p-p             */

    /* ── 相位计数器 ── */
    uint64_t m_phase_counter;
};

}  /* namespace stark_periph_manager_node */
