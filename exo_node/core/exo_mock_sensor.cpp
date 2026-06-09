/*
 * exo_mock_sensor.cpp — IMU + 气压计 Mock 数据源实现
 *
 * 模拟真实传感器行为:
 *   - IMU: sin 波形模拟周期运动 + 随机噪声
 *   - 气压计: 固定基准值 + 小幅随机漂移
 */
#include "exo_mock_sensor.h"
#include <cmath>
#include <cstdlib>  /* rand() */

namespace stark_periph_manager_node {

ExoMockSensor::ExoMockSensor()
    : m_base_roll(0.0f)
    , m_base_pitch(0.0f)
    , m_base_yaw(0.0f)
    , m_base_pressure(1013.25f)
    , m_base_temp(25.0f)
    , m_gyro_noise(0.5f)
    , m_acc_noise(0.05f)
    , m_press_noise(0.1f)
    , m_phase_counter(0)
{
    memset(&m_imu,  0, sizeof(m_imu));
    memset(&m_baro, 0, sizeof(m_baro));
}

void ExoMockSensor::Init()
{
    m_phase_counter = 0;

    /* IMU 基准: 站立姿态 (俯仰前倾 5°) */
    m_base_roll  = 0.0f;
    m_base_pitch = 5.0f;
    m_base_yaw   = 0.0f;

    /* 气压基准: 海平面 */
    m_base_pressure = 1013.25f;
    m_base_temp     = 25.0f;

    m_gyro_noise  = 0.5f;
    m_acc_noise   = 0.05f;
    m_press_noise = 0.1f;
}

void ExoMockSensor::Tick(uint64_t timestamp_us)
{
    m_phase_counter++;

    /* ── 辅助: [-1, 1] 均匀随机 ── */
    float rand01 = (float)rand() / (float)RAND_MAX;
    float rand_n = 2.0f * rand01 - 1.0f;  /* [-1, 1] */

    /* ── 辅助: 相位正弦 (模拟 1Hz 周期运动) ── */
    double phase = (double)m_phase_counter * 0.0062831853;  /* 2π / 1000 */
    float  sin_wave = (float)sin(phase);
    float  cos_wave = (float)cos(phase);

    /* ════════════════════════════════════════════════════════════
     * IMU 模拟
     * ════════════════════════════════════════════════════════════ */

    m_imu.timestamp_us = timestamp_us;

    /* 姿态角: 基准 + 3°正弦摆动 + 噪声 */
    m_imu.roll  = m_base_roll  + 3.0f * sin_wave  + m_gyro_noise * rand_n * 0.01f;
    m_imu.pitch = m_base_pitch + 3.0f * cos_wave  + m_gyro_noise * rand_n * 0.01f;
    m_imu.yaw   = m_base_yaw   + 0.5f * sin_wave  + m_gyro_noise * rand_n * 0.01f;

    /* 加速度: 重力投影 + 运动加速度 + 噪声 */
    m_imu.acc_x = 9.81f * sinf(m_imu.roll  * 0.0174533f)  + m_acc_noise * rand_n;
    m_imu.acc_y = 9.81f * sinf(m_imu.pitch * 0.0174533f)  + m_acc_noise * rand_n;
    m_imu.acc_z = 9.81f * cosf(m_imu.roll  * 0.0174533f)
                * cosf(m_imu.pitch * 0.0174533f)           + m_acc_noise * rand_n;

    /* 角速度: 姿态变化率 (微分近似) + 噪声 */
    m_imu.gyro_x = 18.8496f * cos_wave  + m_gyro_noise * rand_n;  /* 3° * 2π / 1000 */
    m_imu.gyro_y = -18.8496f * sin_wave + m_gyro_noise * rand_n;
    m_imu.gyro_z = m_gyro_noise * rand_n * 0.1f;

    /* ════════════════════════════════════════════════════════════
     * 气压计模拟
     * ════════════════════════════════════════════════════════════ */

    m_baro.timestamp_us = timestamp_us;

    /* 气压: 基准 + 缓慢漂移 + 噪声 */
    m_baro.pressure_hpa = m_base_pressure
                        + 0.02f * sin_wave           /* 呼吸级波动           */
                        + m_press_noise * rand_n;

    /* 温度: 基准 + 缓慢热漂移 + 噪声 */
    m_baro.temperature_c = m_base_temp
                         + 0.1f * sinf((float)m_phase_counter * 0.0001f)
                         + 0.05f * rand_n;

    /* 海拔: 标准大气压公式估算 */
    m_baro.altitude_m = 44330.0f * (1.0f - powf(m_baro.pressure_hpa / 1013.25f, 0.1903f));
}

void ExoMockSensor::SetImuNoise(float gyro_noise, float acc_noise)
{
    m_gyro_noise = gyro_noise;
    m_acc_noise  = acc_noise;
}

void ExoMockSensor::SetBaroNoise(float press_noise)
{
    m_press_noise = press_noise;
}

void ExoMockSensor::ResetPhase()
{
    m_phase_counter = 0;
}

}  /* namespace stark_periph_manager_node */
