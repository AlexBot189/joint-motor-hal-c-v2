/*
 * exo_imu_sensor.cpp — IMU HAL 传感器实现
 *
 * 封装 libimu_hal.so (emd_gaf API)。
 * 硬件未接入时所有 Read() 返回零，不阻塞不崩溃。
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "exo_imu_sensor.h"

#include <cstring>
#include <cstdio>

extern "C" {
#include "emd_gaf.h"
}

namespace stark_periph_manager_node {

ImuHALSensor::~ImuHALSensor()
{
    Deinit();
}

bool ImuHALSensor::Init(const char* i2c_dev, const char* gpio_chip,
                         unsigned int gpio_line, int op_mode)
{
    if (m_handle) {
        return true; /* 已初始化 */
    }

    /* 1. 创建实例 */
    m_handle = (struct emd_gaf*)emd_gaf_create();
    if (!m_handle) {
        fprintf(stderr, "[ImuHALSensor] emd_gaf_create() failed\n");
        return false;
    }

    /* 2. 初始化 (打开 I2C, 配置 GPIO, 载入 eDMP) */
    int ret = emd_gaf_init(m_handle, i2c_dev, gpio_chip, gpio_line, op_mode);
    if (ret != 0) {
        fprintf(stderr, "[ImuHALSensor] emd_gaf_init(%s, %s:%u, mode=%d) failed: %d\n",
                i2c_dev, gpio_chip, gpio_line, op_mode, ret);
        emd_gaf_destroy((emd_gaf_t*)m_handle);
        m_handle = nullptr;
        return false;
    }

    /* 3. 启动后台采集线程 */
    ret = emd_gaf_start((emd_gaf_t*)m_handle);
    if (ret != 0) {
        fprintf(stderr, "[ImuHALSensor] emd_gaf_start() failed: %d\n", ret);
        emd_gaf_destroy((emd_gaf_t*)m_handle);
        m_handle = nullptr;
        return false;
    }

    printf("[ImuHALSensor] initialized: %s %s:%u mode=%d\n",
           i2c_dev, gpio_chip, gpio_line, op_mode);
    return true;
}

void ImuHALSensor::Deinit()
{
    if (!m_handle) return;

    emd_gaf_stop((emd_gaf_t*)m_handle);
    emd_gaf_destroy((emd_gaf_t*)m_handle);
    m_handle = nullptr;

    printf("[ImuHALSensor] deinit done\n");
}

void ImuHALSensor::Read(imu_data_t* out) const
{
    if (!out) return;

    memset(out, 0, sizeof(imu_data_t));

    if (!m_handle) return;

    emd_output_t imu;
    int ret = emd_gaf_get_output((emd_gaf_t*)m_handle, &imu);

    if (ret != 0) {
        return; /* 无新数据，保持全零 */
    }

    /* 原始传感器数据 (来自 eDMP 校准输出) */
    out->acc_x     = imu.accel_x;
    out->acc_y     = imu.accel_y;
    out->acc_z     = imu.accel_z;
    out->gyro_x    = imu.gyro_x;
    out->gyro_y    = imu.gyro_y;
    out->gyro_z    = imu.gyro_z;

    /* 9轴融合输出 */
    out->quat_w      = imu.quat_w;
    out->quat_x      = imu.quat_x;
    out->quat_y      = imu.quat_y;
    out->quat_z      = imu.quat_z;
    out->mag_x       = imu.mag_x;
    out->mag_y       = imu.mag_y;
    out->mag_z       = imu.mag_z;
    out->heading_deg = imu.heading_deg;
    out->temp_c      = imu.temp_c;
    out->stationary  = imu.stationary;
    out->gyr_accuracy = imu.gyr_accuracy;
    out->mag_accuracy = imu.mag_accuracy;
    out->timestamp_us = imu.timestamp_us;
}

} /* namespace stark_periph_manager_node */
