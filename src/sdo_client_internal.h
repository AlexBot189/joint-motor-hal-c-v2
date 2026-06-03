/**
 * @file sdo_client_internal.h
 * @brief SDO 客户端 - 内部头文件
 */
#ifndef SDO_CLIENT_INTERNAL_H
#define SDO_CLIENT_INTERNAL_H

#include "motor_hal_types.h"
#include "can_driver_internal.h"

int sdo_write(can_driver_t *drv, uint8_t node,
              uint16_t index, uint8_t subidx,
              uint32_t value, uint8_t size_bytes,
              int retry_count, int timeout_ms);

int sdo_read(can_driver_t *drv, uint8_t node,
             uint16_t index, uint8_t subidx,
             uint32_t *value,
             int retry_count, int timeout_ms);

/* 简化接口 (3次重试, 200ms超时) */
int sdo_write_simple(can_driver_t *drv, uint8_t node,
                     uint16_t index, uint8_t subidx,
                     uint32_t value, uint8_t size_bytes);

int sdo_read_simple(can_driver_t *drv, uint8_t node,
                    uint16_t index, uint8_t subidx,
                    uint32_t *value);

#endif /* SDO_CLIENT_INTERNAL_H */
