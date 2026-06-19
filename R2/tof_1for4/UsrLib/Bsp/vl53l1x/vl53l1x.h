/**
 * @file    vl53l1x.h
 * @brief   VL53L1X ToF传感器封装层 - 多实例版 (4个ToF, 4组软件I2C)
 *          基于ST官方API, 支持多通道软件I2C
 */
#ifndef __VL53L1X_H__
#define __VL53L1X_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sw_i2c.h"
#include "vl53l1.h"
#include "vl53l1_api.h"

/* VL53L1X I2C设备地址 (8位地址, 所有ToF共用0x29, 每个独占一条I2C总线) */
#define VL53L1X_I2C_ADDR    0x52  /* 0x29 << 1 */

/* ToF数量 */
#define TOF_COUNT           4

/* 测距距离模式 */
typedef enum {
    VL53L1X_SHORT_DISTANCE  = 0,
    VL53L1X_MEDIUM_DISTANCE = 1,
    VL53L1X_LONG_DISTANCE   = 2
} VL53L1X_DistanceMode;

/* 初始化指定通道的ToF (包含完整ST API初始化流程) */
uint8_t VL53L1X_Init(uint8_t index, SW_I2C_Channel_t *ch, VL53L1X_DistanceMode mode);

/* 读取指定通道的距离(mm), <0表示失败 */
int16_t VL53L1X_GetDistance(uint8_t index);

/* 调试: I2C总线扫描 (指定通道) */
void VL53L1X_I2C_Scan(SW_I2C_Channel_t *ch);

/* 全局变量: 4个ToF设备实例 + 4个测距结果 */
extern VL53L1_Dev_t tof_dev[TOF_COUNT];
extern VL53L1_RangingMeasurementData_t tof_result[TOF_COUNT];
extern int32_t tof_distance_mm[TOF_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* __VL53L1X_H__ */
