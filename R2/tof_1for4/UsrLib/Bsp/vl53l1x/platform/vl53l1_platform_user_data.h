/**
 * @file    vl53l1_platform_user_data.h
 * @brief   VL53L1X平台用户数据定义 (多通道软件I2C版)
 *          去掉I2C_HandleTypeDef指针, 增加SW_I2C_Channel_t指针
 */
#ifndef _VL53L1_PLATFORM_USER_DATA_H_
#define _VL53L1_PLATFORM_USER_DATA_H_

#include "stm32f1xx_hal.h"
#include "vl53l1_def.h"
#include "sw_i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {

	VL53L1_DevData_t   Data;

	uint8_t   I2cDevAddr;       /* 8位I2C设备地址 (0x52=写, 0x53=读) */
	uint8_t   comms_type;       /* 通信类型: 1=I2C */
	uint16_t  comms_speed_khz;  /* 通信速度(软件I2C此字段仅作参考) */
	uint32_t  new_data_ready_poll_duration_ms;

	/* 多通道软件I2C: 指向对应的I2C通道 */
	SW_I2C_Channel_t *i2c_channel;

} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

#define VL53L1DevDataGet(Dev, field) (Dev->Data.field)
#define VL53L1DevDataSet(Dev, field, VL53L1_PRM_00005) ((Dev->Data.field) = (VL53L1_PRM_00005))
#define VL53L1DevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)
#define VL53L1DevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)

#ifdef __cplusplus
}
#endif
#endif
