/**
 * @file    vl53l1x.c
 * @brief   VL53L1X ToF传感器封装层 - 多实例版 (4个ToF, 4组软件I2C)
 *          基于ST官方API, 支持多通道软件I2C, 含调试输出
 */
#include "vl53l1x.h"
#include "vl53l1_api.h"
#include "usart.h"
#include "sw_i2c.h"
#include "main.h"
#include <string.h>

/* 全局变量: 4个ToF实例 */
VL53L1_Dev_t tof_dev[TOF_COUNT];
VL53L1_RangingMeasurementData_t tof_result[TOF_COUNT];
int32_t tof_distance_mm[TOF_COUNT] = {0};

/* 模式参数表 (来自参考工程) */
static const mode_data Mode_data[] = {
    {(FixPoint1616_t)(16384),  (FixPoint1616_t)(1179648), 33000,  14, 10},
    {(FixPoint1616_t)(16384),  (FixPoint1616_t)(1179648), 200000, 14, 10},
    {(FixPoint1616_t)(6554),   (FixPoint1616_t)(3932160), 33000,  18, 14},
    {(FixPoint1616_t)(16384),  (FixPoint1616_t)(2097152), 20000,  14, 10},
};

uint8_t VL53L1X_Init(uint8_t index, SW_I2C_Channel_t *ch, VL53L1X_DistanceMode mode)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    VL53L1_Dev_t *dev = &tof_dev[index];

    dev->i2c_channel = ch;
    dev->I2cDevAddr = 0x52;
    dev->comms_type = 1;
    dev->comms_speed_khz = 400;

    /* ===== 步骤0: I2C总线软复位 (替代XSHUT硬件复位) ===== */
    /* 发送9个SCL脉冲+Stop, 复可能卡死的从机状态 */
    debug_printf("[TOF%d] I2C soft reset...\r\n", index);
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(ch->scl_port, ch->scl_pin, GPIO_PIN_RESET);
        SW_I2C_DelayUs(5);
        HAL_GPIO_WritePin(ch->scl_port, ch->scl_pin, GPIO_PIN_SET);
        SW_I2C_DelayUs(5);
    }
    /* 发送Stop条件确保总线空闲 */
    HAL_GPIO_WritePin(ch->sda_port, ch->sda_pin, GPIO_PIN_RESET);
    SW_I2C_DelayUs(5);
    HAL_GPIO_WritePin(ch->scl_port, ch->scl_pin, GPIO_PIN_SET);
    SW_I2C_DelayUs(5);
    HAL_GPIO_WritePin(ch->sda_port, ch->sda_pin, GPIO_PIN_SET);
    SW_I2C_DelayUs(10);

    /* 额外延时等待传感器稳定 (参考工程XSHUT复位后等100ms) */
    HAL_Delay(100);
    debug_printf("[TOF%d] Soft reset done\r\n", index);

    Status = VL53L1_WaitDeviceBooted(dev);
    debug_printf("[TOF%d] WaitBoot=%d\r\n", index, Status);
    if (Status != VL53L1_ERROR_NONE) return 1;
    HAL_Delay(2);

    Status = VL53L1_DataInit(dev);
    debug_printf("[TOF%d] DataInit=%d\r\n", index, Status);
    if (Status != VL53L1_ERROR_NONE) return 2;
    HAL_Delay(2);

    Status = VL53L1_StaticInit(dev);
    debug_printf("[TOF%d] StaticInit=%d\r\n", index, Status);
    if (Status != VL53L1_ERROR_NONE) return 3;
    HAL_Delay(2);

    {
        VL53L1_CalibrationData_t CaliData;
        VL53L1_GetCalibrationData(dev, &CaliData);
        VL53L1_SetCalibrationData(dev, &CaliData);
    }
    HAL_Delay(2);

    VL53L1_SetXTalkCompensationEnable(dev, 1);
    HAL_Delay(2);

    {
        VL53L1_DistanceModes dist_mode;
        switch (mode)
        {
            case VL53L1X_SHORT_DISTANCE:  dist_mode = VL53L1_DISTANCEMODE_SHORT;  break;
            case VL53L1X_MEDIUM_DISTANCE: dist_mode = VL53L1_DISTANCEMODE_MEDIUM; break;
            default:
            case VL53L1X_LONG_DISTANCE:   dist_mode = VL53L1_DISTANCEMODE_LONG;   break;
        }
        Status = VL53L1_SetDistanceMode(dev, dist_mode);
        debug_printf("[TOF%d] SetDistMode=%d\r\n", index, Status);
        if (Status != VL53L1_ERROR_NONE) return 4;
    }
    HAL_Delay(2);

    Status = VL53L1_SetLimitCheckEnable(dev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (Status != VL53L1_ERROR_NONE) { debug_printf("[TOF%d] SigmaEn=%d\r\n", index, Status); return 5; }

    Status = VL53L1_SetLimitCheckEnable(dev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (Status != VL53L1_ERROR_NONE) { debug_printf("[TOF%d] SigRateEn=%d\r\n", index, Status); return 6; }

    Status = VL53L1_SetLimitCheckValue(dev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        Mode_data[DEFAULT_MODE].sigmaLimit);
    if (Status != VL53L1_ERROR_NONE) { debug_printf("[TOF%d] SigmaVal=%d\r\n", index, Status); return 7; }

    Status = VL53L1_SetLimitCheckValue(dev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                        Mode_data[DEFAULT_MODE].signalLimit);
    if (Status != VL53L1_ERROR_NONE) { debug_printf("[TOF%d] SigRateVal=%d\r\n", index, Status); return 8; }

    Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 20000);
    debug_printf("[TOF%d] TimingBudget=%d\r\n", index, Status);
    if (Status != VL53L1_ERROR_NONE) return 9;

    Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 30);
    debug_printf("[TOF%d] InterPeriod=%d\r\n", index, Status);
    if (Status != VL53L1_ERROR_NONE) return 10;

    Status = VL53L1_StartMeasurement(dev);
    debug_printf("[TOF%d] StartMeas=%d\r\n", index, Status);
    if (Status != VL53L1_ERROR_NONE) return 11;

    return 0;
}

int16_t VL53L1X_GetDistance(uint8_t index)
{
    VL53L1_Error Status;
    VL53L1_Dev_t *dev = &tof_dev[index];

    Status = VL53L1_WaitMeasurementDataReady(dev);
    if (Status != VL53L1_ERROR_NONE) return -1;

    Status = VL53L1_GetRangingMeasurementData(dev, &tof_result[index]);
    if (Status != VL53L1_ERROR_NONE) return -2;

    tof_distance_mm[index] = tof_result[index].RangeMilliMeter;

    VL53L1_ClearInterruptAndStartMeasurement(dev);

    return (int16_t)tof_distance_mm[index];
}

void VL53L1X_I2C_Scan(SW_I2C_Channel_t *ch)
{
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        uint8_t dummy = 0;
        SW_I2C_Write(ch, addr << 1, &dummy, 0);
    }
}
