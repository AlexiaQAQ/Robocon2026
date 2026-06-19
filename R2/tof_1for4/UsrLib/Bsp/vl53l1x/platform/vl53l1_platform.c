/**
 * @file    vl53l1_platform.c
 * @brief   VL53L1X平台适配层 - 多通道软件I2C版(带调试追踪)
 *          所有读操作使用SW_I2C_Read实现Repeated Start
 *          通过Dev->i2c_channel选择对应的软件I2C通道
 */
#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "sw_i2c.h"
#include "usart.h"
#include <string.h>

/* 全局I2C缓冲区 (256字节, 与ST X-CUBE-53L1A1一致) */
uint8_t _I2CBuffer[256];

/* 调试: I2C操作计数器 */
static uint32_t i2c_op_count = 0;

/* ===== I2C底层写操作 (多通道软件I2C版) ===== */

/**
 * @brief  软件I2C写操作 (多通道, 带调试追踪)
 */
static int _I2CWrite(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count)
{
    int ret = SW_I2C_Write(Dev->i2c_channel, Dev->I2cDevAddr, pdata, (uint16_t)count);
    if (ret != HAL_OK)
        debug_printf("[I2C#%lu] Wr FAIL len=%lu\r\n", i2c_op_count, count);
    i2c_op_count++;
    return ret;
}

/* ===== VL53L1平台接口实现 ===== */

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (count > sizeof(_I2CBuffer) - 2)
        return VL53L1_ERROR_INVALID_PARAMS;

    _I2CBuffer[0] = (uint8_t)(index >> 8);
    _I2CBuffer[1] = (uint8_t)(index & 0xFF);
    memcpy(&_I2CBuffer[2], pdata, count);

    if (_I2CWrite(Dev, _I2CBuffer, count + 2) != HAL_OK) {
        debug_printf("[WM] FAIL idx=0x%04X len=%lu\r\n", index, count);
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }

    return VL53L1_ERROR_NONE;
}

/* ===== 读操作: 全部使用SW_I2C_Read实现Repeated Start ===== */

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t reg_addr[2];
    reg_addr[0] = (uint8_t)(index >> 8);
    reg_addr[1] = (uint8_t)(index & 0xFF);

    /* 使用SW_I2C_Write: 写寄存器地址 + Repeated Start + 读数据 */
    if (SW_I2C_Read(Dev->i2c_channel, Dev->I2cDevAddr, reg_addr, 2, pdata, (uint16_t)count) != HAL_OK) {
        debug_printf("[RM] FAIL idx=0x%04X len=%lu\r\n", index, count);
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    i2c_op_count++;

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
    _I2CBuffer[0] = (uint8_t)(index >> 8);
    _I2CBuffer[1] = (uint8_t)(index & 0xFF);
    _I2CBuffer[2] = data;

    if (_I2CWrite(Dev, _I2CBuffer, 3) != HAL_OK)
        return VL53L1_ERROR_CONTROL_INTERFACE;

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
    _I2CBuffer[0] = (uint8_t)(index >> 8);
    _I2CBuffer[1] = (uint8_t)(index & 0xFF);
    _I2CBuffer[2] = (uint8_t)(data >> 8);
    _I2CBuffer[3] = (uint8_t)(data & 0xFF);

    if (_I2CWrite(Dev, _I2CBuffer, 4) != HAL_OK)
        return VL53L1_ERROR_CONTROL_INTERFACE;

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
    _I2CBuffer[0] = (uint8_t)(index >> 8);
    _I2CBuffer[1] = (uint8_t)(index & 0xFF);
    _I2CBuffer[2] = (uint8_t)((data >> 24) & 0xFF);
    _I2CBuffer[3] = (uint8_t)((data >> 16) & 0xFF);
    _I2CBuffer[4] = (uint8_t)((data >> 8) & 0xFF);
    _I2CBuffer[5] = (uint8_t)(data & 0xFF);

    if (_I2CWrite(Dev, _I2CBuffer, 6) != HAL_OK)
        return VL53L1_ERROR_CONTROL_INTERFACE;

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(Dev, index, &data);
    if (Status != VL53L1_ERROR_NONE) return Status;

    data = (data & AndData) | OrData;
    return VL53L1_WrByte(Dev, index, data);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
    uint8_t reg_addr[2];
    reg_addr[0] = (uint8_t)(index >> 8);
    reg_addr[1] = (uint8_t)(index & 0xFF);

    if (SW_I2C_Read(Dev->i2c_channel, Dev->I2cDevAddr, reg_addr, 2, data, 1) != HAL_OK) {
        debug_printf("[RB] FAIL idx=0x%04X\r\n", index);
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    i2c_op_count++;

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
    uint8_t reg_addr[2];
    uint8_t buf[2];

    reg_addr[0] = (uint8_t)(index >> 8);
    reg_addr[1] = (uint8_t)(index & 0xFF);

    if (SW_I2C_Read(Dev->i2c_channel, Dev->I2cDevAddr, reg_addr, 2, buf, 2) != HAL_OK) {
        debug_printf("[RW] FAIL idx=0x%04X\r\n", index);
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    i2c_op_count++;

    *data = ((uint16_t)buf[0] << 8) + (uint16_t)buf[1];
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
    uint8_t reg_addr[2];
    uint8_t buf[4];

    reg_addr[0] = (uint8_t)(index >> 8);
    reg_addr[1] = (uint8_t)(index & 0xFF);

    if (SW_I2C_Read(Dev->i2c_channel, Dev->I2cDevAddr, reg_addr, 2, buf, 4) != HAL_OK) {
        debug_printf("[RDW] FAIL idx=0x%04X\r\n", index);
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    i2c_op_count++;

    *data = ((uint32_t)buf[0] << 24) + ((uint32_t)buf[1] << 16) +
            ((uint32_t)buf[2] << 8) + (uint32_t)buf[3];
    return VL53L1_ERROR_NONE;
}

/* ===== 系统函数 ===== */

VL53L1_Error VL53L1_CommsInitialise(VL53L1_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz)
{
    (void)pdev; (void)comms_type; (void)comms_speed_khz;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev)
{
    (void)pdev;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
    *ptick_count_ms = 0;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 0;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count)
{
    *ptimer_count = 0;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
    (void)pdev;
    HAL_Delay(wait_ms);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
    (void)pdev;
    HAL_Delay(wait_us / 1000);
    return VL53L1_ERROR_NONE;
}

/* GPIO stubs */
VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode) { (void)pin; (void)mode; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioSetValue(uint8_t pin, uint8_t value) { (void)pin; (void)value; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue) { (void)pin; (void)pvalue; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioXshutdown(uint8_t value) { (void)value; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value) { (void)value; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioPowerEnable(uint8_t value) { (void)value; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type) { (void)function; (void)edge_type; return VL53L1_ERROR_NONE; }
VL53L1_Error VL53L1_GpioInterruptDisable(void) { return VL53L1_ERROR_NONE; }

VL53L1_Error VL53L1_WaitValueMaskEx(
    VL53L1_Dev_t *pdev,
    uint32_t      timeout_ms,
    uint16_t      index,
    uint8_t       value,
    uint8_t       mask,
    uint32_t      poll_delay_ms)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    uint32_t     start_time_ms = 0;
    uint32_t     current_time_ms = 0;
    uint32_t     polling_time_ms = 0;
    uint8_t      byte_value = 0;
    uint8_t      found = 0;

    start_time_ms = HAL_GetTick();

    while ((status == VL53L1_ERROR_NONE) &&
           (polling_time_ms < timeout_ms) &&
           (found == 0))
    {
        status = VL53L1_RdByte(pdev, index, &byte_value);

        if ((byte_value & mask) == value)
            found = 1;

        if (status == VL53L1_ERROR_NONE && found == 0 && poll_delay_ms > 0)
            status = VL53L1_WaitMs(pdev, poll_delay_ms);

        current_time_ms = HAL_GetTick();
        polling_time_ms = current_time_ms - start_time_ms;
    }

    if (found == 0 && status == VL53L1_ERROR_NONE)
        status = VL53L1_ERROR_TIME_OUT;

    return status;
}
