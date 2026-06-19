/**
 * @file    sw_i2c.h
 * @brief   多通道软件模拟I2C驱动 (GPIO位操作)
 *          支持4组独立I2C总线, 每组有独立的SDA/SCL引脚
 *          延时: DWT周期计数器 (不修改SysTick, 兼容FreeRTOS)
 */
#ifndef __SW_I2C_H
#define __SW_I2C_H

#include "stm32f1xx_hal.h"

/* ===== 多通道I2C结构体 ===== */

typedef struct {
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;
} SW_I2C_Channel_t;

/* 全局4个通道实例 (在sw_i2c.c中定义) */
extern SW_I2C_Channel_t sw_i2c_ch[4];

/* 通道编号枚举 */
typedef enum {
    SW_I2C_CH1 = 0,   /* PB10(SDA)/PB11(SCL) - front_front */
    SW_I2C_CH2 = 1,   /* PB0(SDA)/PB1(SCL)   - front_back  */
    SW_I2C_CH3 = 2,   /* PA6(SDA)/PA7(SCL)   - back_front  */
    SW_I2C_CH4 = 3,   /* PA4(SDA)/PA5(SCL)   - back_back   */
} SW_I2C_ChannelNum_t;

/* ===== 初始化 ===== */
void SW_I2C_Init(void);         /* 初始化DWT延时 + 4个通道GPIO映射 */

/* ===== 底层I2C时序 (需指定通道) ===== */
void SW_I2C_Start(SW_I2C_Channel_t *ch);
void SW_I2C_Stop(SW_I2C_Channel_t *ch);
uint8_t SW_I2C_WaitACK(SW_I2C_Channel_t *ch);   /* 返回0=ACK, 1=NACK */
void SW_I2C_SendACK(SW_I2C_Channel_t *ch);
void SW_I2C_SendNACK(SW_I2C_Channel_t *ch);

void SW_I2C_WriteByte(SW_I2C_Channel_t *ch, uint8_t byte);
uint8_t SW_I2C_ReadByte(SW_I2C_Channel_t *ch, uint8_t ack);

/* ===== 高层接口: 16位寄存器地址读写 (VL53L1X专用) ===== */
HAL_StatusTypeDef SW_I2C_Write(SW_I2C_Channel_t *ch, uint8_t dev_addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef SW_I2C_Read(SW_I2C_Channel_t *ch, uint8_t dev_addr, uint8_t *cmd, uint16_t cmd_len, uint8_t *buf, uint16_t buf_len);

/* ===== 工具函数 ===== */
void SW_I2C_DelayUs(uint32_t us);

#endif /* __SW_I2C_H */
