/**
 * @file    sw_i2c.c
 * @brief   多通道软件模拟I2C驱动实现
 *          支持4组独立I2C总线, 每组有独立的SDA/SCL引脚
 *          延时: DWT周期计数器 (不修改SysTick, 兼容FreeRTOS)
 *
 * 引脚映射 (与CubeMX gpio.c配置一致):
 *   CH1: PB10(SDA)/PB11(SCL) - front_front
 *   CH2: PB0(SDA)/PB1(SCL)   - front_back
 *   CH3: PA6(SDA)/PA7(SCL)   - back_front
 *   CH4: PA4(SDA)/PA5(SCL)   - back_back
 */
#include "sw_i2c.h"
#include "main.h"

/* DWT延时相关 */
static uint32_t dwt_cycles_per_us = 0;

/* 全局4个通道实例 */
SW_I2C_Channel_t sw_i2c_ch[4] = {
    { GPIOB, SDA_1_Pin, GPIOB, SCL_1_Pin },   /* CH1: PB10/PB11 */
    { GPIOB, SDA_2_Pin, GPIOB, SCL_2_Pin },   /* CH2: PB0/PB1   */
    { GPIOA, SDA_3_Pin, GPIOA, SCL_3_Pin },   /* CH3: PA6/PA7   */
    { GPIOA, SDA_4_Pin, GPIOA, SCL_4_Pin },   /* CH4: PA4/PA5   */
};

/* ===== 通道级GPIO操作内联函数 ===== */

static inline void SCL_HIGH(SW_I2C_Channel_t *ch)  { HAL_GPIO_WritePin(ch->scl_port, ch->scl_pin, GPIO_PIN_SET); }
static inline void SCL_LOW(SW_I2C_Channel_t *ch)   { HAL_GPIO_WritePin(ch->scl_port, ch->scl_pin, GPIO_PIN_RESET); }
static inline void SDA_HIGH(SW_I2C_Channel_t *ch)  { HAL_GPIO_WritePin(ch->sda_port, ch->sda_pin, GPIO_PIN_SET); }
static inline void SDA_LOW(SW_I2C_Channel_t *ch)   { HAL_GPIO_WritePin(ch->sda_port, ch->sda_pin, GPIO_PIN_RESET); }
static inline GPIO_PinState SDA_READ(SW_I2C_Channel_t *ch) { return HAL_GPIO_ReadPin(ch->sda_port, ch->sda_pin); }

/**
 * @brief  初始化DWT延时单元 + 4个通道GPIO映射
 *         GPIO初始化由CubeMX gpio.c完成, 这里只初始化DWT
 */
void SW_I2C_Init(void)
{
    /* 使能DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* 清零周期计数器 */
    DWT->CYCCNT = 0;
    /* 使能周期计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 计算每微秒的周期数 (72MHz → 72 cycles/us) */
    dwt_cycles_per_us = SystemCoreClock / 1000000;
}

/**
 * @brief  微秒级延时 (基于DWT周期计数器)
 * @param  us 延时微秒数
 */
void SW_I2C_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * dwt_cycles_per_us;
    while ((DWT->CYCCNT - start) < ticks);
}

/* ===== I2C时序基本操作 (多通道版) ===== */

void SW_I2C_Start(SW_I2C_Channel_t *ch)
{
    SDA_HIGH(ch);
    SCL_HIGH(ch);
    SW_I2C_DelayUs(5);
    SDA_LOW(ch);
    SW_I2C_DelayUs(5);
    SCL_LOW(ch);
    SW_I2C_DelayUs(2);
}

void SW_I2C_Stop(SW_I2C_Channel_t *ch)
{
    SDA_LOW(ch);
    SCL_HIGH(ch);
    SW_I2C_DelayUs(5);
    SDA_HIGH(ch);
    SW_I2C_DelayUs(5);
}

uint8_t SW_I2C_WaitACK(SW_I2C_Channel_t *ch)
{
    uint8_t timeout = 255;

    SDA_HIGH(ch);     /* 释放SDA, 让从机拉低 */
    SCL_HIGH(ch);     /* SCL拉高, 读取SDA */
    SW_I2C_DelayUs(2);

    while (SDA_READ(ch))
    {
        if (--timeout == 0)
        {
            SCL_LOW(ch);
            return 1;   /* NACK */
        }
    }

    SW_I2C_DelayUs(2);
    SCL_LOW(ch);     /* SCL拉低, 完成ACK时钟 */
    SW_I2C_DelayUs(2);

    return 0;   /* ACK */
}

void SW_I2C_SendACK(SW_I2C_Channel_t *ch)
{
    SDA_LOW(ch);
    SW_I2C_DelayUs(2);
    SCL_HIGH(ch);
    SW_I2C_DelayUs(5);
    SCL_LOW(ch);
    SW_I2C_DelayUs(2);
}

void SW_I2C_SendNACK(SW_I2C_Channel_t *ch)
{
    SDA_HIGH(ch);
    SW_I2C_DelayUs(2);
    SCL_HIGH(ch);
    SW_I2C_DelayUs(5);
    SCL_LOW(ch);
    SW_I2C_DelayUs(2);
}

void SW_I2C_WriteByte(SW_I2C_Channel_t *ch, uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & 0x80)
            SDA_HIGH(ch);
        else
            SDA_LOW(ch);

        byte <<= 1;
        SW_I2C_DelayUs(2);
        SCL_HIGH(ch);
        SW_I2C_DelayUs(5);
        SCL_LOW(ch);
        SW_I2C_DelayUs(2);
    }
}

uint8_t SW_I2C_ReadByte(SW_I2C_Channel_t *ch, uint8_t ack)
{
    uint8_t byte = 0;

    SDA_HIGH(ch);  /* 释放SDA */

    for (uint8_t i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCL_HIGH(ch);
        SW_I2C_DelayUs(5);

        if (SDA_READ(ch))
            byte |= 0x01;

        SCL_LOW(ch);
        SW_I2C_DelayUs(2);
    }

    if (ack)
        SW_I2C_SendACK(ch);
    else
        SW_I2C_SendNACK(ch);

    return byte;
}

/* ===== 高层接口 (VL53L1X platform层调用, 多通道版) ===== */

HAL_StatusTypeDef SW_I2C_Write(SW_I2C_Channel_t *ch, uint8_t dev_addr, uint8_t *data, uint16_t len)
{
    SW_I2C_Start(ch);

    /* 发送设备地址 (写) */
    SW_I2C_WriteByte(ch, dev_addr & 0xFE);
    if (SW_I2C_WaitACK(ch) != 0)
    {
        debug_printf("[SW_I2C] addr NACK!\r\n");
        SW_I2C_Stop(ch);
        return HAL_ERROR;
    }

    /* 发送数据 */
    for (uint16_t i = 0; i < len; i++)
    {
        SW_I2C_WriteByte(ch, data[i]);
        if (SW_I2C_WaitACK(ch) != 0)
        {
            debug_printf("[SW_I2C] byte[%u]=0x%02X NACK! total=%u\r\n", i, data[i], len);
            SW_I2C_Stop(ch);
            return HAL_ERROR;
        }
    }

    SW_I2C_Stop(ch);
    return HAL_OK;
}

HAL_StatusTypeDef SW_I2C_Read(SW_I2C_Channel_t *ch, uint8_t dev_addr, uint8_t *cmd, uint16_t cmd_len, uint8_t *buf, uint16_t buf_len)
{
    /* 第一阶段: 写寄存器地址 */
    SW_I2C_Start(ch);

    SW_I2C_WriteByte(ch, dev_addr & 0xFE);
    if (SW_I2C_WaitACK(ch) != 0)
    {
        SW_I2C_Stop(ch);
        return HAL_ERROR;
    }

    for (uint16_t i = 0; i < cmd_len; i++)
    {
        SW_I2C_WriteByte(ch, cmd[i]);
        if (SW_I2C_WaitACK(ch) != 0)
        {
            SW_I2C_Stop(ch);
            return HAL_ERROR;
        }
    }

    /* 第二阶段: 重新启动, 切换读模式 */
    SW_I2C_Start(ch);

    SW_I2C_WriteByte(ch, dev_addr | 0x01);
    if (SW_I2C_WaitACK(ch) != 0)
    {
        SW_I2C_Stop(ch);
        return HAL_ERROR;
    }

    /* 读取数据 */
    for (uint16_t i = 0; i < buf_len; i++)
    {
        buf[i] = SW_I2C_ReadByte(ch, (i < buf_len - 1) ? 1 : 0);
    }

    SW_I2C_Stop(ch);
    return HAL_OK;
}
