#include "wit_motion.h"

float wit_roll  = 0.0f;
float wit_pitch = 0.0f;
float wit_yaw   = 0.0f;
volatile bool wit_updated = false;

static uint8_t  wit_buf[WIT_PKT_LEN];
static uint8_t  wit_byte;
static uint8_t  wit_sta = 0;   /* 0=找帧头, 1=接收剩余10字节 */

void wit_init(void)
{
    wit_sta = 0;
    HAL_UART_Receive_IT(&WIT_UART, &wit_byte, 1);   /* 先收 1 字节找 0x55 */
}

/**
 * @brief  解析 WT901C 角度包 (11 字节)
 * @note   buf[0]=0x55, buf[1]=0x53(角度), buf[2..7]=Roll/Pitch/Yaw int16,
 *         角度 = ((H<<8)|L) / 32768.0f * 180.0f
 */
static void wit_parse(void)
{
    if (wit_buf[0] != 0x55 || wit_buf[1] != 0x53)
        return;

    int16_t raw;
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum += wit_buf[i];

    if (sum != wit_buf[10])
        return;                              /* 校验和不匹配 */

    raw   = (int16_t)((wit_buf[3] << 8) | wit_buf[2]);
    wit_roll  = raw / 32768.0f * 180.0f;

    raw   = (int16_t)((wit_buf[5] << 8) | wit_buf[4]);
    wit_pitch = raw / 32768.0f * 180.0f;

    raw   = (int16_t)((wit_buf[7] << 8) | wit_buf[6]);
    wit_yaw   = raw / 32768.0f * 180.0f;

    wit_updated = true;
}

/**
 * @brief  USART6 接收回调 — 状态机接收 11 字节 IMU 包
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART8)
        return;

    if (wit_sta == 0)
    {
        if (wit_byte == 0x55)
        {
            wit_buf[0] = 0x55;
            wit_sta = 1;
            HAL_UART_Receive_IT(&WIT_UART, wit_buf + 1, 10);  /* 收剩余 10 字节 */
        }
        else
        {
            HAL_UART_Receive_IT(&WIT_UART, &wit_byte, 1);       /* 继续找帧头 */
        }
    }
    else
    {
        wit_parse();
        wit_sta = 0;
        HAL_UART_Receive_IT(&WIT_UART, &wit_byte, 1);           /* 继续找下一帧 */
    }
}
