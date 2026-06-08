#include "imu.h"

#define IMU_FRAME_LEN   14

uint8_t imu_rx_buf[IMU_FRAME_LEN];

float   imu_roll  = 0.0f;
float   imu_pitch = 0.0f;
float   imu_yaw   = 0.0f;
float   imu_yaw_filtered = 0.0f;
uint8_t imu_ok    = 0;

#define IMU_YAW_ALPHA   0.9f    /* 滤波系数, 越小越平滑 */

/*
 * 配置 IMU 只订阅欧拉角
 * 帧: FA FB 03 1F 04 23 FC FD
 */
static void imu_config_euler(void)
{
    uint8_t cmd[8] = {
        0xFA, 0xFB,
        0x03, 0x1F,
        0x04, 0x23,
        0xFC, 0xFD
    };

    HAL_UART_Transmit(&IMU_UART, cmd, 8, 50);
}

static void imu_restart_dma(void)
{
    HAL_UART_DMAStop(&IMU_UART);
    for (uint8_t i = 0; i < IMU_FRAME_LEN; i++)
        imu_rx_buf[i] = 0;
    HAL_UART_Receive_DMA(&IMU_UART, imu_rx_buf, IMU_FRAME_LEN);
}

void imu_task(void *parameter)
{
    imu_config_euler();
    vTaskDelay(50);

    imu_restart_dma();

    while (1)
    {
        if (imu_rx_buf[0] == 0xFA && imu_rx_buf[1] == 0xFB &&
            imu_rx_buf[12] == 0xFC && imu_rx_buf[13] == 0xFD)
        {
            int16_t roll_raw  = (int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[5]);
            int16_t pitch_raw = (int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[7]);
            int16_t yaw_raw   = (int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[9]);

            imu_roll  = roll_raw  * 0.0054931640625f;
            imu_pitch = pitch_raw * 0.0054931640625f;
            imu_yaw   = yaw_raw   * 0.0054931640625f;

            /* 一阶低通滤波 */
            imu_yaw_filtered = IMU_YAW_ALPHA * imu_yaw +
                                (1.0f - IMU_YAW_ALPHA) * imu_yaw_filtered;

            imu_ok    = 1;
        }
        else
        {
            imu_ok = 0;
            imu_restart_dma();
        }

        vTaskDelay(50);
    }
}
