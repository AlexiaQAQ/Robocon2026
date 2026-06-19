/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : 4组软件I2C读取4个VL53L1X + 阻塞式UART发送HEX协议
  *                      含调试输出
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "sw_i2c.h"
#include "vl53l1x.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* HEX协议帧: [0xCC][d1_h][d1_l][d2_h][d2_l][d3_h][d3_l][d4_h][d4_l][0xEE] */
static uint8_t hex_frame[10];

/* USER CODE END Variables */

/* Definitions for AppTask */
osThreadId_t appTaskHandle;
const osThreadAttr_t appTask_attributes = {
  .name = "AppTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
void AppTaskEntry(void *argument);

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void) {
  appTaskHandle = osThreadNew(AppTaskEntry, NULL, &appTask_attributes);
}

/**
  * @brief  组装HEX帧并通过阻塞式UART发送
  *         协议: [0xCC][ff_h][ff_l][fb_h][fb_l][bf_h][bf_l][bb_h][bb_l][0xEE]
  *         大端序: 高位在前, 低位在后
  */
static void send_hex_frame(uint16_t ff, uint16_t fb, uint16_t bf, uint16_t bb)
{
    hex_frame[0] = 0xCC;
    hex_frame[1] = (uint8_t)(ff >> 8);
    hex_frame[2] = (uint8_t)(ff & 0xFF);
    hex_frame[3] = (uint8_t)(fb >> 8);
    hex_frame[4] = (uint8_t)(fb & 0xFF);
    hex_frame[5] = (uint8_t)(bf >> 8);
    hex_frame[6] = (uint8_t)(bf & 0xFF);
    hex_frame[7] = (uint8_t)(bb >> 8);
    hex_frame[8] = (uint8_t)(bb & 0xFF);
    hex_frame[9] = 0xEE;

    HAL_UART_Transmit_DMA(&huart1, hex_frame, 10);
}

/**
  * @brief  AppTask: 4个ToF初始化 → 连续读取4个距离 → HEX帧发送 + LED心跳
  */
void AppTaskEntry(void *argument)
{
  /* 等待传感器上电稳定 */
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* 初始化软件I2C (DWT延时) */
  SW_I2C_Init();

  /* 初始化4个VL53L1X */
  uint8_t init_ok[TOF_COUNT] = {0};
  for (int i = 0; i < TOF_COUNT; i++)
  {
    init_ok[i] = VL53L1X_Init(i, &sw_i2c_ch[i], VL53L1X_LONG_DISTANCE);
    debug_printf("[TOF%d] Init %s (err=%d)\r\n", i, init_ok[i] ? "FAIL" : "OK", init_ok[i]);
  }

  /* 主循环: 连续读取4个ToF, 发送HEX帧 */
//  int loop_count = 0;
  for(;;)
  {
    uint16_t dist[TOF_COUNT] = {0};
    for (int i = 0; i < TOF_COUNT; i++)
    {
      if (init_ok[i] == 0)
      {
        int16_t d = VL53L1X_GetDistance(i);
        dist[i] = (d >= 0) ? (uint16_t)d : 0;
      }
    }
    send_hex_frame(dist[0], dist[1], dist[2], dist[3]);

    /* LED心跳 */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* 让出CPU给FreeRTOS调度器 */
    //vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
