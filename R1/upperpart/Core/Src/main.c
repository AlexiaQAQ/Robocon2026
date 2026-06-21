/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "cmsis_os.h"
#include "sbus_set.h"
#include "bsp_can.h"
#include "lift.h"
#include "arm.h"
#include "grab.h"
#include "solenoid_valves.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SBUS_TIMEOUT_MS 100
#define LIFT_H_3F   25.0f   /* 抬升 3层 */
#define LIFT_H_2F   28.8f   /* 抬升 2层 (中位) */
#define LIFT_H_1F   19.0f   /* 抬升 1层 */
#define LIFT_H_PLACE 15.0f  /* 放方块高度 */
#define CH1_FINE_MAX   3.0f  /* CH1连续微调最大偏移 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t led_buf = 0x0001;
static uint32_t last_sbus_tick = 0;
static bool     last_ch4 = false;
bool            g_sys_enabled = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------- SBUS helpers ------- */

static inline bool sbus_connected(void)
{
    return (xTaskGetTickCount() - last_sbus_tick) < pdMS_TO_TICKS(SBUS_TIMEOUT_MS);
}
static inline bool ch_high(int ch) { return sbus_ch.ch[ch] > 1300; }
static inline bool ch_low(int ch)  { return sbus_ch.ch[ch] < 650;  }
static inline bool ch_mid(int ch)  { return sbus_ch.ch[ch] >= 650 && sbus_ch.ch[ch] <= 1300; }

/* ------- CH4 锁车/解锁 ------- */

static void system_enable_handler(void)
{
    static bool last_ch4_mid = false;
    bool ch4 = (ch_mid(4) || ch_high(4));

    if(ch4 && !last_ch4)                    /* 上沿 -> 解锁 */
    {
        lift_enable();
        arm_enable();
        grab_enable();
        g_sys_enabled = true;
    }
    else if(!ch4 && last_ch4)               /* 下沿 -> 锁车 */
    {
        g_sys_enabled = false;
        lift_disable();
        arm_disable();
        grab_disable();
    }

    /* CH4 拨入中位 -> 仅一次清零抓取工序 */
    if(ch_mid(4) && !last_ch4_mid)
        grab_reset();
    last_ch4_mid = ch_mid(4);

    last_ch4 = ch4;
}

/* ------- SBUS 遥控调度 ------- */

void sbus_task(void *parameter)
{
    while(1)
    {
        sbus_poll();

        if(sbus_frame_ready)
        {
            sbus_frame_ready = false;

            /* 失控保护 -> 立刻锁车 */
            if(sbus_ch.failsafe)
            {
                if(g_sys_enabled)
                {
                    g_sys_enabled = false;
                    lift_disable();
                    arm_disable();
                    grab_disable();
                }
                continue;
            }

            last_sbus_tick = xTaskGetTickCount();
            system_enable_handler();

            if(ch_low(5))       /* 抓取模式 (含抬升回零/R2对接) */
            {
                lift_update(grab_lift_target());
                grab_update(&sbus_ch);
            }
            else if(ch_mid(5))  /* 抬升高度 + 机械臂 */
            {
                /* CH1 摇杆连续微调抬升 (中位死区±20) */
                float ch1 = (float)sbus_ch.ch[1];
                float offset = 0.0f;
                if      (ch1 > 1012.0f) offset = -Map(ch1, 1012.0f, 1659.0f, 0.0f, CH1_FINE_MAX);
                else if (ch1 < 972.0f)  offset =  Map(ch1,  972.0f,  326.0f, 0.0f, CH1_FINE_MAX);

                float base = ch_high(6) ? LIFT_H_3F :
                             ch_low(6)  ? LIFT_H_1F : LIFT_H_2F;
                lift_update(base + offset);

                bool sel = (sbus_ch.ch[11] > 1000);
                arm_update(&sbus_ch, sel, ch_high(6));
            }
            else if(ch_high(5)) /* 放方块模式 (抬升15 + 机械臂末端朝前) */
            {
                lift_update(LIFT_H_PLACE);
                bool sel = (sbus_ch.ch[11] > 1000);
                arm_update(&sbus_ch, sel, true);  /* ch6_high=true: 末端朝前 */
            }

            /* CH8左吸盘 CH9右吸盘 CH10齿条, 所有模式有效 */
            YV1(sbus_ch.ch[8] > 1300 ? 1 : 0);
            YV2(sbus_ch.ch[9] > 1300 ? 1 : 0);
            grab_update_rack(sbus_ch.ch[10]);
        }
        else if(!sbus_connected())
        {
            if(g_sys_enabled)
            {
                g_sys_enabled = false;
                lift_disable();
                arm_disable();
                grab_disable();
            }
        }

        vTaskDelay(4);
    }
}

/* ------- LED 流水灯 ------- */

void led_task(void *parameter)
{
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOE, led_buf);
        if(led_buf < 0x0100) { led_buf <<= 1; }
        else { led_buf = 0x0001; }
        vTaskDelay(g_sys_enabled ? 200 : 100);
    }
}

/* ------- 启动任务 ------- */

void start_task(void *parameter)
{
    sbus_rx_init();
    can_filter_init();
    lift_init();
    arm_init();
    grab_init();

    xTaskCreate(led_task,  "led_task",   56, NULL, 0, NULL);
    xTaskCreate(sbus_task, "sbus_task", 512, NULL, 0, NULL);
    xTaskCreate(lift_task, "lift_task", 512, NULL, 0, NULL);
    xTaskCreate(arm_task,  "arm_task",  256, NULL, 0, NULL);
    xTaskCreate(grab_task, "grab_task", 256, NULL, 0, NULL);

    vTaskDelete(NULL);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	if (xTaskCreate(start_task, "start_task", 256, NULL, 0, NULL) != pdPASS)
	{
		while (1);
	}

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
