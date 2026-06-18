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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "sbus_set.h"
#include "bsp_can.h"
#include "dm_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SBUS_TIMEOUT_MS 100

/* 抬升电机: 4x DM4340, CAN1, 位置-速度模式 */
#define LIFT_CAN      hcan1
#define LIFT_SPEED    4.0f
#define LIFT_RETURN_SPEED  2.0f

/* 双机械臂: 2x2 电机, CAN2, 位置模式 */
#define ARM_CAN       hcan2
/* 左臂: 根部 4340(ID1) + 末端 4310(ID2) | 右臂: 根部 4340(ID3) + 末端 4310(ID4) */

/* 机械臂关节限幅 (rad), 0=向前伸直 */
#define ARM_L_ROOT_MIN    -1.68f    /* 左根: 向上为负, 竖着稍微往后 */
#define ARM_L_ROOT_MAX     0.0f
#define ARM_L_TIP_MIN     -1.57f    /* 左末: 向上为正, 朝地面 */
#define ARM_L_TIP_MAX      0.0f
#define ARM_R_ROOT_MIN     0.0f     /* 右根: 向上为正, 竖着稍微往后 */
#define ARM_R_ROOT_MAX     1.68f
#define ARM_R_TIP_MIN      0.0f     /* 右末: 向上为负, 朝地面 */
#define ARM_R_TIP_MAX      1.57f
#define ARM_SPEED    0.5f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t led_buf = 0x0001;
static uint32_t last_sbus_tick = 0;
static bool last_ch4 = false;
static bool sys_enabled = false;
static float lift_target = 0.2f;          /* 抬升目标高度, CH6 选择 */
static motor_t lift_motor[4];
static motor_t arm_motor[4];   /* [0]左根4340 [1]左末4310 [2]右根4340 [3]右末4310 */
static float arm_root = 0.0f;         /* 机械臂根部目标角度 */
static float arm_tip  = 0.0f;         /* 机械臂末端目标角度 */
static bool  arm_left = true;         /* true=左臂, false=右臂 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void lift_enable(void);
void lift_disable(void);
void arm_enable(void);
void arm_disable(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------- SBUS helper functions ------- */

static inline bool sbus_connected(void)
{
	return (xTaskGetTickCount() - last_sbus_tick) < pdMS_TO_TICKS(SBUS_TIMEOUT_MS);
}

static inline bool ch_high(int ch)
{
	return sbus_ch.ch[ch] > 1300;
}

static inline bool ch_low(int ch)
{
	return sbus_ch.ch[ch] < 650;
}

static inline bool ch_mid(int ch)
{
	return sbus_ch.ch[ch] >= 650 && sbus_ch.ch[ch] <= 1300;
}

/* ------- 系统使能 / 锁车 ------- */

static void system_enable_handler(void)
{
	bool ch4 = ch_high(4);

	if(ch4 && !last_ch4)
	{
		lift_enable();
		arm_enable();
		sys_enabled = true;
	}
	else if(!ch4 && last_ch4)
	{
		sys_enabled = false;
		lift_disable();
		arm_disable();
	}

	last_ch4 = ch4;
}

/* ------- 抬升电机控制 ------- */

void lift_init(void)
{
	dm_init(&lift_motor[0], 1, DM_MODE_POS, DM_4310);
	dm_init(&lift_motor[1], 2, DM_MODE_POS, DM_4310);
	dm_init(&lift_motor[2], 3, DM_MODE_POS, DM_4310);
	dm_init(&lift_motor[3], 4, DM_MODE_POS, DM_4310);
}

void lift_enable(void)
{
	dm_enable(&LIFT_CAN, &lift_motor[0]);	vTaskDelay(2);
	dm_enable(&LIFT_CAN, &lift_motor[1]);	vTaskDelay(2);
	dm_enable(&LIFT_CAN, &lift_motor[2]);	vTaskDelay(2);
	dm_enable(&LIFT_CAN, &lift_motor[3]);	vTaskDelay(2);
}

void lift_disable(void)
{
	dm_disable(&LIFT_CAN, &lift_motor[0]);	vTaskDelay(2);
	dm_disable(&LIFT_CAN, &lift_motor[1]);	vTaskDelay(2);
	dm_disable(&LIFT_CAN, &lift_motor[2]);	vTaskDelay(2);
	dm_disable(&LIFT_CAN, &lift_motor[3]);	vTaskDelay(2);
}

/* ------- 机械臂电机控制 ------- */

void arm_init(void)
{
	dm_init(&arm_motor[0], 1, DM_MODE_POS, DM_4340);   /* 左臂根部 */
	dm_init(&arm_motor[1], 2, DM_MODE_POS, DM_4310);   /* 左臂末端 */
	dm_init(&arm_motor[2], 3, DM_MODE_POS, DM_4340);   /* 右臂根部 */
	dm_init(&arm_motor[3], 4, DM_MODE_POS, DM_4310);   /* 右臂末端 */
}

void arm_enable(void)
{
	dm_enable(&ARM_CAN, &arm_motor[0]);	vTaskDelay(2);
	dm_enable(&ARM_CAN, &arm_motor[1]);	vTaskDelay(2);
	dm_enable(&ARM_CAN, &arm_motor[2]);	vTaskDelay(2);
	dm_enable(&ARM_CAN, &arm_motor[3]);	vTaskDelay(2);
}

void arm_disable(void)
{
	dm_disable(&ARM_CAN, &arm_motor[0]);	vTaskDelay(2);
	dm_disable(&ARM_CAN, &arm_motor[1]);	vTaskDelay(2);
	dm_disable(&ARM_CAN, &arm_motor[2]);	vTaskDelay(2);
	dm_disable(&ARM_CAN, &arm_motor[3]);	vTaskDelay(2);
}

/* ------- Tasks ------- */

void led_task(void *parameter)
{
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOE, led_buf);
		if(led_buf < 0x0100) { led_buf <<= 1; }
		else { led_buf = 0x0001; }

		if(sys_enabled)
			vTaskDelay(200);
		else
			vTaskDelay(100);
	}
}

void sbus_task(void *parameter)
{
	while(1)
	{
		if(sbus_frame_ready)
		{
			sbus_frame_ready = false;
			last_sbus_tick = xTaskGetTickCount();

			system_enable_handler();

			/* CH5 下拨 → 回零, 中位 → CH6 三段选高度 */
			if(ch_low(5))
				lift_target = 0.2f;
			else if(ch_mid(5))
			{
				if(ch_high(6))
					lift_target = 29.3f;
				else if(ch_low(6))
					lift_target = 19.0f;
				else
					lift_target = 28.8f;
			}

			/* CH11 选臂: >1000左臂, <1000右臂 */
			arm_left = (sbus_ch.ch[11] > 1000);

			if(ch_low(7))
			{
				/* CH7 拨下 → 机械臂放下 */
				if(arm_left)
				{
					arm_root = -1.68f;
					arm_tip  = ch_high(6) ? 0.0f : 1.57f;  /* 3层末端向前, 1~2层朝地 */
				}
				else
				{
					arm_root = 1.68f;
					arm_tip  = ch_high(6) ? 0.0f : -1.57f;
				}
			}
			else
			{
				/* CH7 未拨下 → 机械臂抬起回零 */
				arm_root = 0.0f;
				arm_tip  = 0.0f;
			}
		}
		else if(!sbus_connected())
		{
			/* 超时断联 → 强制锁车 */
			if(sys_enabled)
			{
				sys_enabled = false;
				lift_disable();
				last_ch4 = false;
			}
		}

		vTaskDelay(4);
	}
}

/* 抬升控制任务: 50Hz 发送位置-速度指令 */
void lift_task(void *parameter)
{
	while(1)
	{
		if(sys_enabled)
		{
			float speed = (lift_target == 0.2f) ? LIFT_RETURN_SPEED : LIFT_SPEED;
			dm_pos_ctrl(&LIFT_CAN, 1, -lift_target, speed);	vTaskDelay(2);
			dm_pos_ctrl(&LIFT_CAN, 2,  lift_target, speed);	vTaskDelay(2);
			dm_pos_ctrl(&LIFT_CAN, 3, -lift_target, speed); vTaskDelay(2);
			dm_pos_ctrl(&LIFT_CAN, 4,  lift_target, speed);	vTaskDelay(2);
		}
		vTaskDelay(20);
	}
}

/* 机械臂控制任务: 50Hz */
void arm_task(void *parameter)
{
	while(1)
	{
		if(sys_enabled)
		{
			if(arm_left)
			{
				dm_pos_ctrl(&ARM_CAN, 1,  arm_root, ARM_SPEED);	vTaskDelay(2);
				dm_pos_ctrl(&ARM_CAN, 2,  arm_tip,  ARM_SPEED);
			}
			else
			{
				dm_pos_ctrl(&ARM_CAN, 3,  arm_root, ARM_SPEED);	vTaskDelay(2);
				dm_pos_ctrl(&ARM_CAN, 4, -arm_tip,  ARM_SPEED);
			}
		}
		vTaskDelay(20);
	}
}

void start_task(void *parameter)
{
	while(1)
	{
		sbus_rx_init();
		can_filter_init();
		lift_init();
		arm_init();

		xTaskCreate(led_task, "led_task", 56, NULL, 0, NULL);
		xTaskCreate(sbus_task, "remote_task", 512, NULL, 0, NULL);
		xTaskCreate(lift_task, "lift_task", 512, NULL, 0, NULL);
		xTaskCreate(arm_task, "arm_task", 256, NULL, 0, NULL);

		vTaskDelete(NULL);
	}
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
