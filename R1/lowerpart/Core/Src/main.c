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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "bsp_can.h"
#include "sbus_set.h"
#include "dm_motor.h"
#include "chassis.h"
#include "wit_motion.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define YAW_HOLD_ENABLE   /* 取消注释 → 开启航向角保持 */

#define led1(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,(GPIO_PinState)dat)
#define led2(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,(GPIO_PinState)dat)
#define led3(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,(GPIO_PinState)dat)
#define led4(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,(GPIO_PinState)dat)
#define led5(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,(GPIO_PinState)dat)
#define led6(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,(GPIO_PinState)dat)
#define led7(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,(GPIO_PinState)dat)
#define led8(dat) HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,(GPIO_PinState)dat)

#define xt1(dat) HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,(GPIO_PinState)dat)
#define xt2(dat) HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,(GPIO_PinState)dat)
#define xt3(dat) HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,(GPIO_PinState)dat)
#define xt4(dat) HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,(GPIO_PinState)dat)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t led_buf = 0x0001;

#define SBUS_TIMEOUT_MS 200
static uint32_t last_sbus_tick = 0;

float set_vx = 0.0f;
float set_vy = 0.0f;
float set_vw = 0.0f;

bool sys_enabled = false;
static bool last_ch4 = false;

/* ---- 航向角 PID 参数 (Keil Watch 窗口可实时修改) ---- */
float yaw_gain[3]  = {0.65f, 0.0f, 0.06f};  /* Kp, Ki, Kd */
float yaw_max_out  = 23.0f;                /* PID 输出限幅 (rad/s) */
float yaw_max_iout = 3.0f;                 /* 积分限幅 */
float yaw_target   = 0.0f;                 /* 目标航向角 (°) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline bool sbus_frame_valid(void)
{
	return (sbus_rx_buf[23] == 0x00 && sbus_rx_buf[0] == 0x0f && sbus_rx_buf[24] == 0x00);
}

static inline bool sbus_connected(void)
{
	return (xTaskGetTickCount() - last_sbus_tick) < pdMS_TO_TICKS(SBUS_TIMEOUT_MS);
}

/* 摇杆线性映射 */
static float expo_map(int16_t val, int16_t lo, int16_t hi, int16_t dead, float max_out)
{
    int16_t center = (lo + hi) / 2;
    if (val >= center - dead && val <= center + dead) return 0;
    float t = (float)(abs(val - center) - dead) / (float)((hi - lo) / 2 - dead);
    if (t > 1.0f) t = 1.0f;
    return (val > center ? 1.0f : -1.0f) * t * t * max_out;
}

bool ch_down(int16_t ch)
{
	return sbus_ch.ch[ch] > 1600;
}

static bool ch4_enable(void)
{
	return sbus_ch.ch[4] > 650;   /* 中位或高位 → 使能; 低位 → 锁车 */
}

static void system_enable_handler(void)
{
	bool ch4 = sbus_connected() && ch4_enable();

	if (ch4 && !last_ch4)
	{
		sys_enabled = true;
		chassis_enable();
	}
	else if (!ch4 && last_ch4)
	{
		sys_enabled = false;
		chassis_disable();
	}

	last_ch4 = ch4;
}

void led_task(void *parameter)
{
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOG,led_buf);

		if(led_buf<0x0100){led_buf <<= 1;}
		else{led_buf = 0x0001;}

		vTaskDelay(200);
	}
}

void remote_task(void *parameter)
{
	static PidTypeDef  yaw_pid;
	static bool        yaw_inited   = false;
	static bool        last_enabled = false;

	if (!yaw_inited)
	{
		PID_Init(&yaw_pid, PID_POSITION, yaw_gain, yaw_max_out, yaw_max_iout);
		yaw_inited = true;
	}

	while(1)
	{
		yaw_pid.Kp = yaw_gain[0];
		yaw_pid.Ki = yaw_gain[1];
		yaw_pid.Kd = yaw_gain[2];
		yaw_pid.max_out  = yaw_max_out;
		yaw_pid.max_iout = yaw_max_iout;

		rx_set(&sbus_ch);

		if (sbus_frame_valid())
			last_sbus_tick = xTaskGetTickCount();

		system_enable_handler();

		if (!sys_enabled)
		{
			set_vx = 0; set_vy = 0; set_vw = 0;
			last_enabled = false;
		}

		if (sys_enabled)
		{
			if (!last_enabled)
			{
				yaw_target   = wit_yaw;
				PID_clear(&yaw_pid);
				last_enabled = true;
			}

			int16_t ch_val;

			ch_val = sbus_ch.ch[2];
			set_vx = expo_map(ch_val, 326, 1659, 1, 15.0f);

			ch_val = sbus_ch.ch[3];
			set_vy = expo_map(ch_val, 326, 1659, 1, 15.0f);

			ch_val = sbus_ch.ch[0];
			float stick_vw = expo_map(ch_val, 326, 1659, 2, 10.0f);
			if (stick_vw != 0.0f)
			{
				/* 摇杆偏转 → 开环跟手, 记录目标 */
				yaw_target   = wit_yaw;
				set_vw       = stick_vw;
				PID_clear(&yaw_pid);
			}
			else
			{
				float fdb = wit_yaw;
				while (fdb - yaw_target >  180.0f) fdb -= 360.0f;
				while (fdb - yaw_target < -180.0f) fdb += 360.0f;
				if (fabsf(fdb - yaw_target) < 0.3f)
					set_vw = 0;
				else
					set_vw = -PID_Calc(&yaw_pid, fdb, yaw_target);
			}
		}

		vTaskDelay(10);
	}
}

void chassis_task(void *parameter)
{
	while(1)
	{
		if (sys_enabled)
		{
			chassis_update();
		}
		else
		{
			vTaskDelay(2);
		}
	}
}


void start_task(void *parameter)
{
	while(1)
	{
		xt1(0);
		xt2(0);
		xt3(0);
		xt4(0);

		can_filter_init();
		sbus_rx_init();
		wit_init();
		chassis_init();

		xTaskCreate(led_task,"led_task",128,NULL,0,NULL);
		xTaskCreate(remote_task,"remote_task",512,NULL,0,NULL);
		xTaskCreate(chassis_task,"chassis_task",1024,NULL,0,NULL);

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
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

	if (xTaskCreate(start_task, "start_task", 256, NULL, 0, NULL) != pdPASS)
	{
		while(1);
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
