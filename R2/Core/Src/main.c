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
#include "sbus_set.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "mcp2515.h"
#include "chassis.h"
#include "uart_task.h"
#include "solenoid_valves.h"

/* ⚠️ Removed (2026-06-06 mechanical redesign):
   four_steering_wheel_ik.c/h — old 4 steering wheel kinematics
   upstairs.c/h — old YUN motor lifting
   arm.c/h — old single arm (now dual-arm, left+right)
   solenoid_valves.c/h — old 10-way solenoid valves */
/* ✅ New: chassis.c/h — omni wheel kinematics + independent lift control */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 夹爪翻转电机速度 (rad/s) — 模式区分 */
#define GRIPPER_FLIP_VEL_SLOW  0.5f   // 遥控器手动慢速
#define GRIPPER_FLIP_VEL_FAST  2.0f   // 串口自动快速
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t led_buf = 0x0001;

/* Chassis velocity setpoints */
float set_vx = 0.0f;
float set_vy = 0.0f;
float set_vw = 0.0f;

/* SBUS health monitoring */
#define SBUS_TIMEOUT_MS 100
static uint32_t last_sbus_tick = 0;

/* System enable state (centralized CH4 edge detection) */
bool sys_enabled = false;
static bool last_ch4 = false;

/* Gripper flip motor position: 0=竖起来, 1.57=翻下去 */
float gripper_flip_pos = 0.0f;
bool gripper_flip_ready = false;   // 首次CH7操作后置true, 防止上电夹人
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ====================== Helpers ====================== */

static inline bool sbus_frame_valid(void)
{
	return (sbus_rx_buf[23] == 0x00 && sbus_rx_buf[0] == 0x0f && sbus_rx_buf[24] == 0x00);
}

static inline bool sbus_connected(void)
{
	return (xTaskGetTickCount() - last_sbus_tick) < pdMS_TO_TICKS(SBUS_TIMEOUT_MS);
}

bool ch_high(int ch)
{
	return sbus_ch.ch[ch] > 1700;
}

/**
 * Centralized CH4 edge detection with SBUS disconnection protection.
 * Call once per SBUS frame cycle.
 */
static void system_enable_handler(void)
{
	bool ch4 = sbus_connected() && sbus_frame_valid() && ch_high(4);

	if (ch4 && !last_ch4)
	{
		/* CH4 rising edge: enable all subsystems */
		sys_enabled = true;
		chassis_enable(&hcan1);               // CAN1: 全向轮×4 + 抬升×4
		/* TODO: re-add dual-arm enable when new module ready */
		/* TODO: re-add upstairs enable (hcan3/hcan5) when ready */      
		dm_enable_mcp2515(&hcan3, 0x01);   // hcan3: 夹爪翻转电机
	}
	else if (!ch4 && last_ch4)
	{
		/* CH4 falling edge: disable all subsystems & safe stop */
		sys_enabled = false;
		chassis_disable(&hcan1);              // CAN1: 全向轮×4 + 抬升×4
		/* TODO: re-add dual-arm disable when new module ready */
		/* TODO: re-add upstairs disable when ready */      
		dm_disable_mcp2515(&hcan3, 0x01);  // hcan3: 夹爪翻转电机
		set_vx = 0;
		set_vy = 0;
		set_vw = 0;
	}

	last_ch4 = ch4;
}

void led_task(void *parameter)
{
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOE, led_buf);

		if (led_buf < 0x0100)
		{
			led_buf <<= 1;
		}
		else
		{
			led_buf = 0x0001;
		}

		vTaskDelay(200);
	}
}

void sbus_task(void *parameter)
{
	static bool ch5_was_high = false;

	while (1)
	{
		rx_set(&sbus_ch);

		/* Update SBUS health timestamp on valid frame */
		if (sbus_frame_valid())
			last_sbus_tick = xTaskGetTickCount();

		/* Update system enable state from CH4 */
		system_enable_handler();

		/* Manual mode: map remote channels to velocities */
		if (sys_enabled && !ch_high(5) && !ch_high(6))
		{
			int16_t ch_val;

			ch_val = sbus_ch.ch[2];
			set_vx = (ch_val >= 1017 && ch_val <= 1030) ? 0 : Map(ch_val, 256, 1800, -10000, 10000);

			ch_val = sbus_ch.ch[3];
			set_vy = (ch_val >= 1017 && ch_val <= 1030) ? 0 : Map(ch_val, 240, 1780, -10000, 10000);

			ch_val = sbus_ch.ch[0];
			set_vw = (ch_val >= 1020 && ch_val <= 1028) ? 0 : Map(ch_val, 268, 1783, -10000, 10000);

			/* TODO: CH8/CH9 gripper control (old YV3/YV4/YV5 removed) */
		}

		/* CH6 + CH7 组合控制:
		   CH6低档(240)  + CH7沿 → 翻转电机 0↔1.57
		   CH6中档(1024) + CH7沿 → 夹爪气缸 YV1 开↔关 */
			{
				static bool last_ch7_state = false;
				int16_t ch6_val = sbus_ch.ch[6];
				bool ch7_now = sbus_ch.ch[7] > 1700;

				if (ch7_now != last_ch7_state)
				{
					if (ch6_val < 632)              // CH6 低档 ~240: 翻转电机
					{
						gripper_flip_pos   = (gripper_flip_pos < 0.5f) ? 1.57f : 0.0f;
						gripper_flip_ready = true;
					}
					else if (ch6_val < 1415)        // CH6 中档 ~1024: 夹爪气缸
					{
						YV1(2);                     // 翻转 YV1 (1→0, 0→1)
					}
				}
				last_ch7_state = ch7_now;
			}

		/* CH5 falling edge: notify upper computer on mode switch */
		if (sbus_frame_valid() && ch_high(4))
		{
			bool ch5_high = ch_high(5);
			if (!ch5_was_high && ch5_high)  // rising edge: manual→auto
			{
				uart_tx_ch5_notify();
			}
			ch5_was_high = ch5_high;
		}

		/* Safe defaults when disabled */
		if (!sys_enabled)
		{
			set_vx = 0;
			set_vy = 0;
			set_vw = 0;
		}

		vTaskDelay(10);
	}
}

void up_cs_task(void *parameter)
{
	while (1)
	{
		if (sys_enabled)
		{
			float lift_vel;     // 抬升速度, 由 lift_mode 控制
			float flip_vel;     // 夹爪翻转速度, 手动慢/自动快

			if (!ch_high(5))
			{
				/* Manual: CH8→前抬升, CH9→后抬升 (mm) */
				int16_t ch8 = sbus_ch.ch[8];
				int16_t ch9 = sbus_ch.ch[9];
				float front_mm = (ch8 >= 1021 && ch8 <= 1027)
								? 0.0f
								: Map(ch8, 240, 1800, 0.0f, 417.0f);
				float rear_mm  = (ch9 >= 1021 && ch9 <= 1027)
								? 0.0f
								: Map(ch9, 240, 1800, 0.0f, 417.0f);
				lift_set(0, front_mm);  // FR
				lift_set(1, front_mm);  // FL
				lift_set(2, rear_mm);   // BL
				lift_set(3, rear_mm);   // BR
				lift_front_actual = (uint16_t)front_mm;
				lift_back_actual  = (uint16_t)rear_mm;
				lift_vel = LIFT_VEL_SLOW;             // 手动模式慢速抬升
				flip_vel = GRIPPER_FLIP_VEL_SLOW;     // 手动模式慢速翻转
			}
			else
			{
				/* Auto: 串口协议控制, 前后分组 */
				lift_set(0, (float)lift_front_target);  // FR
				lift_set(1, (float)lift_front_target);  // FL
				lift_set(2, (float)lift_back_target);   // BL
				lift_set(3, (float)lift_back_target);   // BR
				lift_front_actual = lift_front_target;
				lift_back_actual  = lift_back_target;
				lift_vel = (lift_mode == 1) ? LIFT_VEL_FAST : LIFT_VEL_SLOW;
				flip_vel = GRIPPER_FLIP_VEL_FAST;       // 自动模式快速翻转

				/* 武器头控制: weapon_pitch→翻转电机, weapon_gripper→夹爪气缸 */
				if (weapon_pitch == 1)      // 平行地面 → 翻下
				{
					gripper_flip_pos = 1.57f;
					gripper_flip_ready = true;
				}
				else if (weapon_pitch == 2) // 垂直地面 → 竖起
				{
					gripper_flip_pos = 0.0f;
					gripper_flip_ready = true;
				}
				// weapon_pitch=0 保持, 不更新

				if (weapon_gripper == 1)      { YV1(1); }  // 打开夹爪
				else if (weapon_gripper == 2) { YV1(0); }  // 闭合夹爪
				// weapon_gripper=0 保持, 不更新
			}

			/* CAN1 抬升电机位置控制 */
			lift_update(&hcan1, lift_vel);

			/* 夹爪翻转电机 — 上电不动作, CH7首次操作后才开始控制 */
			if (gripper_flip_ready)
				pos_ctrl_mcp2515(&hcan3, 0x01, gripper_flip_pos, flip_vel);
			YV_flash_mcp2515(&hcan3);   // 电磁阀状态刷新 (每周期)
		}

		vTaskDelay(50);
	}
}

void arm_task(void *parameter)
{
	/* ⚠️ TODO: replace with dual-arm control (left + right)
	   was drive_arm_to_ik_3d() from deleted arm.c, now needs
	   two independent arm IK solvers for left/right arms */
	while (1)
	{
		if (sys_enabled)
		{
			if (!ch_high(5))
			{
				/* TODO: dual-arm safe position */
			}
			else
			{
				/* TODO: dual-arm IK: use fb_des/lr_des/ud_des/end_des from UART2 */
			}
		}

		vTaskDelay(10);
	}
}

void chassis_task(void *parameter)
{
	while (1)
	{
		if (sys_enabled)
		{
			chassis_update(&hcan1);   // 全向轮运动学 + CAN1 MIT 驱动
		}

		vTaskDelay(2);
	}
}

void start_task(void *parameter)
{
	while (1)
	{
		sbus_rx_init();
		can_filter_init();
		uart_rx_init();
		chassis_init(&hcan1);

		mcp2515_sys_init(&hcan3, &hspi1, GPIOA, GPIO_PIN_4);//电磁阀和翻转电机

		xTaskCreate(led_task, "led_task", 56, NULL, 0, NULL);
		xTaskCreate(sbus_task, "remote_task", 256, NULL, 0, NULL);
		xTaskCreate(uart_task, "uart_task", 512, NULL, 0, NULL);
		xTaskCreate(chassis_task, "chassis_task", 1280, NULL, 0, NULL);
		xTaskCreate(up_cs_task, "up_cs_task", 256, NULL, 0, NULL);
		xTaskCreate(arm_task, "arm_task", 512, NULL, 0, NULL);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	if (xTaskCreate(start_task, "start_task", 256, NULL, 0, NULL) != pdPASS)
	{
		while (1)
			;
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
/* (HAL_UART_RxCpltCallback moved to Lib/uart_task.c) */
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
