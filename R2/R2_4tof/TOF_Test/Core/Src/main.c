/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOF_NUM         4
#define TOF_TX_LEN      8
#define TOF_RX_LEN      7
#define TOF_TIMEOUT     20      // ms																		
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tof_addresses[TOF_NUM] =
{
    0x01,
    0x02,
    0x03,
    0x04
};

uint8_t uart1_tx_buf[TOF_NUM][TOF_TX_LEN];
uint8_t uart1_rx_buf[TOF_NUM][TOF_RX_LEN];
volatile uint8_t uart1_rx_finish = 0;

uint8_t uart3_tx_buf[10];
volatile uint8_t uart3_tx_finish = 1;

uint16_t distance_values[TOF_NUM];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t calculate_crc16(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for(uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void generate_command(uint8_t addr, uint8_t *cmd_buf)
{
    cmd_buf[0] = addr;
    cmd_buf[1] = 0x03;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x10;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x01;

    uint16_t crc = calculate_crc16(cmd_buf, 6);

    cmd_buf[6] = crc & 0xFF;
    cmd_buf[7] = crc >> 8;
}

void init_all_tof_commands(void)
{
    for(uint8_t i = 0; i < TOF_NUM; i++)
    {
        generate_command(tof_addresses[i], uart1_tx_buf[i]);
    }
}

uint8_t parse_distance_data(uint8_t *rx_data, uint16_t *distance, uint8_t tof_id)
{
    uint16_t crc;
    uint16_t rx_crc;

    if(rx_data[0] != tof_id)
        return 0;

    if(rx_data[1] != 0x03)
        return 0;

    if(rx_data[2] != 0x02)
        return 0;

    crc = calculate_crc16(rx_data, 5);

    rx_crc = rx_data[5] | (rx_data[6] << 8);

    if(crc != rx_crc)
        return 0;

    *distance = ((uint16_t)rx_data[3] << 8) | rx_data[4];

    return 1;
}

uint8_t read_tof_distance(uint8_t index, uint16_t *distance)
{
    uint32_t tick;

    uart1_rx_finish = 0;

    memset(uart1_rx_buf[index], 0, TOF_RX_LEN);

    if(HAL_UART_Receive_DMA(&huart1, uart1_rx_buf[index], TOF_RX_LEN) != HAL_OK)
    {
        return 0;
    }

    if(HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf[index], TOF_TX_LEN) != HAL_OK)
    {
        HAL_UART_DMAStop(&huart1);
        return 0;
    }

    tick = xTaskGetTickCount();

    while(uart1_rx_finish == 0)
    {
        if((xTaskGetTickCount() - tick) > pdMS_TO_TICKS(TOF_TIMEOUT))
        {
            HAL_UART_DMAStop(&huart1);
            return 0;
        }

        vTaskDelay(1);
    }

    if(parse_distance_data(uart1_rx_buf[index], distance, tof_addresses[index]))
    {
        if((*distance >= 5) && (*distance <= 1300))
        {
            return 1;
        }
    }

    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        uart1_rx_finish = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        uart3_tx_finish = 1;
    }
}

void send_tof_packet(void)
{
    if (uart3_tx_finish == 0)
        return;

    uart3_tx_finish = 0;

    uart3_tx_buf[0] = 0xCC;

    // ǰ������ǰ�߶�
    uart3_tx_buf[1] = (distance_values[0] >> 8) & 0xFF;
    uart3_tx_buf[2] = distance_values[0] & 0xFF;

    // ��������ǰ�߶�
    uart3_tx_buf[3] = (distance_values[1] >> 8) & 0xFF;
    uart3_tx_buf[4] = distance_values[1] & 0xFF;

    // ǰ����ǰ�� ToF
    uart3_tx_buf[5] = (distance_values[2] >> 8) & 0xFF;
    uart3_tx_buf[6] = distance_values[2] & 0xFF;

    // ��������� ToF
    uart3_tx_buf[7] = (distance_values[3] >> 8) & 0xFF;
    uart3_tx_buf[8] = distance_values[3] & 0xFF;

    uart3_tx_buf[9] = 0xEE;

    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

void led_task(void *parameter)
{
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		
		vTaskDelay(200);
	}
}

void tof_task(void *parameter)
{
    init_all_tof_commands();

    while(1)
    {
        for(uint8_t i = 0;i < TOF_NUM;i++)
        {
            uint16_t distance;

            if(read_tof_distance(i,&distance))
            {
                distance_values[i] = distance;
            }
            else
            {
                distance_values[i] = 0xFFFF;
            }

            vTaskDelay(10);
        }
		
		send_tof_packet();
    }
}


void start_task(void *parameter)
{
	while(1)
	{	
		xTaskCreate(led_task,"led_task",56,NULL,0,NULL);
		xTaskCreate(tof_task,"tof_task",256,NULL,0,NULL);
		
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	if (xTaskCreate(start_task, "start_task", 128, NULL, 0, NULL) != pdPASS)
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
