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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODE 2 // 0 IT, 1 DMA, 2 BUSY
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t TX_TIM6_f = 0;
uint8_t TX_UART_f = 0;
uint8_t uart_buff = 0;
uint8_t TX_Packet[8] = {0xbb, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0xee};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  #if (MODE == 1)
  	  HAL_UART_Receive_DMA(&huart2, &uart_buff, 1);
  #elif (MODE == 0)
	  HAL_UART_Receive_IT(&huart2, &uart_buff, 1);
  #endif

  HAL_TIM_Base_Start_IT(&htim6);

  #if (MODE != 2)
  	  HAL_SuspendTick();
  	  HAL_PWR_EnableSleepOnExit();
  	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	#if (MODE == 2)
	  if (HAL_UART_Receive(&huart2, &uart_buff, 1, 10) == HAL_OK) {
		  HAL_UART_Transmit(&huart2, &uart_buff, 1, 10);
	  }
	  if (TX_TIM6_f) {
		  TX_TIM6_f = 0;
		  HAL_UART_Transmit(&huart2, TX_Packet, 8, 10);
	  }
	#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// LED On
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	#if (MODE == 1)
		if (huart2.gState != HAL_UART_STATE_READY) {
			TX_UART_f = 1;
		} else HAL_UART_Transmit_DMA(&huart2, &uart_buff, 1);
	#elif (MODE == 0)
		if (huart2.gState != HAL_UART_STATE_READY) {
			TX_UART_f = 1;
		} else {
			HAL_UART_Transmit_IT(&huart2, &uart_buff, 1);
			HAL_UART_Receive_IT(&huart2, &uart_buff, 1);
		}
	#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (TX_UART_f) {
		TX_UART_f = 0;
		#if (MODE == 1)
			HAL_UART_Transmit_DMA(&huart2, &uart_buff, 1);
		#elif (MODE == 0)
			HAL_UART_Transmit_IT(&huart2, &uart_buff, 1);
			HAL_UART_Receive_IT(&huart2, &uart_buff, 1);
		#endif
	} else if (TX_TIM6_f) {
		TX_TIM6_f = 0;
		#if (MODE == 1)
			HAL_UART_Transmit_DMA(&huart2, TX_Packet, 8);
		#elif (MODE == 0)
			HAL_UART_Transmit_IT(&huart2, TX_Packet, 8);
		#endif
	}
	// LED Off
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		#if (MODE == 1)
			if (huart2.gState != HAL_UART_STATE_READY) {
				TX_TIM6_f = 1;
			} else HAL_UART_Transmit_DMA(&huart2, TX_Packet, 8);
		#elif (MODE == 0)
			if (huart2.gState != HAL_UART_STATE_READY) {
				TX_TIM6_f = 1;
			} else HAL_UART_Transmit_IT(&huart2, TX_Packet, 8);
		#elif (MODE == 2)
			if (huart2.gState != HAL_UART_STATE_READY) {
				TX_TIM6_f = 1;
			} else HAL_UART_Transmit(&huart2, TX_Packet, 8, 10);
		#endif
	}
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
