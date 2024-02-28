/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t SB[] = {13,10};
uint8_t S1[] = {"Trig'd!"};
uint8_t S2[] = {"Delay SET!"};
uint8_t S3[] = {"Width SET!"};
uint8_t S4[] = {"Burst Mode!"};
uint8_t S5[] = {"Single Mode!"};
uint8_t S17[] = {"XFS_TRG_GEN_V2"};
uint8_t S18[] = {"SESAME XAFS Trigger Generator"};
uint8_t S19[] = {"SN:06BRV114"};
uint8_t S20[] = {"By:cdokuyucu"};
uint8_t rx_buff[11];
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_buff, 11);
   HAL_UART_Transmit(&huart1, S17, 11, 1000);
   		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
   		  HAL_UART_Transmit(&huart1, S18, 31, 1000);
   		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
   		  HAL_UART_Transmit(&huart1, S19, 11, 1000);
   		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
   		  HAL_UART_Transmit(&huart1, S20, 12, 1000);
   	 	 HAL_UART_Transmit(&huart1, SB, 2, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if((rx_buff[0]=='C')&(rx_buff[1]=='T')&(rx_buff[2]=='R')&(rx_buff[3]=='I')&(rx_buff[4]=='D')&(rx_buff[5]=='N')&(rx_buff[6]=='/')&(rx_buff[7]=='*')&(rx_buff[8]=='?')){
	  	  	 		  HAL_Delay(100);
	  	  	 		  HAL_UART_Transmit(&huart1, S17, 11, 1000);
	  	  	 		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  	  	 		  HAL_UART_Transmit(&huart1, S18, 31, 1000);
	  	  	 		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  	  	 		  HAL_UART_Transmit(&huart1, S19, 10, 1000);
	  	  	 		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  	  	 		  HAL_UART_Transmit(&huart1, S20, 12, 1000);
	  	  	 	 	 HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  	  	 	 	 	 	  	       rx_buff[0]='x';
	  	  	 	 	 	 	  	 	   rx_buff[1]='x';
	  	  	 	 	 	 	  	 	   rx_buff[2]='x';
	  	  	 	 	 	 	  	 	   rx_buff[3]='x';
	  	  	 	 	 	 	  	 	   rx_buff[4]='x';
	  	  	 	 	 	 	  	 	   rx_buff[5]='x';
	  	  	 	 	 	 	  	 	   rx_buff[6]='x';
	  	  	 	 	 	 	  	 	   rx_buff[7]='x';
	  	  	 	 	 	 	  	 	   rx_buff[8]='x';
	  	  	 	 	 	 	  	 }
	  if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='N')&(rx_buff[5]=='D')&(rx_buff[6]=='T')&(rx_buff[7]=='R')&(rx_buff[8]=='G')){
		  HAL_GPIO_WritePin(STP_GPIO_Port,STP_Pin, GPIO_PIN_RESET);
		  	  	 		HAL_Delay(10);
		  	  	 		HAL_GPIO_WritePin(STP_GPIO_Port,STP_Pin, GPIO_PIN_SET);
		  	  	 		 HAL_Delay(10);
	 	  	  	 		  HAL_Delay(100);
	 	  	  	 		  HAL_UART_Transmit(&huart1, S1, 7, 1000);
	 	  	  	 	      HAL_UART_Transmit(&huart1, SB, 2, 1000);
	 	  	  	 	 	 	 	  	       rx_buff[0]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[1]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[2]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[3]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[4]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[5]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[6]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[7]='x';
	 	  	  	 	 	 	 	  	 	   rx_buff[8]='x';
	 	  	  	 	 	 	 	  	 }



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STP_GPIO_Port, STP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STP_Pin */
  GPIO_InitStruct.Pin = STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STP_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, rx_buff, 11);


 //You need to toggle a breakpoint on this line!
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
