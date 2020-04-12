/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
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
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int encoder_value = 0;
int last_value = 0;
uint8_t encoderDirection;
char strBuffer[32];
int cycle = 0;
int input_capture;
char txBuffer[20] = "UART ILETIM BASLADI:\n";
void islemYap(void);
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	if (HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, sizeof(txBuffer),100) != HAL_OK)
	{
		Error_Handler();
	}

	 // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Bu kýsým loop içerisinde polling modda iþlem için Aktif hale getirilir
	  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL); // Bu kýsým kesme ile iþlem yapmak için aktif hale getirilir
  while (1)
  {
//	  encoder_value = __HAL_TIM_GET_COUNTER(&htim4);
//	  encoderDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
//
//	  sprintf(strBuffer, "Count=%d,Direction=%s\n", encoder_value, (encoderDirection == 1) ? "CCW" : "CW");
//
//		HAL_UART_Transmit(&huart1, (uint8_t *)strBuffer, strlen(strBuffer), 1);
//
//
//		 if(encoderDirection == 1 && encoder_value !=last_value)
//			  {
//				    for (cycle = 0; cycle < 10; cycle++)
//				    {
//				    	//GPIOA->ODR = 0x0001;
//				    	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1 HIGH
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
//				        HAL_Delay(5);
//				    	//GPIOA->ODR = 0x0002;
//				    	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	// IN2 HIGH
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
//				    	HAL_Delay(5);
//				    	//GPIOA->ODR = 0x0004;
//				    	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_SET);    // IN3 HIGH
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
//				    	HAL_Delay(5);
//				    	// GPIOA->ODR = 0x0008;
//				    	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_SET);    // IN4 HIGH
//				    	HAL_Delay(5);
//				    }
//				    last_value = encoder_value;
//			  }
//			  else if(encoderDirection == 0 && encoder_value !=last_value)
//			  {
//				    for (cycle = 0; cycle < 10; cycle++)
//				    {
//				    	GPIOA->ODR = 0x0008;
//				    	HAL_Delay(5);
//				    	GPIOA->ODR = 0x0004;
//				        HAL_Delay(5);
//				    	GPIOA->ODR = 0x0002;
//				    	HAL_Delay(5);
//				    	GPIOA->ODR = 0x0001;
//				    	HAL_Delay(5);
//				    }
//				    last_value = encoder_value;
//			  }
//		HAL_Delay(100);
  }
  /* USER CODE END 3 */
}


 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	 if (htim->Instance==TIM4)
	  {
		 islemYap();
	  }
}

 void islemYap(void){
	 encoder_value = __HAL_TIM_GET_COUNTER(&htim4);
     encoderDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);

     sprintf(strBuffer, "Count=%d,Direction=%s\n", encoder_value, (encoderDirection == 1) ? "CCW" : "CW");

	 HAL_UART_Transmit(&huart1, (uint8_t *)strBuffer, strlen(strBuffer), 1);

	 if(encoderDirection == 1 && encoder_value !=last_value)
	  {
		for (cycle = 0; cycle < 10; cycle++)
		{
			//GPIOA->ODR = 0x0001;
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1 HIGH
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			for (volatile uint32_t i = 0; i < 0xFFFF; i++);
			//GPIOA->ODR = 0x0002;
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	// IN2 HIGH
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			for (volatile uint32_t i = 0; i < 0xFFFF; i++);
			//GPIOA->ODR = 0x0004;
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_SET);    // IN3 HIGH
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			for (volatile uint32_t i = 0; i < 0xFFFF; i++);
			// GPIOA->ODR = 0x0008;
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_SET);    // IN4 HIGH
			for (volatile uint32_t i = 0; i < 0xFFFF; i++);
		}
		last_value = encoder_value;
	  }
	  else if(encoderDirection == 0 && encoder_value !=last_value)
	  {
		for (cycle = 0; cycle < 10; cycle++)
		{
			GPIOA->ODR = 0x0008;
			for (volatile uint16_t i = 0; i < 0xFFFF; i++);
			GPIOA->ODR = 0x0004;
			for (volatile uint16_t i = 0; i < 0xFFFF; i++);
			GPIOA->ODR = 0x0002;
			for (volatile uint16_t i = 0; i < 0xFFFF; i++);
			GPIOA->ODR = 0x0001;
			for (volatile uint16_t i = 0; i < 0xFFFF; i++);
		}
		last_value = encoder_value;
	  }

 }

  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {

 }
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period =  21-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.RepetitionCounter = 0;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
