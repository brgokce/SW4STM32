#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"


UART_HandleTypeDef uartHandle;
ADC_HandleTypeDef adcHandle;

uint8_t txBuffer[] = "Klavyeden 10 karakterli bir metin girin:\n";
uint8_t rxBuffer[10];

int ADC_DMA_Value[3];
char buffer[32];

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void GPIO_Input_Config(void);
void GPIO_Output_Config(void);
void ADC_Config(void);
void Error_Handler(void);
void UART_Config(void);


int main(void)
{
	/*## HAL S�r�c�s�n� ba�lat�yoruz */
	HAL_Init();

	/*## System clocks ayarlar�n� ba�lat�ryoruz */
	/* SYSCLK 72 MHz'e ayarland� */
	RCC_SystemClock_Config();

	/*## GPIO Giri� ve ��k��lar Ba�lat�l�yor */
	GPIO_Output_Config();
	GPIO_Input_Config();

	/*## ADC Ba�lat�l�yor*/
	ADC_Config();

	UART_Config();

	/* UART Haberle�menin ba�lamas� ve program�n ilerlemesi i�in C15 pinine ba�l� butona bas�lmas� gerekiyor
	 * Yoksa sistem �al��m�yor */

	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) != GPIO_PIN_SET);

	/*  "txBuffer" datas�n� USART �zerinden bilgisayara iletiyoruz */
	if (HAL_UART_Transmit(&uartHandle, (uint8_t*)txBuffer, sizeof(txBuffer),100) != HAL_OK)
	{
		Error_Handler();
	}

	/* DMA �zerinden Al�nan veri "rxBuffer" tampon belle�inde saklan�r */
	if (HAL_UART_Receive_DMA(&uartHandle, (uint8_t*)rxBuffer, 10) != HAL_OK)
	{
		Error_Handler();
	}

	/* Yeni bir ileti�im aktar�m�na ba�lamadan �nce, �evre biriminin mevcut durumunu kontrol etmeniz gerekir;
	 * Me�gulse yenisini ba�latmadan �nce mevcut aktar�m�n bitmesini beklemeniz gerekir.
	 * Basitlik nedeniyle, bu �rnek yaln�zca aktar�m�n sonuna kadar bekliyor,
	 * ancak aktar�m i�lemi devam ederken uygulama ba�ka g�revleri de ger�ekle�tirebilir. */
	while (HAL_UART_GetState(&uartHandle) != HAL_UART_STATE_READY);


   // ADC �zerinden ald���m�z veriler haf�zaya aktar�ld�ktan sonra haf�zadan do�rudan DMA ve USART �zerinden bilgisayara g�nderilir.
	if (HAL_UART_Transmit_DMA(&uartHandle, (uint8_t*)buffer, sizeof(buffer)) != HAL_OK)
	{
		Error_Handler();
	}

	while (1); // Loop i�erisine bir�ey yazm�yoruz
}

/*** @brief  System clock konfig�rasyonu:
  *             System clock source = PLL (HSE)
  *             SYSCLK(Hz)          = 72000000
  *             HCLK(Hz)            = 72000000
  *             AHB prescaler       = 1
  *             APB1 prescaler      = 2
  *             APB2 prescaler      = 1
  *             HSE frequency(Hz)   = 8000000
  *             HSE PREDIV1         = 1
  *             PLLMUL              = 9
  *             Flash latency(WS)   = 2 */

void RCC_SystemClock_Config(void)
{
	RCC_ClkInitTypeDef rccClkInit;
	RCC_OscInitTypeDef rccOscInit;

	/*## STEP 1: HSE ve PLL konfig�rasyonu */
	rccOscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	rccOscInit.HSEState       = RCC_HSE_ON;
	rccOscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	rccOscInit.PLL.PLLState   = RCC_PLL_ON;
	rccOscInit.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	rccOscInit.PLL.PLLMUL     = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&rccOscInit) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: SYSCLK, HCLK, PCLK1, ve PCLK2 konfig�rasyonu */
	rccClkInit.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	rccClkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	rccClkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	rccClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	rccClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&rccClkInit, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}
void GPIO_Input_Config(void)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: GPIOC RCC ENABLE konfig�rasyonu ####################################*/
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*## STEP 2:  GPIO Konfig�rasyonu ##############################################*/
	gpioInit.Pin  = GPIO_PIN_15;
	gpioInit.Mode = GPIO_MODE_INPUT;
	gpioInit.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &gpioInit);
}

void GPIO_Output_Config(void)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: GPIOB RCC konfig�rasyonu */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*## STEP 2: GPIO konfig�rasyonu */
	gpioInit.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpioInit);
}

void ADC_Config(void)
{
	ADC_ChannelConfTypeDef ADC_ChannelConfStruct;

	/*## STEP 1: ADC konfig�rasyonu */
	adcHandle.Instance                   = ADC1;
	adcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	adcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
	adcHandle.Init.ContinuousConvMode    = ENABLE;
	adcHandle.Init.NbrOfConversion       = 3;
	adcHandle.Init.DiscontinuousConvMode = DISABLE;
	adcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	if (HAL_ADC_Init(&adcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/*  ADC channel konfig�rasyonu*/
	ADC_ChannelConfStruct.Channel      = ADC_CHANNEL_4;
	ADC_ChannelConfStruct.Rank         = ADC_REGULAR_RANK_1;
	ADC_ChannelConfStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &ADC_ChannelConfStruct) != HAL_OK)
	{
		Error_Handler();
	}
	ADC_ChannelConfStruct.Channel      = ADC_CHANNEL_5;
	ADC_ChannelConfStruct.Rank         = ADC_REGULAR_RANK_2;
	ADC_ChannelConfStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &ADC_ChannelConfStruct) != HAL_OK)
	{
		Error_Handler();
	}

	ADC_ChannelConfStruct.Channel      = ADC_CHANNEL_6;
	ADC_ChannelConfStruct.Rank         = ADC_REGULAR_RANK_3;
	ADC_ChannelConfStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &ADC_ChannelConfStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2:  ADC BA�LAT */
	if (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)ADC_DMA_Value, 3)!= HAL_OK)
	{
		Error_Handler();
	}
}

/** @brief  ADC MSP Konfig�rasyonu MSP Init ile ADC'yi ba�lat�yoruz..  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	RCC_PeriphCLKInitTypeDef rccPeriphCLKInit;
	GPIO_InitTypeDef gpioInit;
	static DMA_HandleTypeDef dmaHandle;

	/*## STEP 1:  RCC konfig�rasyonu  ####################################*/
	/* ADC clock prescaler konfig�rasyonu */
	__HAL_RCC_ADC1_CLK_ENABLE();
	rccPeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	rccPeriphCLKInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
	HAL_RCCEx_PeriphCLKConfig(&rccPeriphCLKInit);
	/* GPIO i�in RCC konfig�rasyonu */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* DMA i�in  RCC konfig�rasyonu */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*## STEP 2: ADC GPIO konfig�rasyonu */
	/*  PA4, PA5 ve A5 ADC giri� konfig�rasyonu */
	gpioInit.Pin  = GPIO_PIN_4 | GPIO_PIN_5;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioInit);

	/*## STEP 3:  DMA konfig�rasyonu */
	dmaHandle.Instance                 = DMA1_Channel1;
	dmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	dmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	dmaHandle.Init.Mode                = DMA_CIRCULAR;
	dmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaHandle);
	__HAL_LINKDMA(hadc, DMA_Handle, dmaHandle);

	/*## STEP 4:  NVIC konfig�rasyonu */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/** @brief  ADC Conversion complete callback  fonksiyonunu non blocking mode. �a��r�yoruz
 * Her d�n���m i�lemi tamamland�ktan sonra ADC de�erine g�re LED'ler yak�lacak.  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (ADC_DMA_Value[0] > 2047){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}

	if (ADC_DMA_Value[1] > 2047){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	}
}

void UART_Config(void){
	/*## STEP 1:  UART konfig�rasyonu */
	uartHandle.Instance        = USART2;
	uartHandle.Init.BaudRate   = 115200;
	uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	uartHandle.Init.StopBits   = UART_STOPBITS_1;
	uartHandle.Init.Parity     = UART_PARITY_NONE;
	uartHandle.Init.Mode       = UART_MODE_TX_RX;
	uartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	if (HAL_UART_Init(&uartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/** @brief  UART MSP konfig�rasyonu . */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpioInit;
	static DMA_HandleTypeDef dmaHandleTx;
	static DMA_HandleTypeDef dmaHandleRx;

	/*## STEP 1: RCC konfig�rasyonu */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*## STEP 2: GPIO konfig�rasyonu */
	/* Configure PA2 for UART TX */
	gpioInit.Pin   = GPIO_PIN_2;
	gpioInit.Mode  = GPIO_MODE_AF_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioInit);
	/* Configure PA3 for UART RX */
	gpioInit.Pin   = GPIO_PIN_3;
	gpioInit.Mode  = GPIO_MODE_AF_INPUT;
	gpioInit.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioInit);

	/*## STEP 3:  DMA konfig�rasyonu */
	/*  UART tx iletimi i�in DMA konfig�rasyonu */
	dmaHandleTx.Instance                 = DMA1_Channel7;
	dmaHandleTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	dmaHandleTx.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaHandleTx.Init.MemInc              = DMA_MINC_ENABLE;
	dmaHandleTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dmaHandleTx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	dmaHandleTx.Init.Mode                = DMA_CIRCULAR;
	dmaHandleTx.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaHandleTx);
	__HAL_LINKDMA(huart, hdmatx, dmaHandleTx);
	/* Configure DMA for UART reception process */
	dmaHandleRx.Instance                 = DMA1_Channel6;
	dmaHandleRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaHandleRx.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaHandleRx.Init.MemInc              = DMA_MINC_ENABLE;
	dmaHandleRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dmaHandleRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	dmaHandleRx.Init.Mode                = DMA_NORMAL;
	dmaHandleRx.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaHandleRx);
	__HAL_LINKDMA(huart, hdmarx, dmaHandleRx);

	/*## Step 4: Configure NVIC ##############################################*/
	/* Configure NVIC for DMA1 channel 7 (USART2 TX) */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	/* Configure NVIC for DMA1 channel 6 (USART2 RX) */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* Configure NVIC for USART2 */
	HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

// @brief  Tx Transfer completed callback.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	sprintf(buffer,"A0: %5d A1: %5d A2: %5d\t\n", ADC_DMA_Value[0], ADC_DMA_Value[1], ADC_DMA_Value[2]);
}


void Error_Handler(void)
{
	/* Turn red LED on */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	while (1);
}

/******************************** END OF FILE *********************************/
/******************************************************************************/
