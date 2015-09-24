/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "string.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
DMA_HandleTypeDef hdma_sdio;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define CHANNELS 14
#define ADCBUFS 2
#define SD_BLOCKSIZE 512

#define ADCMODE 2	// 0 -- 8bit, 1 -- 12bit->8bit, 2 -- 12bit

#if ADCMODE > 0
#	define BUF_LEN ((SD_BLOCKSIZE<<5) + (SD_BLOCKSIZE<<4))
	typedef uint16_t adcword_t;
#else 
#	define BUF_LEN ((SD_BLOCKSIZE<<6) + (SD_BLOCKSIZE<<5))
	typedef uint8_t adcword_t;
#endif

/* SD_CHANNELS should be a power of 2 minus X and greater or equal to CHANNELS; x == 2 while ADCMODE < 2; x == 1 while ADCMODE >= 2 */
#if ADCMODE >= 2
#	define SD_CHANNELS 15
#else
#	define SD_CHANNELS 14
#endif

#if ADCMODE >= 2
#	define	dst_points (SD_BLOCKSIZE / 2 / (SD_CHANNELS + 1))
#else
#	define	dst_points (SD_BLOCKSIZE / (SD_CHANNELS + 2))
#endif


#define ADC_NOT_FILLED ((adcword_t)~0)


adcword_t adcbufs[ADCBUFS][BUF_LEN] = {{0,1,2,3,4,5,6,7,8,9,10},{0,1,2,3,4,5,6,7,8,9,10}};
adcword_t *parsebuf = &adcbufs[1][0];
adcword_t *adcbuf   = &adcbufs[0][0];
uint8_t   adcbuf_runned = 0;
int adc_running = 0;

#if ADCMODE >= 2
	typedef uint16_t sdword_t;
#else
	typedef uint8_t  sdword_t;
#endif

sdword_t writebuf[SD_BLOCKSIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void error (float error_num, char infinite) {
	//printf("%u\r\n", error_num);
	adc_running = 0;
	if (infinite)
		while (1) {
			int i = 0;
			while (i++ < ((int)((float)(error_num) / 1) + 1) ) {
				GPIOA->BSRR = GPIO_PIN_5;
				HAL_Delay(1000 / error_num);
				GPIOA->BSRR = GPIO_PIN_5 << 16;
				HAL_Delay(1000 / error_num);
			}
		};

	GPIOA->BSRR = GPIO_PIN_5;
	HAL_Delay(1000);
	GPIOA->BSRR = GPIO_PIN_5 << 16;

	int i=0;
	while (i++ < error_num) {
		GPIOA->BSRR = GPIO_PIN_5;
		HAL_Delay(300);
		GPIOA->BSRR = GPIO_PIN_5 << 16;
		HAL_Delay(200);
	}

	HAL_Delay(500);
	NVIC_SystemReset();

	return;
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
	return;
}

void HAL_ADC_ErrorCallback (ADC_HandleTypeDef* hadc) {
	NVIC_SystemReset();

	return;
}

void swap_bufs() {
	if (adcbuf_runned == 1) {
		parsebuf = &adcbufs[1][0];
		adcbuf   = &adcbufs[0][0];
		adcbuf_runned = 0;
	} else {
		parsebuf = &adcbufs[0][0];
		adcbuf   = &adcbufs[1][0];
		adcbuf_runned = 1;
	}

	return;
}

void adc_restart() {
	uint32_t i = 0;
	while (parsebuf[i] != ~0 && i < BUF_LEN) parsebuf[i++] = ~0;

	if (adc_running) {
		HAL_ADC_Stop_DMA(&hadc1);
	}

	swap_bufs();

	int err;
	adc_running = 1;

	if ((err = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcbuf, BUF_LEN)) != HAL_OK) {
		error(400, 0);
	}

	return;
}

void fill_writebuf(sdword_t *dst_buf, adcword_t *src_buf, uint16_t src_point_start, uint16_t src_points) {
	float k = (float)src_points / (float)dst_points;

	if (k < 1) {
		error(src_points, 1);
		NVIC_SystemReset();	// Too slow ADC
	}

	uint16_t dst_point = 0, src_point = src_point_start;
	uint32_t dst       = 0, src       = src_point*CHANNELS;

#if ADCMODE >= 2
#	define SD_LEN	(SD_BLOCKSIZE / 2)
#else
#	define SD_LEN	 SD_BLOCKSIZE
#endif

	while (dst < SD_LEN) {

#undef SD_LEN

#if ADCMODE >= 2
		dst_buf[dst++] = src_point;
#else
		dst_buf[dst++] = src_point;//ts[src_point];
		dst_buf[dst++] = src_point >> 8;//ts[src_point] << 8;
#endif

		int chan = 0;
		do {
#if ADCMODE == 1
			dst_buf[dst++] = src_buf[src++] >> 4;
#else
			dst_buf[dst++] = src_buf[src++];
#endif
		} while (++chan < CHANNELS);

#if CHANNELS < SD_CHANNELS
		while (chan < SD_CHANNELS) {
			dst_buf[dst++] = 0;
			chan++;
		}
#endif

		dst_point++;

		src_point = (float)src_point_start + (float)dst_point * k + 0.000001;
		src       = src_point * CHANNELS;
	}

	return;
}

sdword_t *fill_writebuf_detailed() {
	static uint32_t cur_pos = 0;
	static uint8_t buf_switched = 0;

	adcword_t *buf;

	if ((cur_pos > BUF_LEN/CHANNELS / 2) && (buf_switched == 0)) {
		adc_restart();
		buf_switched = 1;
	}

	buf = (buf_switched ? parsebuf : adcbuf);

	while (buf[BUF_LEN / 2] == ADC_NOT_FILLED);

	fill_writebuf(writebuf, buf, cur_pos, dst_points);

	cur_pos += dst_points;

	if (cur_pos + dst_points > BUF_LEN/CHANNELS) {
		cur_pos = 0;
		buf_switched = 0;
	}

	return writebuf;
}


sdword_t *fill_writebuf_inseparable() {
	adc_restart();

	// searching the end of the filled part of the buffer
	static uint32_t buf_len = 0;
	uint32_t s = 0, e = BUF_LEN-1;
	do {
		uint32_t c = (e+s)/2;
		if (parsebuf[c] == ADC_NOT_FILLED)
			e = c;
		else
			s = c;
	} while (e-s > 1);
	buf_len = e;

	buf_len /= CHANNELS;
	buf_len *= CHANNELS;

	if (buf_len == 0)
		error(2, 1);

	//uint8_t *new_writebuf = ((old_writebuf == writebuf[0]) ? writebuf[1] : writebuf[0]);

	uint16_t src_points = buf_len / CHANNELS;
	fill_writebuf(writebuf, parsebuf, 0, src_points);

	return writebuf;
}

void sd_write(SD_HandleTypeDef *hsd, uint32_t block, sdword_t *buf) {
	int err;

	GPIOA->BSRR = GPIO_PIN_5;
	int try = 0;

	while ((err = HAL_SD_WriteBlocks(hsd, (uint32_t *)buf, SD_BLOCKSIZE*block, SD_BLOCKSIZE, 1)) != SD_OK && try++ < 120);

	if (err != SD_OK)
		error(err, 0);

	GPIOA->BSRR = GPIO_PIN_5 << 16;

	return;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();

  /* USER CODE BEGIN 2 */
	//HAL_ADC_MspInit(&hadc1);
	HAL_MspInit();
	HAL_SD_MspInit(&hsd);

	//int err;
/*
	if ((err = HAL_ADC_Start(&hadc1)) != HAL_OK) {
		error(400, 0);
	}

	*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	GPIOA->BSRR = GPIO_PIN_5;
	HAL_Delay(100);
	GPIOA->BSRR = GPIO_PIN_5 << 16;
	HAL_Delay(100);

	uint8_t forceDetails = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET;

	memset(writebuf, 0, SD_BLOCKSIZE);

#define S(a) a,sizeof(a)
#if ADCMODE >= 2
	memcpy(writebuf, S("\000voltlogger\000\000\017\024\000NucleoF411SD000\000"));
#else
	memcpy(writebuf, S("\000voltlogger\000\002\016\024\000NucleoF411SD000\000"));
#endif
#undef S

	if (!forceDetails)
		writebuf[12] |= 0x04;

	sd_write(&hsd, 0, writebuf);

	{
		uint32_t i = 0;
		while (adcbuf[i] != ADC_NOT_FILLED && i < BUF_LEN) adcbuf[i++] = ADC_NOT_FILLED;
	}
	adc_restart();

	GPIOA->BSRR = GPIO_PIN_5;
	HAL_Delay(100);
	GPIOA->BSRR = GPIO_PIN_5 << 16;
	HAL_Delay(100);

	static int block = 1;
	while (1)
	{

		sdword_t *cur_writebuf;

		if (forceDetails)
			cur_writebuf = fill_writebuf_detailed();
		else
			cur_writebuf = fill_writebuf_inseparable();

		sd_write(&hsd, block, cur_writebuf);

		block++;



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 14;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 7;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 8;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 9;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 10;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 11;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 12;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 13;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 14;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;
  HAL_SD_Init(&hsd, &SDCardInfo);

  HAL_SD_WideBusOperation_Config(&hsd, SDIO_BUS_WIDE_4B);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
