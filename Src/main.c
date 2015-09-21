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
#define SD_CHANNELS 14 /* should be a power of 2 minus 2 and greater or equal to CHANNELS */
#define WRITEBUFS 1
#define SD_BLOCKSIZE 512
#define BUF_LEN (SD_BLOCKSIZE<<5)

uint16_t buf[BUF_LEN] = {0,1,2,3,4,5,6,7,8,9,10};
uint16_t tmpbuf[BUF_LEN];

int adc_running = 0;

uint8_t writebuf[WRITEBUFS][SD_BLOCKSIZE];	// Two buffers is for DMA (in future)
uint8_t *old_writebuf = writebuf[WRITEBUFS-1];

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
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
	if (!adc_running)
		return;

	//error(1, 1);
	//NVIC_SystemReset();

	return;
}

void HAL_ADC_ErrorCallback (ADC_HandleTypeDef* hadc) {
	error(30, 1);
	NVIC_SystemReset();

	return;
}

uint8_t *switch_writebuf() {
	adc_running = 0;
	HAL_ADC_Stop_DMA(&hadc1);

	// searching the end of the filled part of the buffer
	uint32_t buf_len;
	uint32_t s = 0, e = BUF_LEN;
	do {
		uint32_t c = (e+s)/2;
		if (buf[c] == 0xffff)
			e = c;
		else
			s = c;
	} while (e-s > 1);
	buf_len = e;

	memcpy(tmpbuf, buf,  buf_len*sizeof(*buf));
	memset(buf,    0xff, buf_len*sizeof(*buf));

	int err;
	adc_running = 1;
	if ((err = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf, BUF_LEN)) != HAL_OK) {
		error(400, 0);
	}

	buf_len /= CHANNELS;
	buf_len *= CHANNELS;

	if (buf_len == 0)
		error(2, 1);

	//uint8_t *new_writebuf = ((old_writebuf == writebuf[0]) ? writebuf[1] : writebuf[0]);
	uint8_t *new_writebuf = writebuf[0];

	uint16_t src_points = buf_len / CHANNELS;
	uint16_t dst_points = SD_BLOCKSIZE / (SD_CHANNELS + 2);

	float k = (float)src_points / (float)dst_points;

	if (k < 1) {
		error(src_points, 1);
		NVIC_SystemReset();	// Too slow ADC
	}

	uint32_t dst       = 0, src       = 0;
	uint16_t dst_point = 0, src_point = 0;
	while (dst < SD_BLOCKSIZE) {

		new_writebuf[dst++] = src_point;//ts[src_point];
		new_writebuf[dst++] = src_point >> 8;//ts[src_point] << 8;

		int chan = 0;
		do {
			new_writebuf[dst++] = tmpbuf[src++] >> 4;
		} while (++chan < CHANNELS);

#if CHANNELS < SD_CHANNELS
		while (chan < SD_CHANNELS) {
			new_writebuf[dst++] = 0;
			chan++;
		}
#endif

		dst_point++;

		src_point = (float)dst_point * k + 0.0001;
		src       = (float)src_point * (float)CHANNELS + 0.0001;
	}

	old_writebuf = new_writebuf;

	return new_writebuf;
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

	int err;
/*
	if ((err = HAL_ADC_Start(&hadc1)) != HAL_OK) {
		error(400, 0);
	}

	*/

	memset(buf, 0xff, sizeof(buf));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	if ((err = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf, BUF_LEN)) != HAL_OK) {
		error(400, 0);
	}

	GPIOA->BSRR = GPIO_PIN_5;
	HAL_Delay(100);
	GPIOA->BSRR = GPIO_PIN_5 << 16;
	HAL_Delay(100);

	static int block = 0;
	while (1)
	{

		uint8_t *cur_writebuf = switch_writebuf();
		GPIOA->BSRR = GPIO_PIN_5;
		int try = 0;

		while ((err = HAL_SD_WriteBlocks(&hsd, (uint32_t *)cur_writebuf, SD_BLOCKSIZE*block, SD_BLOCKSIZE, 1)) != SD_OK && try++ < 120);

		if (err != SD_OK)
			error(err, 0);

		GPIOA->BSRR = GPIO_PIN_5 << 16;

		block++;

		/*
		GPIOA->BSRR = GPIO_PIN_5;
		if ((err = HAL_SD_WriteBlocks_DMA(&hsd, (uint32_t *)&buf, block++, 512, 1)) != SD_OK) {
			error(err, 1);
		}
		GPIOA->BSRR = GPIO_PIN_5 << 16;
		HAL_Delay(500);
		if ((err = HAL_SD_CheckWriteOperation(&hsd, 0xFFFF)) != SD_OK) {
			error(err, 0);
		}
		*/

		//error(30, 1);


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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
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
  hsd.Init.ClockDiv = 10;
  HAL_SD_Init(&hsd, &SDCardInfo);

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
