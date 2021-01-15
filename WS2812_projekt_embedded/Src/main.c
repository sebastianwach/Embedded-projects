/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WS2812B_LEDS 30


#define zero 0b1000000
#define one 0b1111000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
typedef struct ws2812b_color {
	uint8_t red, green, blue;
} ws2812b_color;

static ws2812b_color ws2812b_array[WS2812B_LEDS];			// global led structure array to store rgb values;
const uint16_t BUFFER_LENGTH = WS2812B_LEDS * 24 + 48; 		// length of total buffer 30 pixels* 24 bytes(8 bytes for each colour)+ 24 bytes to reset
static uint8_t buffer[WS2812B_LEDS * 24 + 48];				// buffer with this length
uint16_t CurrentLed;

uint8_t state; 												// state of inbuilt button
uint8_t encoderValue = 0;									// encoderValue [0 - 80]
uint8_t encoderPreviousValue = 0;
uint8_t encoderDifference = 0;
static uint16_t globalCounter = 0;							// global counter to change values depend from time
uint8_t mode = 0;											// selected LED mode
uint8_t modeMax = 4;										// number of modes in application
uint8_t brightness = 255;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Button_Pin) {
		state = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state);
	}

	if (GPIO_Pin == EncButton_Pin) {
		{
			mode++;
			if (mode >= modeMax) {
				mode = 0;
			}
		}
	}
}



void WS2812B_SetDiodeRGB(uint8_t diode_id, uint8_t R, uint8_t G, uint8_t B) {
	if (diode_id >= WS2812B_LEDS || diode_id < 0)
		return;
	ws2812b_array[diode_id].red = R;
	ws2812b_array[diode_id].green = G;
	ws2812b_array[diode_id].blue = B;
}
void WS2812B_SetDiodeHSV(int16_t diode_id, uint16_t Hue, uint8_t Saturation,
		uint8_t Brightness) {
	if (diode_id >= WS2812B_LEDS || diode_id < 0)
		return;
	uint16_t Sector, Fracts, p, q, t;

	if (Saturation == 0) {
		ws2812b_array[diode_id].red = Brightness;
		ws2812b_array[diode_id].green = Brightness;
		ws2812b_array[diode_id].blue = Brightness;
	} else {
		if (Hue >= 360)
			Hue = 359;

		Sector = Hue / 60; // Sector 0 to 5
		Fracts = Hue % 60;
		p = (Brightness * (255 - Saturation)) / 256;
		q = (Brightness * (255 - (Saturation * Fracts) / 60)) / 256;
		t = (Brightness * (255 - (Saturation * (59 - Fracts)) / 60)) / 256;

		switch (Sector) {
		case 0:
			ws2812b_array[diode_id].red = Brightness;
			ws2812b_array[diode_id].green = (uint8_t) t;
			ws2812b_array[diode_id].blue = (uint8_t) p;
			break;
		case 1:
			ws2812b_array[diode_id].red = (uint8_t) q;
			ws2812b_array[diode_id].green = Brightness;
			ws2812b_array[diode_id].blue = (uint8_t) p;
			break;
		case 2:
			ws2812b_array[diode_id].red = (uint8_t) p;
			ws2812b_array[diode_id].green = Brightness;
			ws2812b_array[diode_id].blue = (uint8_t) t;
			break;
		case 3:
			ws2812b_array[diode_id].red = (uint8_t) p;
			ws2812b_array[diode_id].green = (uint8_t) q;
			ws2812b_array[diode_id].blue = Brightness;
			break;
		case 4:
			ws2812b_array[diode_id].red = (uint8_t) t;
			ws2812b_array[diode_id].green = (uint8_t) p;
			ws2812b_array[diode_id].blue = Brightness;
			break;
		default:		// case 5:
			ws2812b_array[diode_id].red = Brightness;
			ws2812b_array[diode_id].green = (uint8_t) p;
			ws2812b_array[diode_id].blue = (uint8_t) q;
			break;
		}
	}
}
void WS2812B_CleanBuffer() {
	for (uint8_t i = 0; i < WS2812B_LEDS; i++) {
		WS2812B_SetDiodeRGB(i, 0, 0, 0);
	}
}

void WS2812B_RefreshStrip() {
	uint8_t i = 0;
	for (CurrentLed = 0; CurrentLed < WS2812B_LEDS; CurrentLed++) {
		//green
		for (i = 0; i < 8; i++) {
			if ((ws2812b_array[CurrentLed].green & (0x80 >> i)) == 0)
				buffer[CurrentLed * 24 + i] = zero;
			else
				buffer[CurrentLed * 24 + i] = one;
		}
		// red
		for (i = 0; i < 8; i++) {
			if ((ws2812b_array[CurrentLed].red & (0x80 >> i)) == 0)
				buffer[CurrentLed * 24 + i + 8] = zero;
			else
				buffer[CurrentLed * 24 + i + 8] = one;
		}
		//blue
		for (i = 0; i < 8; i++) {
			if ((ws2812b_array[CurrentLed].blue & (0x80 >> i)) == 0)
				buffer[CurrentLed * 24 + i + 16] = zero;
			else
				buffer[CurrentLed * 24 + i + 16] = one;
		}

	}
	for (uint8_t k = 0; k < 24; k++) {
		buffer[WS2812B_LEDS * 24 + k] = 0x00;
	}
	HAL_SPI_Transmit_DMA(&hspi2, buffer, BUFFER_LENGTH);
	while (HAL_DMA_STATE_READY != HAL_DMA_GetState(hspi2.hdmatx));
}
void WS2812B_OnePixelRoundMode(uint32_t delay, uint8_t R, uint8_t G, uint8_t B) {
	for (int j = 0; j < WS2812B_LEDS; j++) {
		WS2812B_CleanBuffer();
		WS2812B_SetDiodeRGB(j, R, G, B);
		WS2812B_RefreshStrip();
		HAL_Delay(delay);
	}
}

void WS2812B_RainbowMode(uint8_t encSpeed) {
	globalCounter++;
	for (int i = 0; i < WS2812B_LEDS; i++) {

		WS2812B_SetDiodeHSV(i, (i * 12 + globalCounter) % 360, 255, brightness);
	}
	WS2812B_RefreshStrip();
	HAL_Delay(encSpeed);
}

void WS2812B_EncoderMode(uint16_t encHue, uint16_t encMaxValue) {
	for (int i = 0; i < WS2812B_LEDS; i++) {

		WS2812B_SetDiodeHSV(i, ((360 / encMaxValue) * encHue) % 360, 255, brightness);
	}
	WS2812B_RefreshStrip();
}
void ChangeBrightness(uint8_t encValue) {
	brightness = encValue;
}

void lcd_print_enc_value(uint8_t encValue, uint8_t row, uint8_t col) {
	if ( encValue >= 100)
	{
		lcd_put_cur(row, col);
		lcd_send_data(((encValue / 100) % 10) + 48);
	}else
	{
		lcd_put_cur(row, col);
		lcd_send_data(32);  //just print space, not '0'
	}
	lcd_put_cur(row, col+1);
	lcd_send_data(((encValue / 10) % 10) + 48);
	lcd_put_cur(row, col + 2);
	lcd_send_data(encValue % 10 + 48);
}

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
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  lcd_init();
  lcd_send_string("MODE");
  HAL_Delay(500);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		encoderValue = htim1.Instance->CNT;
		encoderDifference = encoderValue - encoderPreviousValue;
		if(abs(encoderDifference)> 10)
		{
			encoderDifference = 0;
		}

		switch (mode) {
		case 0:
			lcd_put_cur(1, 0);
			lcd_send_string("Rainbow     ");
			lcd_print_enc_value(encoderValue, 1, 12);
			WS2812B_RainbowMode(encoderValue);
			break;

		case 1:
			lcd_put_cur(1, 0);
			lcd_send_string("Round pixel");
			lcd_print_enc_value(encoderValue, 1, 12);
			WS2812B_OnePixelRoundMode(encoderValue, 255, 255, 255);
			break;

		case 2:
			lcd_put_cur(1, 0);
			lcd_send_string("Encoder    ");
			lcd_print_enc_value(encoderValue, 1, 12);
			WS2812B_EncoderMode(encoderValue, 80);
			break;

		case 3:
			lcd_put_cur(1, 0);
			lcd_send_string("Brightness");
			brightness += encoderDifference;
			lcd_print_enc_value(brightness, 1, 12);
			WS2812B_EncoderMode(encoderValue, 80);
			break;

		default:

			break;
		}
		encoderPreviousValue = encoderValue;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 80;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EncButton_Pin */
  GPIO_InitStruct.Pin = EncButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EncButton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
