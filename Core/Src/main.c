/* USER CODE BEGIN Header */
/**

TODO:
-	iButton interface

Vmot remeasured on PA1 named ADC1_IN1

field feedback measured on PA2 named ADC_IN2

OLED SSD1306
	Resolution: 128 x 64
	Input Voltage: 3.3V ~ 6V
	SDA on PB7 named I2C1_SDA
	SCL on PB6 named I2C1_SCL

BlackPill onboard Blue LED on PC13 named LED

BlackPill onboard button on PA0 named KEY

Optical encoder
	channel one on PA6 named TIM3_CH1
	channel two on PA7 named TIM3_CH2

A4988 stepper motor driver with chopping killed
	S_ENABLE	on	PB12
	MS1			on	PB13
	MS2			on	PB14
	MS3			on	PB15
	S_RESET		on	PA10
	S_SLEEP		on	B0
	STEP		on	B1
	DIR			on	B2
	The step limit is in the "counter_limit" constant.

1-Wire interface on USART2
	OW	on 	PA2	named	USART2_TX
	SN: 01/0000129A55D6/F3

RS232 on
	tx	on  PA11	named USART6_TX
	rx	on  PA12	named USART6_RX


  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
#include "stdio.h"
#include "OneWire.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

OledHandleTypedef oled = {
  .type = SH1106_I2C,        // SH1106 or SSD1306
  .orientation = 1,           // 1: downward 0: upward
  .i2c = &hi2c1,              // I2C1 at RB9/RB8
  .i2c_address = 0x78         // 0x78 (0x3c<<1) or 0x7A (0x3D<<1)
};

uint16_t size;
uint8_t Data[256];
int count=0;
int motor_position =0;
char text[10] = "empty",spi_out_buf[10]="",spi_in_buf[10]="";
int raw;
float voltage;
uint32_t counter = 0;
int16_t counter_position = 0;
uint32_t us_delay = 0;
const int counter_limit=15300; // put maximum counter value here
int rec_rom_id;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern float Temp[MAXDEVICES_ON_THE_BUS];

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
	counter = __HAL_TIM_GET_COUNTER(htim) / 2;
	if (counter > 30000)
	{
		__HAL_TIM_SetCounter(&htim3,0);
		counter=0;
	}
	if (counter > counter_limit)
	{
		__HAL_TIM_SetCounter(&htim3, counter_limit * 2);
		counter=counter_limit;
	}
	counter_position = (int16_t)counter;
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start_IT (&htim3, TIM_CHANNEL_ALL   );
  /* Initialize stepper motor controller */
  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET); // set high for half step
  HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET); // set for microstep
  HAL_GPIO_WritePin(MS3_GPIO_Port, MS3_Pin, GPIO_PIN_RESET); // set for microstep
  HAL_GPIO_WritePin(S_SLEEP_GPIO_Port, S_SLEEP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(S_ENABLE_GPIO_Port, S_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(S_RESET_GPIO_Port, S_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(S_RESET_GPIO_Port, S_RESET_Pin, GPIO_PIN_SET);

// OLED init
  SSD1306_Init(&oled);
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("SH1106", &Font_11x18, SSD1306_COLOR_WHITE);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  raw = HAL_ADC_GetValue(&hadc1);
  voltage = raw /  187.3 ; // Umcu = 3.375V
  sprintf(text, "%1.2f", voltage);
  SSD1306_GotoXY (0, 20);
  SSD1306_Puts (text, &Font_11x18, 1);
  SSD1306_Puts ("V", &Font_11x18, 1);
//  SSD1306_dim(127);
  SSD1306_UpdateScreen(); // update screen
  HAL_Delay(1000);

  OneWire_Init();
  OneWire_SetCallback();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  while (motor_position != counter_position)
	  {
		  HAL_GPIO_WritePin(S_ENABLE_GPIO_Port, S_ENABLE_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  if (motor_position < counter_position)
		  {

			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
			  motor_position = motor_position + 1 ;
			  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
			  us_delay = 7500;
			  while (us_delay) { us_delay = us_delay - 1;}
		  }
		  else
		  {

			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
			  motor_position = motor_position - 1 ;
			  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
			  us_delay = 7500;
			  while (us_delay) { us_delay = us_delay - 1;}
		  }
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(S_ENABLE_GPIO_Port, S_ENABLE_Pin, GPIO_PIN_SET);
	  }


//	  sprintf(text, "%5i", counter_position);
//	  SSD1306_GotoXY(0, 30);
//	  SSD1306_Puts (text, &Font_11x18, 1);
//	  SSD1306_UpdateScreen();

	  rec_rom_id = get_ROMid();
	  SSD1306_Fill(0);
	  SSD1306_GotoXY(0, 0);
//	  sprintf(text, "%s %u", ow.ids);
	  //sprintf(textout1, "%s %u", name1, var1);
	  SSD1306_Puts (text, &Font_11x18, 1);
	  SSD1306_UpdateScreen();

	  HAL_Delay (1000);


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
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 104;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S_SLEEP_Pin|STEP_Pin|DIR_Pin|S_ENABLE_Pin
                          |MS1_Pin|MS2_Pin|MS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S_RESET_GPIO_Port, S_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S_SLEEP_Pin STEP_Pin DIR_Pin S_ENABLE_Pin
                           MS1_Pin MS2_Pin MS3_Pin */
  GPIO_InitStruct.Pin = S_SLEEP_Pin|STEP_Pin|DIR_Pin|S_ENABLE_Pin
                          |MS1_Pin|MS2_Pin|MS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : S_RESET_Pin */
  GPIO_InitStruct.Pin = S_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_RESET_GPIO_Port, &GPIO_InitStruct);

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
