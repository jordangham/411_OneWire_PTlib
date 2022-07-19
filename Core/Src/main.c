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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ds18b20.h"

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
uint8_t ROMBuffer[10]="empty";
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
//  OneWire_SetCallback();


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


	  sprintf(text, "%5i", counter_position);
	  SSD1306_Fill(0);
	  SSD1306_GotoXY(0, 0);
	  SSD1306_Puts (text, &Font_11x18, 1);
	  SSD1306_UpdateScreen();

	  OneWire_Init();

/*
	  OneWire_Init();
	  OneWire_SetCallback(DS18B20_OnComplete, DS18B20_Error);
	  OneWire_Execute(0xcc,0,0,0); // skip rom phase
	  OneWire_Execute(0xcc,0,0x44,0); // start to Convert T
	  HAL_Delay(1100); // Wait to convertion time
	  OneWire_Execute(0xcc,0,0,0); // skip rom phase
	  OneWire_Execute(0xcc,0,0xbe,&(FunctionBuffer[0])); // start to read configuration & result
*/


	  HAL_Delay (100);


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
