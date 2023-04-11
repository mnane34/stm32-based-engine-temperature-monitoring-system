/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "FIRFilter.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct sensor{
	float resistor;
	float resistorLogaritmic;
	float refResistor;
	float refVoltage;
	float steinhartA;
	float steinhartB;
	float steinhartC;
	float analogSignal;

	int temperature;
	int tempTemperature;

}sensor_t;

sensor_t sensor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
FIRFilter barFilterMovingAverage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define bufferSize 10
#define txBuffer 20
#define preBuffer 100

uint32_t adcValue[bufferSize] = {0};
uint32_t averageValue = 0;
unsigned long int summingValue = 0;
uint8_t sendMessage[txBuffer];

void determineCoefficients(void);
void convertTemperature(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1){

		for(int counter = 0; counter<bufferSize; counter++){
			summingValue += adcValue[counter];
		}
		averageValue = (uint32_t)(summingValue / bufferSize);
		summingValue = 0;
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
	 sensor.refResistor = 9870.0;
	 sensor.refVoltage = 3.37;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, &adcValue[0], bufferSize);

  FIRFilter_Init(&barFilterMovingAverage);

  for(int counter = 0; counter<preBuffer; counter++){

  	sensor.analogSignal = (averageValue * sensor.refVoltage) / 4095.0;
  	sensor.resistor = ((sensor.refResistor * (sensor.refVoltage-sensor.analogSignal)) / (sensor.analogSignal));
  	determineCoefficients();
  	convertTemperature();
 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	sensor.analogSignal = (averageValue * sensor.refVoltage) / 4095.0;
	sensor.resistor = ((sensor.refResistor * (sensor.refVoltage-sensor.analogSignal)) / (sensor.analogSignal));
	determineCoefficients();
	convertTemperature();
	sprintf((char*)sendMessage, "|%d|\n", sensor.temperature);
	HAL_UART_Transmit(&huart1, sendMessage, sizeof(sendMessage), 1000);
	memset(sendMessage, 0, sizeof(sendMessage));
	HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void convertTemperature(void){

 	sensor.resistorLogaritmic = log(sensor.resistor);
 	sensor.tempTemperature = (int)( 1 / (sensor.steinhartA + (sensor.steinhartB * sensor.resistorLogaritmic) + (sensor.steinhartC * sensor.resistorLogaritmic * sensor.resistorLogaritmic * sensor.resistorLogaritmic)));
 	sensor.tempTemperature -= 273;
 	sensor.temperature = FIRFilter_Update(&barFilterMovingAverage, sensor.tempTemperature);
}

void determineCoefficients(void){

	if(sensor.resistor >= 13018.0 && sensor.resistor <= 23342.0){
		sensor.steinhartA = 1.390630395e-03;
		sensor.steinhartB = 2.805660005e-04;
		sensor.steinhartC = 0.7520202377e-07;
	}
	else if (sensor.resistor >= 5855.0 && sensor.resistor < 13018.0){
		sensor.steinhartA = 1.393819878e-03;
		sensor.steinhartB = 2.800689802e-04;
		sensor.steinhartC = 0.7702209069e-07;
	}
	else if (sensor.resistor >= 2854.0 && sensor.resistor < 5855.0){
		sensor.steinhartA = 1.356809063e-03;
		sensor.steinhartB = 2.868265492e-04;
		sensor.steinhartC = 0.4378552036e-07;
	}
	else if (sensor.resistor >= 1491.0 && sensor.resistor < 2854.0){
		sensor.steinhartA = 1.373696154e-03;
		sensor.steinhartB = 2.840852189e-04;
		sensor.steinhartC = 0.5343930830e-07;
	}
	else if(sensor.resistor >= 826.6 && sensor.resistor < 1491.0){
		sensor.steinhartA = 1.410553889e-03;
		sensor.steinhartB = 2.765768030e-04;
		sensor.steinhartC = 0.9992682963e-07;
	}
	else if(sensor.resistor >= 482.7 && sensor.resistor < 826.6){
		sensor.steinhartA = 1.462092077e-03;
		sensor.steinhartB = 2.644119345e-04;
		sensor.steinhartC = 1.999218291e-07;
	}
	else if(sensor.resistor >= 293.7 && sensor.resistor < 482.7){
		sensor.steinhartA = 1.494842800e-03;
		sensor.steinhartB = 2.565749327e-04;
		sensor.steinhartC = 2.659751121e-07;
	}
	else if(sensor.resistor >= 184.7 && sensor.resistor < 293.7){
		sensor.steinhartA = 1.584648860e-03;
		sensor.steinhartB = 2.325539830e-04;
		sensor.steinhartC = 5.209143518e-07;
	}
	else if(sensor.resistor >= 119.4 && sensor.resistor < 184.7){
		sensor.steinhartA = 1.556142469e-03;
		sensor.steinhartB = 2.412217354e-04;
		sensor.steinhartC = 4.013575752e-07;
	}
	else if(sensor.resistor >= 79.0 && sensor.resistor < 119.4){
		sensor.steinhartA = 1.622693990e-03;
		sensor.steinhartB = 2.204903946e-04;
		sensor.steinhartC = 6.993132700e-07;
	}
	else if(sensor.resistor >= 53.32 && sensor.resistor < 79.0){
		sensor.steinhartA = 1.640897658e-03;
		sensor.steinhartB = 2.141247478e-04;
		sensor.steinhartC = 8.149252608e-07;
	}
	else if(sensor.resistor >= 36.63 && sensor.resistor < 53.32){
		sensor.steinhartA = 1.626967742e-03;
		sensor.steinhartB = 2.199439949e-04;
		sensor.steinhartC = 6.665851264e-07;
	}
	else if(sensor.resistor >= 25.56 && sensor.resistor < 36.63){
		sensor.steinhartA = 1.673744484e-03;
		sensor.steinhartB = 2.003603190e-04;
		sensor.steinhartC = 11.78163989e-07;
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
