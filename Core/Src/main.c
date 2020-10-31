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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_5110.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct leitorAD{
	int32_t buffer[10];
	int length;
	int current;
};
struct leitorAD ad1;
struct leitorAD ad2;
char buffer[50];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

osThreadId sensor01TaskHandle;
osThreadId sensor02TaskHandle;
osThreadId displayTaskHandle;
osMutexId MutexS01Handle;
osMutexId MutexS02Handle;
osMutexId MutexSerialHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartSensor01Task(void const * argument);
void StartSensor02Task(void const * argument);
void StartdisplayTask(void const * argument);

/* USER CODE BEGIN PFP */
static int32_t adRead(ADC_HandleTypeDef* hadc);
static void init(struct leitorAD* this);
static void next(struct leitorAD* this);
static void writeBuffer(struct leitorAD* this,int num);
static int calcMedia(struct leitorAD* this);
static int getTemp(struct leitorAD* this);
static int difTemp(struct leitorAD* this, struct leitorAD* other);
static int diftep(int a,int b){int n = a-b;return n<0?-n:n;}
static void writeSafe(struct leitorAD* this,osMutexId* mutex,ADC_HandleTypeDef* hadc);
static int getSafe(struct leitorAD* this,osMutexId* mutex);
static void writeSerialSafe(int line,char* word);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  init(&ad1);
  init(&ad2);
//  LCD_Init();
//  LCD_Write_String(0,0, "teste");
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MutexS01 */
  osMutexDef(MutexS01);
  MutexS01Handle = osMutexCreate(osMutex(MutexS01));

  /* definition and creation of MutexS02 */
  osMutexDef(MutexS02);
  MutexS02Handle = osMutexCreate(osMutex(MutexS02));

  /* definition and creation of MutexSerial */
  osMutexDef(MutexSerial);
  MutexSerialHandle = osMutexCreate(osMutex(MutexSerial));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of sensor01Task */
  osThreadDef(sensor01Task, StartSensor01Task, osPriorityNormal, 0, 128);
  sensor01TaskHandle = osThreadCreate(osThread(sensor01Task), NULL);

  /* definition and creation of sensor02Task */
  osThreadDef(sensor02Task, StartSensor02Task, osPriorityIdle, 0, 128);
  sensor02TaskHandle = osThreadCreate(osThread(sensor02Task), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartdisplayTask, osPriorityIdle, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	  HAL_Delay(1000);
//	  writeBuffer(&ad1, adRead(&hadc1));
//	  sprintf(buffer,"%04d C",getTemp(&ad1));
//	  LCD_Write_String(0, 1, buffer);
//	  writeBuffer(&ad2, adRead(&hadc2));
//	  sprintf(buffer,"%04d C",getTemp(&ad2));
//	  LCD_Write_String(0, 2, buffer);
//	  HAL_Delay(200);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CE_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static int32_t adRead(ADC_HandleTypeDef* hadc){
	HAL_ADC_Start(hadc);
	while(HAL_ADC_PollForConversion(hadc, 5)!= HAL_OK){}
	return HAL_ADC_GetValue(hadc);
}
static void next(struct leitorAD* this){
	this->current=(this->current+1)%this->length;
}
static void init(struct leitorAD* this){
	this->current =0;
	this->length =10;
	for(int i =0 ;i<this->length;i++){
		this->buffer[i]=0;
	}
}
static void writeBuffer(struct leitorAD* this,int num){
	this->buffer[this->current]=num;
	next(this);
}
static int calcMedia(struct leitorAD* this){
	int soma =0;
	for(int i = 0; i<this->length;i++){
		soma += this->buffer[i];
	}
	return soma/this->length;
}
static int getTemp(struct leitorAD* this){
	return 100*calcMedia(this)/4095;
}
static int difTemp(struct leitorAD* this, struct leitorAD* other){
	int val = getTemp(this)-getTemp(other);
	return val<0?-val:val;
}
static void writeSafe(struct leitorAD* this,osMutexId* mutex,ADC_HandleTypeDef* hadc){
	osMutexWait(*mutex, 100);
	writeBuffer(this, adRead(hadc));
	osMutexRelease(*mutex);
}
static int getSafe(struct leitorAD* this,osMutexId* mutex){
	osMutexWait(*mutex, 150);
	int temp = getTemp(this);
	osMutexRelease(*mutex);
	return temp;
}
static void writeSerialSafe(int line,char* word){
	osMutexWait(MutexSerialHandle, 100);
	LCD_Write_String(0, line, word);
	osMutexRelease(MutexSerialHandle);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensor01Task */
/**
  * @brief  Function implementing the sensor01Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensor01Task */
void StartSensor01Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  writeSafe(&ad1, &MutexS01Handle, &hadc1);
	  osDelay(300);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensor02Task */
/**
* @brief Function implementing the sensor02Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor02Task */
void StartSensor02Task(void const * argument)
{
  /* USER CODE BEGIN StartSensor02Task */
  /* Infinite loop */
  for(;;)
  {
	writeSafe(&ad2, &MutexS02Handle, &hadc2);
    osDelay(300);
  }
  /* USER CODE END StartSensor02Task */
}

/* USER CODE BEGIN Header_StartdisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartdisplayTask */
void StartdisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartdisplayTask */
  /* Infinite loop */
	LCD_Init();
	LCD_Write_String(0, 0, "Temperatura:");
  for(;;)
  {
	int t1= getSafe(&ad1, &MutexS01Handle);
	int t2= getSafe(&ad2, &MutexS02Handle);
	sprintf(buffer,"T1 %03d C",t1);
	writeSerialSafe(1, buffer);
	sprintf(buffer,"T2 %03d C",t2);
	writeSerialSafe(2, buffer);
	if(diftep(t1, t2)>10){
		writeSerialSafe(3, "Alerta");
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	}else{
		writeSerialSafe(3, "------");
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	}
	osDelay(5000);
  }
  /* USER CODE END StartdisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
