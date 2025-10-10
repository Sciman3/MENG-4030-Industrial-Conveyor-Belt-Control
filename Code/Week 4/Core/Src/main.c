/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "task.h"
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

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart3;

/* Definitions for sensorReadTaskH */
osThreadId_t sensorReadTaskHHandle;
const osThreadAttr_t sensorReadTaskH_attributes = {
  .name = "sensorReadTaskH",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for contTaskHandle */
osThreadId_t contTaskHandleHandle;
const osThreadAttr_t contTaskHandle_attributes = {
  .name = "contTaskHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for actuatorTaskH */
osThreadId_t actuatorTaskHHandle;
const osThreadAttr_t actuatorTaskH_attributes = {
  .name = "actuatorTaskH",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* USER CODE BEGIN PV */
uint8_t ADXL_Bytes [1];
static const uint8_t ADDR = 0x1D<<1; /*Shift for 7-bit addresses*/
static const uint8_t DATA_FORMAT = 0x31;
static const uint8_t PWRCTL = 0x2D;
static const uint8_t DATA_X0 = 0x32;
float acc_Data[3] = {0};
int state;
HAL_StatusTypeDef ret;
		uint8_t REGISTER;
		uint8_t DATA;
		uint8_t DATA_WRITE[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void sensorReadTask(void *argument);
void contTask(void *argument);
void actuatorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void INIT_Accel(){
	REGISTER = DATA_FORMAT;
			    DATA = 0x00;
			    DATA_WRITE[0] = REGISTER;
			    DATA_WRITE[1] = DATA;
			    ret = HAL_I2C_Master_Transmit(&hi2c1,ADDR,DATA_WRITE,2,HAL_MAX_DELAY);

			    REGISTER = PWRCTL;
			    DATA = 0x00;
			    DATA_WRITE[0] = REGISTER;
			    DATA_WRITE[1] = DATA;
			    ret = HAL_I2C_Master_Transmit(&hi2c1,ADDR,DATA_WRITE,2,HAL_MAX_DELAY);

			    REGISTER = PWRCTL;
			    DATA = 0x08;
			    DATA_WRITE[0] = REGISTER;
			    DATA_WRITE[1] = DATA;
			    ret = HAL_I2C_Master_Transmit(&hi2c1,ADDR,DATA_WRITE,2,HAL_MAX_DELAY);
}
void ACC_ReadAccel(float *sensor_data) {

		    REGISTER = DATA_X0;
		    	  	uint8_t RAWACCEL[6];

		    	  	//ret = HAL_I2C_Master_Transmit(&hi2c1,ADDR,&REGISTER,1,HAL_MAX_DELAY);
		    	  	//HAL_Delay(1);
		    	  	//ret = HAL_I2C_Master_Receive(&hi2c1,ADDR,RAWACCEL,6,HAL_MAX_DELAY);

		    	  	HAL_I2C_Mem_Read(&hi2c1, ADDR, DATA_X0, I2C_MEMADD_SIZE_8BIT, RAWACCEL, 6, HAL_MAX_DELAY);


		    	  	int16_t x = (int16_t)((RAWACCEL[1]<<8)|RAWACCEL[0]);
		    	  	int16_t y = (int16_t)((RAWACCEL[3]<<8)|RAWACCEL[2]);
		    	  	int16_t z = (int16_t)((RAWACCEL[5]<<8)|RAWACCEL[4]);

		    	  	char DebugTxMsg[43];
		    	  	snprintf(DebugTxMsg, sizeof(DebugTxMsg), "Prescaled: X: %d Y: %d Z: %d \n", x, y, z);
		    	  	HAL_UART_Transmit(&huart3,(uint8_t*)DebugTxMsg, strlen(DebugTxMsg),30);

		    	  	float x_scaled = x * 0.0039f;
		    	  	float y_scaled = y * 0.0039f;
		    	  	float z_scaled = z * 0.0039f;

		    	  	sensor_data[0]=x_scaled;
		    	  	sensor_data[1]=y_scaled;
		    	  	sensor_data[2]=z_scaled;

}
void sim_Act_Output(int trigger_State){
	if (trigger_State == 1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	}
}

void traceTaskSwitch(void){
	uint8_t buf1[50];
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	snprintf(buf1, sizeof(buf1), "Current task: %s\r\n", pcTaskGetName(NULL));
	HAL_UART_Transmit(&huart3,(uint8_t*)buf1, strlen(buf1),30);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of sensorReadTaskH */
  sensorReadTaskHHandle = osThreadNew(sensorReadTask, NULL, &sensorReadTaskH_attributes);

  /* creation of contTaskHandle */
  contTaskHandleHandle = osThreadNew(contTask, NULL, &contTaskHandle_attributes);

  /* creation of actuatorTaskH */
  actuatorTaskHHandle = osThreadNew(actuatorTask, NULL, &actuatorTaskH_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10707DBC;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_LED_Pin|Red_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Green_LED_Pin Red_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|Red_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Yellow_LED_Pin */
  GPIO_InitStruct.Pin = Yellow_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Yellow_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_sensorReadTask */
/**
  * @brief  Function implementing the sensorReadTaskH thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_sensorReadTask */
void sensorReadTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	INIT_Accel();
  /* Infinite loop */
  for(;;)
  {
	  ACC_ReadAccel(acc_Data);

	  char TxMsg[40];
	  snprintf(TxMsg, sizeof(TxMsg), "Scaled: X: %3.2f Y: %3.2f Z: %3.2f \n\n", acc_Data[0], acc_Data[1], acc_Data[2]);
	  HAL_UART_Transmit(&huart3,(uint8_t*)TxMsg, strlen(TxMsg),30);
	  //HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);
	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_contTask */
/**
* @brief Function implementing the contTaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_contTask */
void contTask(void *argument)
{
  /* USER CODE BEGIN contTask */
  /* Infinite loop */
  for(;;)
  {
	  if (fabs(acc_Data[0]) > 1 || fabs(acc_Data[1]) > 1 || fabs(acc_Data[2]) > 1){
		  state = 1;
		  char DebugTxMsg[59];
	      snprintf(DebugTxMsg, sizeof(DebugTxMsg), "Large acceleration detected, initiating jam clear routine.");
	      HAL_UART_Transmit(&huart3,(uint8_t*)DebugTxMsg, strlen(DebugTxMsg),30);
	  }
	  else {
	      	 state = 0;
	  }
	sim_Act_Output(state);
	//HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_1);
    osDelay(10);
  }
  /* USER CODE END contTask */
}

/* USER CODE BEGIN Header_actuatorTask */
/**
* @brief Function implementing the actuatorTaskH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_actuatorTask */
void actuatorTask(void *argument)
{
  /* USER CODE BEGIN actuatorTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);
    osDelay(400);
  }
  /* USER CODE END actuatorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
