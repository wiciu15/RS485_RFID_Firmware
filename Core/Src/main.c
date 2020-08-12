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
#include "rc522.h"
#include <stdio.h>
#include <string.h>
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	CONN_TIMEOUT,
	UID_UNKNOWN,
	READY,
	CONN_IN_PROGRESS,
	WAITING_FOR_RESPONSE,
	UID_VALID,
	ERROR_READER,
	ERROR_SQL,
	ERROR_HTTP,
	ERROR_SERVER,
	ERROR_WIFI}device_status_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONNECTION_TIMEOUT  5  //timeout in seconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId ReceiveDataHandle;
osThreadId StatusReportingHandle;
/* USER CODE BEGIN PV */
uint8_t cardIDarray [5];
uint32_t lastCardIDdec=0;
uint32_t lastRFReadSystick=0;
device_status_typedef device_status=READY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void vReceiveDataTask(void const * argument);
void vStatusReporting(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  //HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_ALL);


  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ReceiveData */
  osThreadDef(ReceiveData, vReceiveDataTask, osPriorityNormal, 0, 1024);
  ReceiveDataHandle = osThreadCreate(osThread(ReceiveData), NULL);

  /* definition and creation of StatusReporting */
  osThreadDef(StatusReporting, vStatusReporting, osPriorityNormal, 0, 128);
  StatusReportingHandle = osThreadCreate(osThread(StatusReporting), NULL);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUGLED_GPIO_Port, DEBUGLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin|BUZZER_Pin 
                          |RS485_TXENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RC522_RESET_GPIO_Port, RC522_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUGLED_Pin */
  GPIO_InitStruct.Pin = DEBUGLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUGLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin BUZZER_Pin 
                           RS485_TXENABLE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin|BUZZER_Pin 
                          |RS485_TXENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC522_RESET_Pin */
  GPIO_InitStruct.Pin = RC522_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RC522_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC522_IRQ_Pin */
  GPIO_InitStruct.Pin = RC522_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RC522_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUGBTN_Pin */
  GPIO_InitStruct.Pin = DEBUGBTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DEBUGBTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Main task polling reader for new card UID to process
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	char UARTInitMsg[10];
	  snprintf(UARTInitMsg,10,"INIT\r\n");
	  HAL_GPIO_WritePin(RS485_TXENABLE_GPIO_Port, RS485_TXENABLE_Pin, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart1, (uint8_t *) UARTInitMsg, strlen(UARTInitMsg),20);
	  HAL_GPIO_WritePin(RS485_TXENABLE_GPIO_Port, RS485_TXENABLE_Pin, GPIO_PIN_RESET);
	/* Infinite loop */
	for(;;)
	{
		if(MFRC522_Check(cardIDarray)==MI_OK){
			HAL_GPIO_WritePin(DEBUGLED_GPIO_Port, DEBUGLED_Pin,GPIO_PIN_SET);
			BUZZ_Enable();
			uint32_t cardIDdec;
			cardIDdec =  (uint32_t)cardIDarray[0] << 24;
			cardIDdec += (uint32_t)cardIDarray[1] << 16;
			cardIDdec += (uint32_t)cardIDarray[2] <<  8;
			cardIDdec += (uint32_t)cardIDarray[3];
			if(cardIDdec!=lastCardIDdec || (lastRFReadSystick+7000 < xTaskGetTickCount()) || lastRFReadSystick>xTaskGetTickCount()){ //dont accept 2 reads of the same card within 10 sec(ignore if systick overflowed)

				lastCardIDdec=cardIDdec;
				lastRFReadSystick=xTaskGetTickCount();
				device_status=CONN_IN_PROGRESS;
				osDelay(200);
				HAL_GPIO_WritePin(DEBUGLED_GPIO_Port, DEBUGLED_Pin,GPIO_PIN_RESET);
				BUZZ_Disable();
				vTaskResume(ReceiveDataHandle);
				vTaskSuspend(defaultTaskHandle);
			}
			else{
				osDelay(5);
				HAL_GPIO_WritePin(DEBUGLED_GPIO_Port, DEBUGLED_Pin,GPIO_PIN_RESET);
				BUZZ_Disable();
			}
		}

		osDelay(100);
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_vReceiveDataTask */
/**
* @brief Function implementing ReceiveData thread which handles communication and device state changes
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vReceiveDataTask */
void vReceiveDataTask(void const * argument)
{
	/* USER CODE BEGIN vReceiveDataTask */
	vTaskSuspend(ReceiveDataHandle); //suspended by default until new UID is read in DefaultTask thread
	/* Infinite loop */
	for(;;)
	{
		uint8_t UARTReceiveBuffer=8;
		uint8_t* pUARTReceiveBuffer=&UARTReceiveBuffer;
		HAL_UART_Receive(&huart1, pUARTReceiveBuffer, 1, 100); //start receiver and read anything that was in UART already

		//////SEND PING AND THEN CARD UID VIA RS485////
		//__HAL_UART_FLUSH_DRREGISTER(&huart1);
		char UARTSendBuffer[20];
		snprintf(UARTSendBuffer,20,"PING\r\n");
		HAL_GPIO_WritePin(RS485_TXENABLE_GPIO_Port, RS485_TXENABLE_Pin, GPIO_PIN_SET); //enable TX on transceiver
		osDelay(2);
		HAL_UART_Transmit(&huart1, (uint8_t *) UARTSendBuffer, strlen(UARTSendBuffer),15); //transmit in blocking mode - no need to implement DMA or interrupts
		HAL_GPIO_WritePin(RS485_TXENABLE_GPIO_Port, RS485_TXENABLE_Pin, GPIO_PIN_RESET); //RS485 transceiver is high impedance (listening for response)

		osDelay(5); //wait for indoor unit to process and reply

		switch(HAL_UART_Receive(&huart1, pUARTReceiveBuffer, 1, 100)){
		case HAL_TIMEOUT:
			device_status=CONN_TIMEOUT;
			break;
		case HAL_ERROR:
			device_status=ERROR_READER;
			break;
		case HAL_OK:
			if(UARTReceiveBuffer=='9'){ //indoor unit responds with '9' if its ready to process new UID
				snprintf(UARTSendBuffer,20,"ID%lu\r\n",lastCardIDdec);
				HAL_GPIO_WritePin(RS485_TXENABLE_GPIO_Port, RS485_TXENABLE_Pin, GPIO_PIN_SET); //enable TX on transceiver
				HAL_UART_Transmit(&huart1, (uint8_t *) UARTSendBuffer, strlen(UARTSendBuffer),15); //transmit in blocking mode - no need to implement DMA or interrupts
				osDelay(10);
				HAL_GPIO_WritePin(RS485_TXENABLE_GPIO_Port, RS485_TXENABLE_Pin, GPIO_PIN_RESET); //RS485 transceiver is high impedance (listening for response)

				device_status=WAITING_FOR_RESPONSE;
				__HAL_UART_FLUSH_DRREGISTER(&huart1);
				switch(HAL_UART_Receive(&huart1, pUARTReceiveBuffer, 1, 8000)){ //receiving in blocking mode to use 8s timeout if there is a problem within indoor unit
				case HAL_TIMEOUT:
					device_status=CONN_TIMEOUT;
					break;
				case HAL_OK:
					//device_status=READY;
					switch(UARTReceiveBuffer){      /////////set reader state depending on received code from indoor unit
					case '1': 						//UID OK
						device_status=UID_VALID;
						break;
					case '0':						//UID unknown/intruder
						device_status=UID_UNKNOWN;
						break;
					case '2':						//SQL error
						device_status=ERROR_SQL;
						break;
					case '3':						//HTTP ERROR
						device_status=ERROR_HTTP;
						break;
					case '4':						//SERVER ERROR
						device_status=ERROR_SERVER;
						break;
					case '5':						//WIFI ERROR
						device_status=ERROR_WIFI;
						break;
					default:
						device_status=ERROR_READER;
						break;
					}
					break;
					case HAL_ERROR:
						device_status=ERROR_READER;
						break;
					default:
						device_status=READY;
						break;
				}
			}
			else{
				device_status=ERROR_READER;
				break;
			}
			break;
		}

		vTaskResume(defaultTaskHandle);
		vTaskSuspend(ReceiveDataHandle); //task suspended, default task is now in charge
	}
  /* USER CODE END vReceiveDataTask */
}

/* USER CODE BEGIN Header_vStatusReporting */
/**
* @brief Function implementing the StatusReporting thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStatusReporting */
void vStatusReporting(void const * argument)
{
  /* USER CODE BEGIN vStatusReporting */
	osDelay(100);
	LED_Init();
  /* Infinite loop */
	for(;;)
	{
		 // HAL_TIM_StateTypeDef timstate = HAL_TIM_PWM_GetState(&htim2);
		switch(device_status){
		case READY:
			LED_SetColor(BLUE);
			osDelay(3);
			LED_Clear();
			break;
		case CONN_IN_PROGRESS:
			LED_Clear();
			for(uint8_t i=0;i<3;i++){
				LED_SetColor(BLUE);
				osDelay(50);
				LED_Clear();
				osDelay(200);
			}
			break;
		case WAITING_FOR_RESPONSE:
			LED_Clear();
			for(uint8_t i=0;i<3;i++){
				LED_SetColor(BLUE);
				osDelay(50);
				LED_Clear();
				osDelay(200);
			}
			break;
		case UID_VALID:
			vTaskSuspend(defaultTaskHandle);
			LED_Clear();
			LED_SetColor(GREEN);
			BUZZ_Enable();
			osDelay(2000);
			device_status=READY;
			BUZZ_Disable();
			vTaskResume(defaultTaskHandle);
			break;
////ERROR signalling with blinking RED led and buzzer/////
		case UID_UNKNOWN:
			vTaskSuspend(defaultTaskHandle);
			LED_SignalDeviceError(1); //1 beep for unknown
			device_status=READY;
			vTaskResume(defaultTaskHandle);
			break;
		case ERROR_SQL:
			vTaskSuspend(defaultTaskHandle);
			LED_SignalDeviceError(2); //2 beeps for SQL error
			device_status=READY;
			vTaskResume(defaultTaskHandle);
			break;
		case ERROR_HTTP:
			vTaskSuspend(defaultTaskHandle);
			LED_SignalDeviceError(3); //3 beeps for HTTP error
			device_status=READY;
			vTaskResume(defaultTaskHandle);
			break;
		case ERROR_SERVER:
			vTaskSuspend(defaultTaskHandle);
			LED_SignalDeviceError(4); //4 beeps for server error
			device_status=READY;
			vTaskResume(defaultTaskHandle);
			break;
		case ERROR_WIFI:
			vTaskSuspend(defaultTaskHandle);
			LED_SignalDeviceError(5); //5 beeps for wifi error
			device_status=READY;
			vTaskResume(defaultTaskHandle);
			break;
		default:
			vTaskSuspend(defaultTaskHandle);
			LED_SignalDeviceError(6); //6 beeps for other problems
			device_status=READY;
			vTaskResume(defaultTaskHandle);
			break;
		}
		osDelay(10);
	}
  /* USER CODE END vStatusReporting */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
