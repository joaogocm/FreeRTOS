/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId defaultTaskHandle;
osThreadId Led01TaskHandle;
osThreadId Led02TaskHandle;
osThreadId Led03TaskHandle;
osMutexId mutex01Handle;

int HPC, MPC, LPC = 0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void vLed01Task(void const * argument);
void vLed02Task(void const * argument);
void vLed03Task(void const * argument);

GPIO gpio1(GPIOA, 12);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of mutex01 */
  osMutexDef(mutex01);
  mutex01Handle = osMutexCreate(osMutex(mutex01));

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Led01Task */
  osThreadDef(Led01Task, vLed01Task, osPriorityHigh, 0, 128);
  Led01TaskHandle = osThreadCreate(osThread(Led01Task), NULL);

  /* definition and creation of Led02Task */
  osThreadDef(Led02Task, vLed02Task, osPriorityNormal, 0, 128);
  Led02TaskHandle = osThreadCreate(osThread(Led02Task), NULL);

  /* definition and creation of Led03Task */
  osThreadDef(Led03Task, vLed03Task, osPriorityLow, 0, 128);
  Led03TaskHandle = osThreadCreate(osThread(Led03Task), NULL);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED2_Pin|LED1_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_vLed01Task */
/**
* @brief Function implementing the Led01TaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vLed01Task */
void vLed01Task(void const * argument)
{
  /* USER CODE BEGIN vLed01Task */
  /* Infinite loop */
  for(;;)
  {
	  //osThreadResume(Led02TaskHandle);
	  //osThreadResume(Led03TaskHandle);
	  if(osMutexWait(mutex01Handle, 2) != osOK)
	  {
		  HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
	  }

	  osDelay(20);

	  if(osMutexRelease(mutex01Handle) != osOK)
	  {
		  HAL_GPIO_TogglePin(GPIOD,LED1_Pin);
	  }

	  HPC++;
	  HAL_GPIO_TogglePin(GPIOD, LED2_Pin);

	  osThreadSuspend(NULL);
  }
  /* USER CODE END vLed01Task */
}

/* USER CODE BEGIN Header_vLed02Task */
/**
* @brief Function implementing the Led02Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vLed02Task */
void vLed02Task(void const * argument)
{
  /* USER CODE BEGIN vLed02Task */
  /* Infinite loop */
  for(;;)
  {
	  if (osMutexWait(mutex01Handle, 5000) == osOK)
	  {
		  if (osThreadGetState(Led01TaskHandle) != osThreadSuspended)
		  {
			  HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
		  }
		  else
		  {
			  if (osMutexRelease(mutex01Handle) != osOK)
			  {
				  HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
			  }

			  MPC++;
			  HAL_GPIO_TogglePin(GPIOD, LED3_Pin);
			  osThreadSuspend(NULL);
		  }
	  }
	  else
	  {
		  HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
	  }
  }
  /* USER CODE END vLed02Task */
}

/* USER CODE BEGIN Header_vLed03Task */
/**
* @brief Function implementing the Led03Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vLed03Task */
void vLed03Task(void const * argument)
{
  /* USER CODE BEGIN vLed03Task */
  /* Infinite loop */
  for(;;)
  {
    if (osMutexWait(mutex01Handle, 0) == osOK)
    {
    	if((osThreadGetState(Led01TaskHandle) != osThreadSuspended) || (osThreadGetState(Led02TaskHandle) != osThreadSuspended))
    	{
    		HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
    	}
    	else
    	{
    		osThreadResume(Led01TaskHandle);
    		osThreadResume(Led02TaskHandle);

    		if (osMutexRelease(mutex01Handle) != osOK)
    		{
    			HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
    		}

    		LPC++;
    		HAL_GPIO_TogglePin(GPIOD, LED4_Pin);
    		//osThreadSuspend(NULL);
    	}
    }
  }
  /* USER CODE END vLed03Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
