/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "semphr.h"
#include "event_groups.h"
#include "control.h"
#include "bmi160_wrapper.h"
#include "HighPerformanceTimer.h"
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
/* USER CODE BEGIN Variables */

TaskHandle_t imuTaskHandle;
TaskHandle_t pidTaskHandle;
TaskHandle_t shellTaskHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  DWT_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  globalMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  shellQueue = xQueueCreate(2, sizeof(SHELLMSG));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(imuTask, "IMU Task", 128, NULL, osPriorityNormal, &imuTaskHandle);
  xTaskCreate(pidTask, "PID Task", 128, NULL, osPriorityRealtime, &pidTaskHandle);
  xTaskCreate(shellTask, "Shell Task", 512, NULL, osPriorityBelowNormal, &shellTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  eventGroup = xEventGroupCreate();
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  if(BMI160_init() == 0)  // succeed
  {
    bmi160Gyro_Zero_Calibration(&imu_t);
    xEventGroupSetBits(eventGroup, EVENT_BMI160_INITIALIZED);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);  // enable STBY
    xEventGroupSetBits(eventGroup, EVENT_MOTOR_ENABLED);
  }

  /* Infinite loop */
  for(; ; )
  {
    HAL_GPIO_TogglePin(boardLED_GPIO_Port, boardLED_Pin);
    HAL_ADC_Start(&hadc1);
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 5);
    if (HAL_OK == status)
    {
      uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
      float vcc = adc_value * (3.3f * 11.f / 4095.f);
      xTaskNotify(shellTaskHandle, *(int*)(&vcc), eSetValueWithOverwrite);
      TickType_t delayTicks = 1 + (369U * (uint32_t)adc_value) / 500U;
      vTaskDelay(delayTicks);
    }
    else
      vTaskDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && GPIO_Pin == BMI_INT1_Pin)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(eventGroup, EVENT_INTERRUPT_TRIGGERED, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
/* USER CODE END Application */

