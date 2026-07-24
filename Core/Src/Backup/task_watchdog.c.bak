/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    task_watchdog.c
  * @brief   Task watchdog system implementation
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
#include "task_watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Private types -------------------------------------------------------------*/
typedef struct {
  char name[16];
  TickType_t last_feed_time;
  uint8_t is_registered;
  uint8_t is_alive;
} TaskWatchdogEntry_t;

/* Private variables ---------------------------------------------------------*/
static TaskWatchdogEntry_t monitored_tasks[MAX_MONITORED_TASKS];
static osMutexId watchdog_mutex = NULL;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize task watchdog system
  * @retval None
  */
void TaskWatchdog_Init(void)
{
  osMutexDef(MUTEX_WATCHDOG);
  watchdog_mutex = osMutexCreate(osMutex(MUTEX_WATCHDOG));
  memset(monitored_tasks, 0, sizeof(monitored_tasks));
}

/**
  * @brief  Register a task with the watchdog
  * @param  task_name: Task name string
  * @retval None
  */
void TaskWatchdog_Register(const char *task_name)
{
  if (task_name == NULL) return;
  
  if (osMutexWait(watchdog_mutex, pdMS_TO_TICKS(100)) == osOK) {
    for (int i = 0; i < MAX_MONITORED_TASKS; i++) {
      if (!monitored_tasks[i].is_registered) {
        strncpy(monitored_tasks[i].name, task_name, sizeof(monitored_tasks[i].name) - 1);
        monitored_tasks[i].name[sizeof(monitored_tasks[i].name) - 1] = '\0';
        monitored_tasks[i].last_feed_time = xTaskGetTickCount();
        monitored_tasks[i].is_registered = 1;
        monitored_tasks[i].is_alive = 1;
        break;
      }
    }
    osMutexRelease(watchdog_mutex);
  }
}

/**
  * @brief  Feed the watchdog for a task
  * @param  task_name: Task name string
  * @retval None
  */
void TaskWatchdog_Feed(const char *task_name)
{
  if (task_name == NULL) return;
  
  if (osMutexWait(watchdog_mutex, pdMS_TO_TICKS(10)) == osOK) {
    for (int i = 0; i < MAX_MONITORED_TASKS; i++) {
      if (monitored_tasks[i].is_registered && 
          strcmp(monitored_tasks[i].name, task_name) == 0) {
        monitored_tasks[i].last_feed_time = xTaskGetTickCount();
        monitored_tasks[i].is_alive = 1;
        break;
      }
    }
    osMutexRelease(watchdog_mutex);
  }
}

/**
  * @brief  Get count of dead tasks
  * @retval Number of dead tasks
  */
uint8_t TaskWatchdog_GetDeadTaskCount(void)
{
  uint8_t dead_count = 0;
  TickType_t current_time = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(TASK_WATCHDOG_TIMEOUT_MS);
  
  if (osMutexWait(watchdog_mutex, pdMS_TO_TICKS(10)) == osOK) {
    for (int i = 0; i < MAX_MONITORED_TASKS; i++) {
      if (monitored_tasks[i].is_registered) {
        if ((current_time - monitored_tasks[i].last_feed_time) > timeout_ticks) {
          monitored_tasks[i].is_alive = 0;
          dead_count++;
        }
      }
    }
    osMutexRelease(watchdog_mutex);
  }
  
  return dead_count;
}

/**
  * @brief  Check if a task is alive
  * @param  task_name: Task name string
  * @retval 1 if alive, 0 if dead
  */
uint8_t TaskWatchdog_IsTaskAlive(const char *task_name)
{
  uint8_t is_alive = 0;
  
  if (task_name == NULL) return 0;
  
  if (osMutexWait(watchdog_mutex, pdMS_TO_TICKS(10)) == osOK) {
    for (int i = 0; i < MAX_MONITORED_TASKS; i++) {
      if (monitored_tasks[i].is_registered && 
          strcmp(monitored_tasks[i].name, task_name) == 0) {
        is_alive = monitored_tasks[i].is_alive;
        break;
      }
    }
    osMutexRelease(watchdog_mutex);
  }
  
  return is_alive;
}

