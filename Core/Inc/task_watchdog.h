/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    task_watchdog.h
  * @brief   Task watchdog system header
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

#ifndef TASK_WATCHDOG_H
#define TASK_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "system_config.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define MAX_MONITORED_TASKS  10

/* Function prototypes -------------------------------------------------------*/
void TaskWatchdog_Init(void);
void TaskWatchdog_Register(const char *task_name);
void TaskWatchdog_Feed(const char *task_name);
uint8_t TaskWatchdog_GetDeadTaskCount(void);
uint8_t TaskWatchdog_IsTaskAlive(const char *task_name);

#ifdef __cplusplus
}
#endif

#endif /* TASK_WATCHDOG_H */

