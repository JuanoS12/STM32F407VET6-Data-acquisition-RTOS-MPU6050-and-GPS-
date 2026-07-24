/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor_common.c
  * @brief   Sensor abstraction layer implementation
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
#include "sensor_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize sensor info structure
  * @param  info: Sensor info structure pointer
  * @param  type: Sensor type
  * @retval None
  */
void Sensor_Init(SensorInfo_t *info, SensorType_t type)
{
  if (info == NULL) return;
  
  memset(info, 0, sizeof(SensorInfo_t));
  info->type = type;
  info->status = SENSOR_STATUS_NOT_INITIALIZED;
  info->is_initialized = 0;
}

/**
  * @brief  Update sensor status
  * @param  info: Sensor info structure pointer
  * @param  status: New status
  * @retval None
  */
void Sensor_UpdateStatus(SensorInfo_t *info, SensorStatus_t status)
{
  if (info == NULL) return;
  
  info->status = status;
  
  if (status == SENSOR_STATUS_OK) {
    info->success_count++;
    info->is_initialized = 1;
  } else {
    info->error_count++;
  }
}

/**
  * @brief  Notify that sensor data was updated
  * @param  info: Sensor info structure pointer
  * @retval None
  */
void Sensor_NotifyUpdate(SensorInfo_t *info)
{
  if (info == NULL) return;
  
  info->last_update_time = xTaskGetTickCount();
  info->status = SENSOR_STATUS_OK;
  info->success_count++;
}

/**
  * @brief  Check if sensor data is fresh
  * @param  info: Sensor info structure pointer
  * @param  stale_timeout_ms: Timeout in milliseconds
  * @retval 1 if fresh, 0 if stale
  */
uint8_t Sensor_IsDataFresh(SensorInfo_t *info, uint32_t stale_timeout_ms)
{
  TickType_t current_time;
  TickType_t stale_ticks;
  
  if (info == NULL || !info->is_initialized) {
    return 0;
  }
  
  current_time = xTaskGetTickCount();
  stale_ticks = pdMS_TO_TICKS(stale_timeout_ms);
  
  if ((current_time - info->last_update_time) > stale_ticks) {
    return 0;
  }
  
  return 1;
}

