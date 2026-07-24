/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    health.c
  * @brief   System health monitoring implementation
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
#include "health.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Private types -------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
HealthData_t health_data;
osMutexId health_mutex = NULL;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize health monitoring system
  * @retval None
  */
void Health_Init(void)
{
  /* Mutex is created in MX_FREERTOS_Init() before scheduler starts */
  /* Just initialize the data structure */
  memset(&health_data, 0, sizeof(HealthData_t));
  health_data.last_imu_update = xTaskGetTickCount();
  health_data.last_gps_update = xTaskGetTickCount();
}

/**
  * @brief  Notify that IMU data was updated
  * @retval None
  */
void Health_NotifyIMUUpdated(void)
{
  if (osMutexWait(health_mutex, pdMS_TO_TICKS(10)) == osOK) {
    health_data.last_imu_update = xTaskGetTickCount();
    health_data.imu_update_count++;
    osMutexRelease(health_mutex);
  }
}

/**
  * @brief  Notify that GPS data was updated
  * @retval None
  */
void Health_NotifyGPSUpdated(void)
{
  if (osMutexWait(health_mutex, pdMS_TO_TICKS(10)) == osOK) {
    health_data.last_gps_update = xTaskGetTickCount();
    health_data.gps_update_count++;
    osMutexRelease(health_mutex);
  }
}

/**
  * @brief  Update health status
  * @retval None
  */
void Health_Update(void)
{
  TickType_t current_time = xTaskGetTickCount();
  TickType_t imu_stale_ticks = pdMS_TO_TICKS(IMU_STALE_DATA_MS);
  TickType_t gps_stale_ticks = pdMS_TO_TICKS(GPS_STALE_DATA_MS);
  
  if (osMutexWait(health_mutex, pdMS_TO_TICKS(10)) == osOK) {
    /* Check IMU health */
    health_data.imu_healthy = 
      ((current_time - health_data.last_imu_update) <= imu_stale_ticks) ? 1 : 0;
    
    /* Check GPS health */
    health_data.gps_healthy = 
      ((current_time - health_data.last_gps_update) <= gps_stale_ticks) ? 1 : 0;
    
    /* Note: sys_status.imu_ok / gps_ok / system_ok are set by the
       IMU and Telemetry tasks under MUTEX_TELEMETRYHandle.
       We do NOT write sys_status here to avoid a data race. */
    
    osMutexRelease(health_mutex);
  }
}

/**
  * @brief  Check if IMU is healthy
  * @retval 1 if healthy, 0 if not
  */
uint8_t Health_IsIMUHealthy(void)
{
  uint8_t healthy = 0;
  if (osMutexWait(health_mutex, pdMS_TO_TICKS(10)) == osOK) {
    healthy = health_data.imu_healthy;
    osMutexRelease(health_mutex);
  }
  return healthy;
}

/**
  * @brief  Check if GPS is healthy
  * @retval 1 if healthy, 0 if not
  */
uint8_t Health_IsGPSHealthy(void)
{
  uint8_t healthy = 0;
  if (osMutexWait(health_mutex, pdMS_TO_TICKS(10)) == osOK) {
    healthy = health_data.gps_healthy;
    osMutexRelease(health_mutex);
  }
  return healthy;
}

/**
  * @brief  Check if system is healthy
  * @retval 1 if healthy, 0 if not
  */
uint8_t Health_IsSystemHealthy(void)
{
  return sys_status.system_ok;
}

