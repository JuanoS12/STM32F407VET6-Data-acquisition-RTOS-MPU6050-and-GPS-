/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    telemetry_globals.c
  * @brief   Global shared data structures implementation
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
#include "telemetry_globals.h"
#include "system_config.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
IMU_Data_t imu_data;
GPS_Data_t gps_data;
System_Status_t sys_status;

osMutexDef(MUTEX_I2C);
osMutexDef(MUTEX_TELEMETRY);

osMutexId MUTEX_I2CHandle;
osMutexId MUTEX_TELEMETRYHandle;

StreamBufferHandle_t SB_GPS;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize global data structures and synchronization objects
  * @retval None
  */
void TelemetryGlobals_Init(void)
{
  /* NOTE: All mutex/stream creation is handled in MX_FREERTOS_Init().
     This function is intentionally empty to avoid duplicate creation.
     Data structures (imu_data, gps_data, sys_status) are also 
     initialized in MX_FREERTOS_Init() before scheduler start. */
}

