/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    health.h
  * @brief   System health monitoring header
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

#ifndef HEALTH_H
#define HEALTH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "system_config.h"
#include "telemetry_globals.h"
#include <stdint.h>
#include <string.h>

/* Exported types */
typedef struct {
  TickType_t last_imu_update;
  TickType_t last_gps_update;
  uint32_t imu_update_count;
  uint32_t gps_update_count;
  uint8_t imu_healthy;
  uint8_t gps_healthy;
} HealthData_t;

/* Exported variables */
extern HealthData_t health_data;
extern osMutexId health_mutex;

/* Function prototypes -------------------------------------------------------*/
void Health_Init(void);
void Health_NotifyIMUUpdated(void);
void Health_NotifyGPSUpdated(void);
void Health_Update(void);
uint8_t Health_IsIMUHealthy(void);
uint8_t Health_IsGPSHealthy(void);
uint8_t Health_IsSystemHealthy(void);

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_H */

