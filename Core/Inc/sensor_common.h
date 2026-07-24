/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor_common.h
  * @brief   Sensor abstraction layer header
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

#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
  SENSOR_TYPE_IMU = 0,
  SENSOR_TYPE_GPS,
  SENSOR_TYPE_COUNT
} SensorType_t;

typedef enum {
  SENSOR_STATUS_OK = 0,
  SENSOR_STATUS_ERROR,
  SENSOR_STATUS_TIMEOUT,
  SENSOR_STATUS_NOT_INITIALIZED,
  SENSOR_STATUS_FAULT
} SensorStatus_t;

typedef struct {
  SensorType_t type;
  SensorStatus_t status;
  uint32_t error_count;
  uint32_t success_count;
  TickType_t last_update_time;
  uint8_t is_initialized;
  uint8_t retry_count;
} SensorInfo_t;

/* Function prototypes -------------------------------------------------------*/
void Sensor_Init(SensorInfo_t *info, SensorType_t type);
void Sensor_UpdateStatus(SensorInfo_t *info, SensorStatus_t status);
void Sensor_NotifyUpdate(SensorInfo_t *info);
uint8_t Sensor_IsDataFresh(SensorInfo_t *info, uint32_t stale_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_COMMON_H */

