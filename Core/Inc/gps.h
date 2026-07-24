/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gps.h
  * @brief   GPS NMEA parser header
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

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  /* For size_t */

/* Exported types ------------------------------------------------------------*/
/* GPS parsed data structure (simplified - full version in telemetry_globals.h) */
typedef struct {
  double latitude;       /* Decimal degrees */
  double longitude;      /* Decimal degrees */
  float altitude;        /* Meters */
  uint8_t fix;           /* Fix quality (0=invalid, 1=GPS, 2=DGPS) */
  float speed_ms;        /* Speed in m/s */
  float course_deg;      /* Course in degrees (0=North, 90=East) */
  
  /* Quality Metrics */
  float hdop;            /* Horizontal Dilution of Precision */
  float vdop;            /* Vertical Dilution of Precision */
  uint8_t satellites;    /* Number of satellites */
  uint8_t confidence;    /* Estimated confidence (0-100%) */
} GPS_ParsedData_t;

/* Function prototypes -------------------------------------------------------*/
HAL_StatusTypeDef GPS_ParseBuffer(uint8_t *buf, size_t len, GPS_ParsedData_t *out);
double GPS_NMEA_ToDecimal(double nmea_coord, char hemisphere);
void GPS_CalculateVelocity(double lat1, double lon1, double lat2, double lon2,
                           float dt, float *vx, float *vy, float *speed);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */

