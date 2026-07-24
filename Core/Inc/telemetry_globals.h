/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    telemetry_globals.h
  * @brief   Global shared data structures and mutexes
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

#ifndef TELEMETRY_GLOBALS_H
#define TELEMETRY_GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "stream_buffer.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
  /* Orientation & Movement */
  float ax, ay, az;          /* Accelerometer (m/s²) - Body Frame */
  float gx, gy, gz;          /* Gyroscope (deg/s) - Body Frame */
  float pitch, roll, yaw;    /* Euler angles (degrees) */
  float pitch_rate, roll_rate, yaw_rate; /* Rates (deg/s) - same as gx,gy,gz but explicit */
  
  /* Global Acceleration (World Frame) */
  float ax_global;           /* North/South accel (m/s²) */
  float ay_global;           /* East/West accel (m/s²) */
  float az_global;           /* Vertical accel (m/s²) */
  
  /* G-Forces */
  float g_total;             /* Total G-force */
  float g_long;              /* Longitudinal G-force */
  float g_lat;               /* Lateral G-force */
  float g_vert;              /* Vertical G-force */
  
  /* Advanced Dynamics */
  float vibration_level;     /* Vibration magnitude (0-10) */
  
} IMU_Data_t;

typedef struct {
  double latitude;           /* Decimal degrees */
  double longitude;          /* Decimal degrees */
  float altitude;            /* Meters */
  uint8_t fix;               /* Fix quality (0=invalid, 1=GPS, 2=DGPS) */
  float velocity_x;          /* North-South velocity (m/s) */
  float velocity_y;          /* East-West velocity (m/s) */
  float speed;               /* Total speed (m/s) */
  
  /* Quality Metrics */
  float hdop;                /* Horizontal Dilution of Precision */
  float vdop;                /* Vertical Dilution of Precision */
  float course;              /* Course over ground (degrees) */
  uint8_t satellites;        /* Number of satellites */
  uint8_t confidence;        /* GPS confidence (0-100%) */
  
} GPS_Data_t;

typedef struct {
  /* System Status */
  uint8_t imu_ok;            /* IMU status (1=OK, 0=ERROR) */
  uint8_t gps_ok;            /* GPS status (1=OK, 0=ERROR) */
  uint8_t system_ok;         /* Overall system status */
  
  /* Vehicle State */
  uint8_t state;             /* 0=Stop, 1=Accel, 2=Brake, 3=Turn, 4=Air */
  float slip_angle;          /* Estimated slip angle (deg) */
  
  /* Contact & Load */
  uint8_t wheel_contact;     /* Bitmask: FL, FR, RL, RR */
  float wheel_load[4];       /* Load per wheel (N) */
  
} System_Status_t;

/* Exported constants --------------------------------------------------------*/
extern IMU_Data_t imu_data;
extern GPS_Data_t gps_data;
extern System_Status_t sys_status;

/* Exported mutexes ---------------------------------------------------------*/
extern osMutexId MUTEX_I2CHandle;
extern osMutexId MUTEX_TELEMETRYHandle;

/* Exported stream buffers --------------------------------------------------*/
extern StreamBufferHandle_t SB_GPS;

/* Function prototypes ------------------------------------------------------*/
void TelemetryGlobals_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_GLOBALS_H */

