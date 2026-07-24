/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    system_config.h
  * @brief   System-wide configuration parameters
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

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Task Stack Sizes (in words, 4 bytes each) */
#define STACK_SIZE_SENSOR          512   /* Sensor tasks (IMU, GPS) */
#define STACK_SIZE_PROCESSING      384   /* Data processing (UART Debug) */
#define STACK_SIZE_COMMUNICATION   256   /* SPI, UART */
#define STACK_SIZE_LOW             128   /* LED Task */
#define STACK_SIZE_HEALTH          256   /* Health Task */

/* Watchdog Configuration */
#define WATCHDOG_TIMEOUT_MS        2000  /* 2 second timeout */
#define TASK_WATCHDOG_TIMEOUT_MS   5000  /* 5 seconds */

/* Error Handling */
#define ERROR_RECOVERY_RETRIES     3     /* Retries before giving up */

/* Communication */
#define I2C_TIMEOUT_MS            200    /* I2C operation timeout (increased for reliability) */
#define SPI_TIMEOUT_MS            100    /* SPI operation timeout */
#define UART_TIMEOUT_MS           10     /* UART operation timeout */

/* Task Frequencies */
#define IMU_TASK_FREQ_HZ          10     /* IMU update rate */
#define GPS_TASK_FREQ_HZ          1      /* GPS parsing rate (variable) */
#define TELEMETRY_TASK_FREQ_HZ    10     /* Telemetry transmission rate */
#define UART_DEBUG_TASK_FREQ_HZ   0.5f   /* UART debug output rate (reduced for cleaner output) */
#define LED_TASK_FREQ_HZ          1      /* LED blink rate */
#define HEALTH_TASK_FREQ_HZ       2      /* Health check rate */

/* GPS Configuration */
#define GPS_RX_BUF_LEN            512    /* GPS receive buffer size (increased for longer sentences) */
#define GPS_STREAM_BUF_SIZE       512    /* GPS stream buffer size */

/* Filter Parameters */
#define COMPLEMENTARY_FILTER_ALPHA 0.90f  /* Complementary filter coefficient (lowered from 0.98 to reduce drift) */
#define KALMAN_Q                  0.05f  /* Process noise */
#define KALMAN_R_GPS              1.0f   /* GPS measurement noise */
#define KALMAN_R_IMU              0.2f   /* IMU measurement noise */

/* SPI Packet Configuration */
#define SPI_PACKET_START_BYTE     0xAA
#define SPI_PACKET_END_BYTE       0x55
#define SPI_PACKET_TYPE_TELEMETRY 0x01
/* SPI Packet Structure (Total: 79 bytes)
   Header: 3 bytes (Start 0xAA + Type 0x01 + Len)
   Payload: 74 bytes
    - Basic Info: 6 bytes (t_ms u32 + seq u16)
    - Orientation: 12 bytes (pitch, roll, yaw as float)
    - Raw Accel: 12 bytes (ax, ay, az as float)
    - Raw Gyro: 12 bytes (gx, gy, gz as float)
    - GPS Speed: 4 bytes (speed as float)
    - GPS Position: 20 bytes (lat f64 + lon f64 + alt f32)
    - Status: 8 bytes (imu_ok, gps_ok, fix, sats, hdop f32)
   Checksum: 1 byte (XOR of bytes 0..76)
   End: 1 byte (0x55)
*/
#define SPI_PACKET_PAYLOAD_LEN    74
#define SPI_PACKET_TOTAL_LEN      79  /* Start + Type + Len + Payload + Checksum + End */

/* Sensor Status Thresholds */
#define IMU_STALE_DATA_MS         2000   /* IMU data considered stale after 2s */
#define GPS_STALE_DATA_MS         5000   /* GPS data considered stale after 5s */

/* Vehicle State Detection Thresholds */
#define STATE_ACCEL_THRESHOLD     0.15f  /* Longitudinal G threshold for acceleration */
#define STATE_BRAKE_THRESHOLD    -0.15f  /* Longitudinal G threshold for braking */
#define STATE_TURN_THRESHOLD      0.2f   /* Lateral G threshold for turning */
#define STATE_AIRBORNE_G_HIGH     1.5f   /* Vertical G threshold for airborne (high) */
#define STATE_AIRBORNE_G_LOW      0.5f   /* Vertical G threshold for airborne (low) */
#define STATE_MIN_SPEED_MPS       1.0f   /* Minimum speed for state detection (m/s) */

/* Vibration Filter Constants */
#define VIBRATION_FILTER_ALPHA    0.95f  /* High-pass filter alpha for vibration detection */
#define VIBRATION_SMOOTH_ALPHA    0.9f   /* Smoothing alpha for vibration level */
#define VIBRATION_SCALE_FACTOR    2.0f   /* Scale factor to convert to 0-10 range */
#define VIBRATION_MAX_LEVEL       10.0f  /* Maximum vibration level */

/* GPS EMA Filter */
#define GPS_EMA_ALPHA             0.9f   /* GPS position smoothing (90% old, 10% new) */

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CONFIG_H */

