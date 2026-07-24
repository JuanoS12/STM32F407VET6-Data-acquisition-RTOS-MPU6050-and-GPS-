/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 IMU driver header
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

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* MPU6050 I2C Address */
#define MPU6050_I2C_ADDR_7B  0x68
#define MPU6050_ADDR         (MPU6050_I2C_ADDR_7B << 1)

/* MPU6050 Register Addresses */
#define MPU6050_REG_PWR_MGMT_1       0x6B
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43

/* Exported types ------------------------------------------------------------*/
/* Raw IMU data structure (without angles - angles are in telemetry_globals.h) */
typedef struct {
  float ax, ay, az;      /* Accelerometer (m/s²) */
  float gx, gy, gz;      /* Gyroscope (deg/s) */
  float gx_offset, gy_offset, gz_offset; /* Gyro Calibration offsets */
  float ax_offset, ay_offset, az_offset; /* Accel Calibration offsets */
} MPU6050_RawData_t;

/* Function prototypes -------------------------------------------------------*/
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *DataStruct);
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *DataStruct, uint16_t samples);
void MPU6050_CalibrateAccel(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *DataStruct, uint16_t samples);
uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */

