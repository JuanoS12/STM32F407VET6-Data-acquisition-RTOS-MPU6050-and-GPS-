/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 IMU driver implementation
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
#include "mpu6050.h"
#include "system_config.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define MPU6050_REG_WHO_AM_I       0x75

/* Private variables ---------------------------------------------------------*/
static uint8_t mpu6050_active_addr = MPU6050_ADDR;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize MPU6050 sensor (Adafruit library proven sequence)
  * @param  hi2c: I2C handle pointer
  * @retval HAL status
  */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t data;
  HAL_StatusTypeDef status;
  
  if (hi2c == NULL) {
    return HAL_ERROR;
  }
  
  /* Step 1: Device Reset - Write 0x80 to PWR_MGMT_1 (bit 7 = DEVICE_RESET) */
  data = 0x80;
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, MPU6050_REG_PWR_MGMT_1,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  
  /* Wait for reset bit to clear (Adafruit waits in loop, we'll wait fixed time) */
  HAL_Delay(100);
  
  /* Verify reset completed by reading PWR_MGMT_1 (bit 7 should be 0) */
  status = HAL_I2C_Mem_Read(hi2c, mpu6050_active_addr, MPU6050_REG_PWR_MGMT_1,
                            I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  if (data & 0x80) {
    /* Reset didn't complete */
    return HAL_ERROR;
  }
  
  /* Step 2: Signal Path Reset - Write 0x07 to clear analog and digital paths */
  data = 0x07;
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, MPU6050_REG_SIGNAL_PATH_RESET,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  HAL_Delay(100);
  
  /* Step 3: Wake up & Set Clock Source to PLL with X-axis gyro reference */
  /* This is more stable than internal 8MHz oscillator */
  data = 0x01;  /* CLKSEL = 1 (PLL with X-axis gyro), SLEEP = 0 */
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, MPU6050_REG_PWR_MGMT_1,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  HAL_Delay(100);
  
  /* Step 4: Configure DLPF - Use 260Hz bandwidth (Adafruit default) */
  /* DLPF_CFG = 0 gives 260Hz accel bandwidth, 256Hz gyro bandwidth */
  data = 0x00;  /* Not 0x06 which gives only 5Hz! */
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, MPU6050_REG_CONFIG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  
  /* Step 5: Set Sample Rate Divisor to 0 (1kHz sample rate) */
  data = 0x00;
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, 0x19,  /* SMPLRT_DIV register */
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  
  /* Step 6: Accelerometer Configuration - ±2G range (better resolution) */
  /* AFS_SEL = 0 for ±2g range (16384 LSB/g) */
  data = 0x00;  /* Not 0x10 which is ±8g */
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, MPU6050_REG_ACCEL_CONFIG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  
  /* Step 7: Gyroscope Configuration - ±500°/s range */
  /* FS_SEL = 1 for ±500°/s (65.5 LSB/°/s) */
  data = 0x08;  /* Keep this same */
  status = HAL_I2C_Mem_Write(hi2c, mpu6050_active_addr, MPU6050_REG_GYRO_CONFIG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return status;
  
  return HAL_OK;
}

/**
  * @brief  Read all MPU6050 data (accelerometer, temperature, gyroscope)
  * @param  hi2c: I2C handle pointer
  * @param  DataStruct: Pointer to data structure
  * @retval HAL status
  */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *DataStruct)
{
  uint8_t data[14];  /* Accel (6) + Temp (2) + Gyro (6) */
  int16_t raw_ax, raw_ay, raw_az;
  int16_t raw_gx, raw_gy, raw_gz;
  
  if (hi2c == NULL || DataStruct == NULL) {
    return HAL_ERROR;
  }
  
  /* Read all registers in one transaction (0x3B to 0x48) */
  HAL_StatusTypeDef res = HAL_I2C_Mem_Read(hi2c, mpu6050_active_addr, MPU6050_REG_ACCEL_XOUT_H,
                                           I2C_MEMADD_SIZE_8BIT, data, 14, I2C_TIMEOUT_MS);
  
  if (res != HAL_OK) {
    /* Only zero sensor readings, NOT calibration offsets */
    DataStruct->ax = 0; DataStruct->ay = 0; DataStruct->az = 0;
    DataStruct->gx = 0; DataStruct->gy = 0; DataStruct->gz = 0;
    return HAL_ERROR;
  }
  
  /* Parse 16-bit values (big-endian) */
  raw_ax = (data[0] << 8) | data[1];
  raw_ay = (data[2] << 8) | data[3];
  raw_az = (data[4] << 8) | data[5];
  raw_gx = (data[8] << 8) | data[9];
  raw_gy = (data[10] << 8) | data[11];
  raw_gz = (data[12] << 8) | data[13];
  
  /* Convert to physical units */
  /* Accelerometer: ±2G range = 16384 LSB/g (Adafruit default), 1g = 9.8 m/s² */
  DataStruct->ax = ((float)raw_ax / 16384.0f * 9.8f) - DataStruct->ax_offset;
  DataStruct->ay = ((float)raw_ay / 16384.0f * 9.8f) - DataStruct->ay_offset;
  DataStruct->az = ((float)raw_az / 16384.0f * 9.8f) - DataStruct->az_offset;
  
  /* Gyroscope: ±500°/s range = 65.5 LSB/(°/s) */
  DataStruct->gx = ((float)raw_gx / 65.5f) - DataStruct->gx_offset;
  DataStruct->gy = ((float)raw_gy / 65.5f) - DataStruct->gy_offset;
  DataStruct->gz = ((float)raw_gz / 65.5f) - DataStruct->gz_offset;
  
  return HAL_OK;
}

/**
  * @brief  Calibrate Gyroscope (calculate offsets)
  * @param  hi2c: I2C handle pointer
  * @param  DataStruct: Pointer to data structure
  * @param  samples: Number of samples to average
  * @retval None
  */
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *DataStruct, uint16_t samples)
{
  uint8_t data[6];
  int16_t raw_gx, raw_gy, raw_gz;
  float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
  
  if (hi2c == NULL || DataStruct == NULL || samples == 0) return;
  
  /* Reset offsets */
  DataStruct->gx_offset = 0.0f;
  DataStruct->gy_offset = 0.0f;
  DataStruct->gz_offset = 0.0f;
  
  for (uint16_t i = 0; i < samples; i++) {
    HAL_I2C_Mem_Read(hi2c, mpu6050_active_addr, MPU6050_REG_GYRO_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, data, 6, I2C_TIMEOUT_MS);
    
    raw_gx = (data[0] << 8) | data[1];
    raw_gy = (data[2] << 8) | data[3];
    raw_gz = (data[4] << 8) | data[5];
    
    sum_gx += (float)raw_gx;
    sum_gy += (float)raw_gy;
    sum_gz += (float)raw_gz;
    
    HAL_Delay(3); /* Small delay between samples */
  }
  
  /* Calculate average offset in deg/s */
  DataStruct->gx_offset = (sum_gx / samples) / 65.5f;
  DataStruct->gy_offset = (sum_gy / samples) / 65.5f;
  DataStruct->gz_offset = (sum_gz / samples) / 65.5f;
}

/**
  * @brief  Calibrate Accelerometer (calculate offsets)
  * @param  hi2c: I2C handle pointer
  * @param  DataStruct: Pointer to data structure
  * @param  samples: Number of samples to average
  * @retval None
  */
void MPU6050_CalibrateAccel(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *DataStruct, uint16_t samples)
{
  uint8_t data[6];
  int16_t raw_ax, raw_ay, raw_az;
  float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
  
  if (hi2c == NULL || DataStruct == NULL || samples == 0) return;
  
  /* Reset offsets */
  DataStruct->ax_offset = 0.0f;
  DataStruct->ay_offset = 0.0f;
  DataStruct->az_offset = 0.0f;
  
  for (uint16_t i = 0; i < samples; i++) {
    HAL_I2C_Mem_Read(hi2c, mpu6050_active_addr, MPU6050_REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, data, 6, I2C_TIMEOUT_MS);
    
    raw_ax = (data[0] << 8) | data[1];
    raw_ay = (data[2] << 8) | data[3];
    raw_az = (data[4] << 8) | data[5];
    
    /* Convert to physical units (m/s²) immediately for averaging */
    sum_ax += (float)raw_ax / 16384.0f * 9.8f;
    sum_ay += (float)raw_ay / 16384.0f * 9.8f;
    sum_az += (float)raw_az / 16384.0f * 9.8f;
    
    HAL_Delay(3);
  }
  
  /* Calculate average offsets */
  /* Assumption: Device is FLAT. X and Y should be 0. Z should be 9.8. */
  DataStruct->ax_offset = sum_ax / samples;
  DataStruct->ay_offset = sum_ay / samples;
  DataStruct->az_offset = (sum_az / samples) - 9.8f; /* Remove gravity from offset calculation */
}

/**
  * @brief  Read MPU6050 WHO_AM_I register
  * @param  hi2c: I2C handle pointer
  * @retval WHO_AM_I value (0x68 if correct)
  */
uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c)
{
  uint8_t whoami = 0;
  if (hi2c != NULL) {
    HAL_I2C_Mem_Read(hi2c, mpu6050_active_addr, MPU6050_REG_WHO_AM_I,
                    I2C_MEMADD_SIZE_8BIT, &whoami, 1, I2C_TIMEOUT_MS);
  }
  return whoami;
}

