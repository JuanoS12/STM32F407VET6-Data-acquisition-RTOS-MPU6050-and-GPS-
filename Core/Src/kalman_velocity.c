/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kalman_velocity.c
  * @brief   Kalman filter implementation for velocity estimation
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
#include "kalman_velocity.h"
#include <math.h>
#include <string.h>

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize Kalman filter
  * @param  filter: Filter structure pointer
  * @param  Q: Process noise
  * @param  R_gps: GPS measurement noise
  * @param  R_imu: IMU measurement noise
  * @retval None
  */
void KalmanVelocity_Init(KalmanVelocity_t *filter, float Q, float R_gps, float R_imu)
{
  if (filter == NULL) return;
  
  memset(filter, 0, sizeof(KalmanVelocity_t));
  filter->Q = Q;
  filter->R_gps = R_gps;
  filter->R_imu = R_imu;
  filter->P = 1.0f;  /* Initial uncertainty */
  filter->initialized = true;
}

/**
  * @brief  Update Kalman filter with IMU acceleration (prediction step)
  * @param  filter: Filter structure pointer
  * @param  ax, ay, az: Acceleration readings (m/s²)
  * @param  dt: Time step in seconds
  * @retval None
  */
void KalmanVelocity_UpdateIMU(KalmanVelocity_t *filter,
                               float ax, float ay, float az,
                               float dt)
{
  if (filter == NULL || !filter->initialized || dt <= 0.0f) {
    return;
  }
  
  /* Prediction step: integrate acceleration */
  filter->vx += ax * dt;
  filter->vy += ay * dt;
  
  /* Increase uncertainty */
  filter->P += filter->Q;
  
  /* Calculate total speed */
  filter->speed = sqrtf(filter->vx * filter->vx + filter->vy * filter->vy);
}

/**
  * @brief  Update Kalman filter with GPS velocity (correction step)
  * @param  filter: Filter structure pointer
  * @param  vx_gps, vy_gps: GPS velocity components (m/s)
  * @param  fix_quality: GPS fix quality (0=invalid, 1=GPS, 2=DGPS)
  * @retval None
  */
void KalmanVelocity_UpdateGPS(KalmanVelocity_t *filter,
                               float vx_gps, float vy_gps,
                               uint8_t fix_quality)
{
  float K;  /* Kalman gain */
  float R;
  
  if (filter == NULL || !filter->initialized) {
    return;
  }
  
  /* Only update if GPS fix is valid */
  if (fix_quality == 0) {
    return;
  }
  
  /* Use GPS measurement noise */
  R = filter->R_gps;
  
  /* Calculate Kalman gain */
  K = filter->P / (filter->P + R);
  
  /* Update state estimate */
  filter->vx = filter->vx + K * (vx_gps - filter->vx);
  filter->vy = filter->vy + K * (vy_gps - filter->vy);
  
  /* Update uncertainty */
  filter->P = (1.0f - K) * filter->P;
  
  /* Calculate total speed */
  filter->speed = sqrtf(filter->vx * filter->vx + filter->vy * filter->vy);
}

/**
  * @brief  Get current velocity estimate
  * @param  filter: Filter structure pointer
  * @param  vx: Output North-South velocity (m/s)
  * @param  vy: Output East-West velocity (m/s)
  * @param  vz: Output vertical velocity (m/s) - can be NULL
  * @param  speed: Output total speed (m/s) - can be NULL
  * @retval None
  */
void KalmanVelocity_GetVelocity(KalmanVelocity_t *filter,
                                float *vx, float *vy, float *vz, float *speed)
{
  if (filter == NULL) {
    if (vx) *vx = 0.0f;
    if (vy) *vy = 0.0f;
    if (vz) *vz = 0.0f;
    if (speed) *speed = 0.0f;
    return;
  }
  
  if (vx) *vx = filter->vx;
  if (vy) *vy = filter->vy;
  if (vz) *vz = filter->vz;
  if (speed) *speed = filter->speed;
}

/**
  * @brief  Reset velocity estimate to zero (use when confirmed stationary)
  * @param  filter: Filter structure pointer
  * @retval None
  */
void KalmanVelocity_Reset(KalmanVelocity_t *filter)
{
  if (filter == NULL || !filter->initialized) return;
  
  filter->vx = 0.0f;
  filter->vy = 0.0f;
  filter->speed = 0.0f;
  /* Keep P (uncertainty) as is or reset? Usually better to keep it to avoid jump on next update */
}

