/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kalman_velocity.h
  * @brief   Kalman filter for velocity estimation
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

#ifndef KALMAN_VELOCITY_H
#define KALMAN_VELOCITY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
  float vx;              /* North-South velocity (m/s) */
  float vy;              /* East-West velocity (m/s) */
  float vz;              /* Vertical velocity (m/s) - not used */
  float speed;           /* Total speed (m/s) */
  float P;               /* Uncertainty (covariance) */
  float Q;               /* Process noise */
  float R_gps;           /* GPS measurement noise */
  float R_imu;           /* IMU measurement noise */
  bool initialized;      /* Initialization flag */
} KalmanVelocity_t;

/* Function prototypes -------------------------------------------------------*/
void KalmanVelocity_Init(KalmanVelocity_t *filter, float Q, float R_gps, float R_imu);
void KalmanVelocity_UpdateIMU(KalmanVelocity_t *filter,
                              float ax, float ay, float az,  /* Acceleration */
                              float dt);                      /* Time step */
void KalmanVelocity_UpdateGPS(KalmanVelocity_t *filter,
                              float vx_gps, float vy_gps,    /* GPS velocity */
                              uint8_t fix_quality);           /* GPS fix quality */
void KalmanVelocity_GetVelocity(KalmanVelocity_t *filter,
                                float *vx, float *vy, float *vz, float *speed);

/**
  * @brief  Reset velocity estimate to zero (use when confirmed stationary)
  * @param  filter: Filter structure pointer
  * @retval None
  */
void KalmanVelocity_Reset(KalmanVelocity_t *filter);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_VELOCITY_H */

