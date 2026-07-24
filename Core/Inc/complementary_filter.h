/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    complementary_filter.h
  * @brief   Complementary filter for IMU angle estimation
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

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
  float alpha;           /* Filter coefficient (0.98 typical) */
  float pitch_gyro;       /* Gyroscope-integrated pitch */
  float roll_gyro;        /* Gyroscope-integrated roll */
  float yaw_gyro;         /* Gyroscope-integrated yaw */
  float pitch;            /* Filtered pitch angle */
  float roll;             /* Filtered roll angle */
  float yaw;              /* Filtered yaw angle */
  bool initialized;       /* Initialization flag */
} ComplementaryFilter_t;

/* Function prototypes -------------------------------------------------------*/
void ComplementaryFilter_Init(ComplementaryFilter_t *filter, float alpha);
void ComplementaryFilter_Update(ComplementaryFilter_t *filter,
                                 float ax, float ay, float az,  /* Accel */
                                 float gx, float gy, float gz,  /* Gyro */
                                 float dt);                      /* Time step */
float ComplementaryFilter_GetPitch(ComplementaryFilter_t *filter);
float ComplementaryFilter_GetRoll(ComplementaryFilter_t *filter);
float ComplementaryFilter_GetYaw(ComplementaryFilter_t *filter);
void ComplementaryFilter_CorrectYaw(ComplementaryFilter_t *filter, float gps_course_deg);

#ifdef __cplusplus
}
#endif

#endif /* COMPLEMENTARY_FILTER_H */

