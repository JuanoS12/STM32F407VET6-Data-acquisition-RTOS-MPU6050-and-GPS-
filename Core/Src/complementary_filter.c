/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    complementary_filter.c
  * @brief   Complementary filter implementation
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
#include "complementary_filter.h"
#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define PI 3.14159265358979323846f

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize complementary filter
  * @param  filter: Filter structure pointer
  * @param  alpha: Filter coefficient (0.98 typical)
  * @retval None
  */
void ComplementaryFilter_Init(ComplementaryFilter_t *filter, float alpha)
{
  if (filter == NULL) return;
  
  memset(filter, 0, sizeof(ComplementaryFilter_t));
  filter->alpha = alpha;
  filter->initialized = true;
}

/**
  * @brief  Update complementary filter with new sensor data
  * @param  filter: Filter structure pointer
  * @param  ax, ay, az: Accelerometer readings (m/s²)
  * @param  gx, gy, gz: Gyroscope readings (deg/s)
  * @param  dt: Time step in seconds
  * @retval None
  */
void ComplementaryFilter_Update(ComplementaryFilter_t *filter,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float dt)
{
  float accel_mag;
  float pitch_accel = 0.0f, roll_accel = 0.0f;
  
  if (filter == NULL || !filter->initialized || dt <= 0.0f) {
    return;
  }
  
  /* Calculate accelerometer magnitude */
  accel_mag = sqrtf(ax * ax + ay * ay + az * az);
  
  /* Calculate angles from accelerometer (only if reading is valid) */
  if (accel_mag > 0.1f && accel_mag < 2.0f * 9.8f) {
    /* Pitch: rotation around Y-axis */
    pitch_accel = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
    
    /* Roll: rotation around X-axis */
    roll_accel = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
  }
  
  /* Complementary filter: combine previous angle + gyro integration with accel angle */
  /* angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel */
  
  filter->pitch = filter->alpha * (filter->pitch + gx * dt) + 
                  (1.0f - filter->alpha) * pitch_accel;
                  
  filter->roll = filter->alpha * (filter->roll + gy * dt) + 
                 (1.0f - filter->alpha) * roll_accel;
  
  /* Yaw: only from gyroscope (no absolute reference from accelerometer) */
  filter->yaw += gz * dt;
  
  /* Yaw wrapping: 0 to 360 degrees */
  filter->yaw = fmodf(filter->yaw, 360.0f);
  if (filter->yaw < 0.0f) {
    filter->yaw += 360.0f;
  }
}
float ComplementaryFilter_GetPitch(ComplementaryFilter_t *filter)
{
  if (filter == NULL) return 0.0f;
  return filter->pitch;
}

/**
  * @brief  Get filtered roll angle
  * @param  filter: Filter structure pointer
  * @retval Roll angle in degrees
  */
float ComplementaryFilter_GetRoll(ComplementaryFilter_t *filter)
{
  if (filter == NULL) return 0.0f;
  return filter->roll;
}

/**
  * @brief  Get filtered yaw angle
  * @param  filter: Filter structure pointer
  * @retval Yaw angle in degrees
  */
float ComplementaryFilter_GetYaw(ComplementaryFilter_t *filter)
{
  if (filter == NULL) return 0.0f;
  return filter->yaw;
}


/**
  * @brief  Correct yaw angle using GPS course
  * @param  filter: Filter structure pointer
  * @param  gps_course_deg: GPS course in degrees
  * @retval None
  */
void ComplementaryFilter_CorrectYaw(ComplementaryFilter_t *filter, float gps_course_deg)
{
  if (filter == NULL) return;
  
  /* Simple fusion: 98% current yaw, 2% GPS course */
  /* Note: This needs to handle the 0/360 boundary correctly */
  
  float diff = gps_course_deg - filter->yaw;
  
  /* Normalize difference to -180 to +180 */
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  
  /* Apply correction factor (0.02) */
  filter->yaw += diff * 0.02f;
  
  /* Re-wrap result */
  filter->yaw = fmodf(filter->yaw, 360.0f);
  if (filter->yaw < 0.0f) {
    filter->yaw += 360.0f;
  }
}
