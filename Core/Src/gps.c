/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gps.c
  * @brief   GPS NMEA parser implementation
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
#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>  /* For strchr */

/* Private defines -----------------------------------------------------------*/
#define EARTH_RADIUS_M            6371000.0  /* Earth radius in meters */
#define PI                         3.14159265358979323846

/* Private function prototypes -----------------------------------------------*/
static double nmea_to_decimal(double nmea_coord, char hemisphere);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Convert NMEA coordinate format to decimal degrees
  * @param  nmea_coord: NMEA coordinate (DDMM.MMMM format)
  * @param  hemisphere: 'N', 'S', 'E', or 'W'
  * @retval Decimal degrees
  */
static double nmea_to_decimal(double nmea_coord, char hemisphere)
{
  double degrees = floor(nmea_coord / 100.0);
  double minutes = nmea_coord - (degrees * 100.0);
  double decimal = degrees + (minutes / 60.0);
  
  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }
  
  return decimal;
}

/**
  * @brief  Convert NMEA coordinate format to decimal degrees (public)
  * @param  nmea_coord: NMEA coordinate (DDMM.MMMM format)
  * @param  hemisphere: 'N', 'S', 'E', or 'W'
  * @retval Decimal degrees
  */
double GPS_NMEA_ToDecimal(double nmea_coord, char hemisphere)
{
  return nmea_to_decimal(nmea_coord, hemisphere);
}

/**
  * @brief  Parse NMEA GPGGA sentence
  * @param  buf: Buffer containing NMEA sentence
  * @param  len: Length of buffer
  * @param  out: Output GPS data structure
  * @retval HAL status
  */
HAL_StatusTypeDef GPS_ParseBuffer(uint8_t *buf, size_t len, GPS_ParsedData_t *out)
{
  char *token;
  char *next_token;
  int field = 0;
  double lat_nmea = 0.0, lon_nmea = 0.0;
  char lat_hemi = 'N', lon_hemi = 'E';
  char str_buf[512];  /* Increased size */
  uint8_t is_rmc = 0;
  
  if (buf == NULL || out == NULL || len == 0) {
    return HAL_ERROR;
  }
  
  /* Ensure null termination */
  if (len >= sizeof(str_buf)) {
    len = sizeof(str_buf) - 1;
  }
  memcpy(str_buf, buf, len);
  str_buf[len] = '\0';
  
  /* Find start of NMEA sentence */
  char *start = strchr(str_buf, '$');
  if (start == NULL) {
    return HAL_ERROR;
  }
  
  /* Check sentence type */
  if (strncmp(start + 3, "GGA", 3) == 0) {
    is_rmc = 0;
  } else if (strncmp(start + 3, "RMC", 3) == 0) {
    is_rmc = 1;
  } else {
    return HAL_ERROR; /* Not a supported sentence */
  }
  
  /* Thread-safe manual parsing - use start pointer */
  token = start;
  while (token != NULL && field < 15) {
    /* Find next comma */
    next_token = strchr(token, ',');
    size_t field_len;
    
    if (next_token != NULL) {
      field_len = next_token - token;
      next_token++;  /* Move to next field */
    } else {
      field_len = strlen(token);  /* Last field */
    }
    
    /* Process field based on position and sentence type */
    if (!is_rmc) {
      /* --- GGA Parsing --- */
      switch (field) {
        case 2:  /* Latitude */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            lat_nmea = atof(temp);
          }
          break;
        case 3:  /* N/S */
          if (field_len > 0) lat_hemi = token[0];
          break;
        case 4:  /* Longitude */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            lon_nmea = atof(temp);
          }
          break;
        case 5:  /* E/W */
          if (field_len > 0) lon_hemi = token[0];
          break;
        case 6:  /* Fix Quality */
          if (field_len > 0) {
            char temp[8];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            out->fix = (uint8_t)atoi(temp);
            
            /* Map fix type to confidence roughly */
            if (out->fix == 0) out->confidence = 0;
            else if (out->fix == 1) out->confidence = 50;
            else if (out->fix == 2) out->confidence = 90;
            else out->confidence = 100;
          }
          break;
        
        case 7:  /* Number of Satellites */
          if (field_len > 0) {
            char temp[8];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            out->satellites = (uint8_t)atoi(temp);
          }
          break;
          
        case 8:  /* HDOP (Horizontal Dilution of Precision) */
          if (field_len > 0) {
            char temp[16];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            out->hdop = (float)atof(temp);
            
            /* Refine confidence based on HDOP */
            if (out->hdop > 0.1f && out->hdop < 1.0f) out->confidence = 100;
            else if (out->hdop > 10.0f) out->confidence = 10;
          }
          break;
          
        case 9:  /* Altitude */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            out->altitude = (float)atof(temp);
          }
          break;
      }
    } else {
      /* --- RMC Parsing --- */
      /* $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A */
      switch (field) {
        case 2: /* Status (A=Active, V=Void) */
           if (field_len > 0 && token[0] == 'A') {
             out->fix = 1; /* Valid fix */
           } else {
             out->fix = 0;
           }
           break;
        case 3: /* Latitude */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            lat_nmea = atof(temp);
          }
          break;
        case 4: /* N/S */
          if (field_len > 0) lat_hemi = token[0];
          break;
        case 5: /* Longitude */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            lon_nmea = atof(temp);
          }
          break;
        case 6: /* E/W */
          if (field_len > 0) lon_hemi = token[0];
          break;
        case 7: /* Speed in knots */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            /* Convert knots to m/s (1 knot = 0.514444 m/s) */
            out->speed_ms = (float)atof(temp) * 0.514444f;
          }
          break;
        case 8: /* Course in degrees */
          if (field_len > 0) {
            char temp[32];
            strncpy(temp, token, (field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1);
            temp[(field_len < sizeof(temp)-1) ? field_len : sizeof(temp)-1] = '\0';
            out->course_deg = (float)atof(temp);
          }
          break;
      }
    }
    
    if (next_token == NULL) break;  /* End of string */
    token = next_token;  /* Move to next field */
    field++;
  }
  
  /* Convert NMEA format to decimal degrees */
  if (lat_nmea > 0.0 && lon_nmea > 0.0) {
    out->latitude = nmea_to_decimal(lat_nmea, lat_hemi);
    out->longitude = nmea_to_decimal(lon_nmea, lon_hemi);
    return HAL_OK;
  }
  
  return HAL_ERROR;
}

/**
  * @brief  Calculate velocity from position changes
  * @param  lat1: Previous latitude
  * @param  lon1: Previous longitude
  * @param  lat2: Current latitude
  * @param  lon2: Current longitude
  * @param  dt: Time delta in seconds
  * @param  vx: Output North-South velocity (m/s)
  * @param  vy: Output East-West velocity (m/s)
  * @param  speed: Output total speed (m/s)
  * @retval None
  */
void GPS_CalculateVelocity(double lat1, double lon1, double lat2, double lon2,
                          float dt, float *vx, float *vy, float *speed)
{
  double dlat, dlon;
  double dlat_m, dlon_m;
  
  if (dt <= 0.0f || vx == NULL || vy == NULL || speed == NULL) {
    if (vx) *vx = 0.0f;
    if (vy) *vy = 0.0f;
    if (speed) *speed = 0.0f;
    return;
  }
  
  /* Check for invalid GPS coordinates */
  if ((lat1 == 0.0 && lon1 == 0.0) || (lat2 == 0.0 && lon2 == 0.0)) {
    if (vx) *vx = 0.0f;
    if (vy) *vy = 0.0f;
    if (speed) *speed = 0.0f;
    return;
  }
  
  /* Calculate distance changes */
  dlat = lat2 - lat1;
  dlon = lon2 - lon1;
  
  /* Convert to meters */
  dlat_m = dlat * (PI / 180.0) * EARTH_RADIUS_M;
  dlon_m = dlon * (PI / 180.0) * EARTH_RADIUS_M * cos(lat1 * PI / 180.0);
  
  /* Calculate velocity components */
  *vx = (float)(dlat_m / dt);
  *vy = (float)(dlon_m / dt);
  *speed = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
}

