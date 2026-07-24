/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "telemetry_globals.h"
#include "mpu6050.h"
#include "gps.h"
#include "complementary_filter.h"
#include "kalman_velocity.h"
#include "task_watchdog.h"
#include "health.h"
#include "system_config.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stream_buffer.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usbd_cdc_if.h"

#define PI 3.14159265358979323846

/* Helper function to send log messages to both UART3 and USB CDC */
void Log_Msg(char *msg, uint16_t len) {
    if (len == 0) return;
    
    /* Send to UART3 (Original Debug Port) */
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
    
    /* Send to USB CDC (Virtual COM Port) */
    /* Note: CDC_Transmit_FS is non-blocking but might return USBD_BUSY. 
       We make a best-effort attempt here. */
    // CDC_Transmit_FS((uint8_t*)msg, len);
    
    /* Small delay to prevent buffer overrun if sending back-to-back bursts */
    osDelay(1); 
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;  /* DMA handle for GPS UART */

/* Task handles */
osThreadId imuTaskHandle;
osThreadId gpsTaskHandle;
osThreadId telemetryTaskHandle;
osThreadId uartDebugTaskHandle;
osThreadId ledTaskHandle;
osThreadId healthTaskHandle;

/* Mutex handles */
osMutexId kalmanMutexHandle;

/* Filter instances */
static ComplementaryFilter_t imu_filter;
static KalmanVelocity_t velocity_filter;

/* GPS buffer - defined in usart.c */
extern uint8_t gps_rx_buf[];

/* Previous GPS position for velocity calculation */
static double last_latitude = 0.0;
static double last_longitude = 0.0;
static TickType_t last_gps_time = 0;

/* Telemetry statistics */
static uint32_t spi_tx_success_count = 0;
static uint32_t spi_tx_error_count = 0;
static uint8_t last_spi_packet[SPI_PACKET_TOTAL_LEN];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartIMU_Task(void const * argument);
void StartGPS_Task(void const * argument);
void StartTelemetry_Task(void const * argument);
void StartUART_Debug_Task(void const * argument);
void StartLED_Task(void const * argument);
void StartHealth_Task(void const * argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* I2C Bus Scan Function - Check for all connected devices */
static void I2C_BusScan(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
  char msg[256];
  int len;
  
  len = sprintf(msg, "\r\n[I2C_SCAN] ===== I2C BUS SCAN (I2C2 @ 100kHz) =====\r\n");
  Log_Msg(msg, len);
  
  uint8_t device_found = 0;
  uint8_t test_byte = 0;
  
  /* Standard I2C addresses: 0x08 to 0x77 (skip reserved) */
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    /* Skip reserved address ranges */
    if ((addr >= 0x00 && addr < 0x08) || (addr >= 0x78 && addr <= 0x7F)) continue;
    
    /* Try to read 1 byte from this address */
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, (addr << 1), &test_byte, 1, 10);
    
    if (status == HAL_OK) {
      len = sprintf(msg, "[I2C_SCAN]   ✓ Device found at 0x%02X (7-bit) / 0x%02X (8-bit read)\r\n", 
                    addr, addr << 1);
      Log_Msg(msg, len);
      device_found = 1;
    }
  }
  
  if (!device_found) {
    len = sprintf(msg, "[I2C_SCAN]   ✗ NO DEVICES FOUND on I2C bus!\r\n"
                       "[I2C_SCAN]   Check: VCC/GND connections, SDA/SCL pull-ups, slave address config\r\n");
    Log_Msg(msg, len);
  }
  
  len = sprintf(msg, "[I2C_SCAN] ===== SCAN COMPLETE =====\r\n\r\n");
  Log_Msg(msg, len);
}

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  /* Initialize USB Device - Moved to main.c */
  /* extern void MX_USB_DEVICE_Init(void); */
  /* MX_USB_DEVICE_Init(); */
  
  /* Initialize data structures ONLY - no mutex creation yet */
  memset(&imu_data, 0, sizeof(IMU_Data_t));
  memset(&gps_data, 0, sizeof(GPS_Data_t));
  memset(&sys_status, 0, sizeof(System_Status_t));
  
  /* Initialize task watchdog */
  TaskWatchdog_Init();
  
  /* Initialize health data (safe - no mutexes) */
  Health_Init();
  
  /* Initialize filters (no mutexes yet) */
  ComplementaryFilter_Init(&imu_filter, COMPLEMENTARY_FILTER_ALPHA);
  KalmanVelocity_Init(&velocity_filter, KALMAN_Q, KALMAN_R_GPS, KALMAN_R_IMU);
  
  /* Start GPS DMA reception - moved to GPS task to ensure stream buffer is ready */
  /* HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_rx_buf, GPS_RX_BUF_LEN); */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Mutexes will be created after scheduler starts */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  /* Note: Default task removed - using custom tasks instead */
  /* IMU Task - High priority */
  osThreadDef(IMU_Task, StartIMU_Task, osPriorityHigh, 0, STACK_SIZE_SENSOR);
  imuTaskHandle = osThreadCreate(osThread(IMU_Task), NULL);
  
  /* GPS Task - Above Normal priority */
  osThreadDef(GPS_Task, StartGPS_Task, osPriorityAboveNormal, 0, STACK_SIZE_SENSOR);
  gpsTaskHandle = osThreadCreate(osThread(GPS_Task), NULL);
  
  /* Telemetry Task - Above Normal priority */
  osThreadDef(Telemetry_Task, StartTelemetry_Task, osPriorityAboveNormal, 0, STACK_SIZE_SENSOR);
  telemetryTaskHandle = osThreadCreate(osThread(Telemetry_Task), NULL);
  
  /* UART Debug Task - Below Normal priority */
  osThreadDef(UART_Debug_Task, StartUART_Debug_Task, osPriorityBelowNormal, 0, STACK_SIZE_PROCESSING);
  uartDebugTaskHandle = osThreadCreate(osThread(UART_Debug_Task), NULL);
  
  /* LED Task - Low priority */
  osThreadDef(LED_Task, StartLED_Task, osPriorityLow, 0, STACK_SIZE_LOW);
  ledTaskHandle = osThreadCreate(osThread(LED_Task), NULL);
  
  /* Health Task - Low priority */
  osThreadDef(Health_Task, StartHealth_Task, osPriorityLow, 0, STACK_SIZE_HEALTH);
  healthTaskHandle = osThreadCreate(osThread(Health_Task), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Now create mutexes and streams (scheduler about to start after this function returns) */
  osMutexDef(MUTEX_I2C);
  osMutexDef(MUTEX_TELEMETRY);
  osMutexDef(MUTEX_HEALTH);
  
  MUTEX_I2CHandle = osMutexCreate(osMutex(MUTEX_I2C));
  MUTEX_TELEMETRYHandle = osMutexCreate(osMutex(MUTEX_TELEMETRY));
  health_mutex = osMutexCreate(osMutex(MUTEX_HEALTH));
  
  /* CRITICAL: Create Kalman Mutex */
  osMutexDef(MUTEX_KALMAN);
  kalmanMutexHandle = osMutexCreate(osMutex(MUTEX_KALMAN));
  
  SB_GPS = xStreamBufferCreate(GPS_STREAM_BUF_SIZE, 1);
  
  /* Initialize health data (mutex now available) */
  memset(&health_data, 0, sizeof(HealthData_t));
  health_data.last_imu_update = xTaskGetTickCount();
  health_data.last_gps_update = xTaskGetTickCount();

}

/* Helper: Rotate IMU acceleration to World Frame */
void imu_to_world(float ax, float ay, float az,
                  float pitch, float roll, float yaw,
                  float *wx, float *wy, float *wz)
{
    float cp = cosf(pitch * PI/180.0f);
    float sp = sinf(pitch * PI/180.0f);
    float cr = cosf(roll * PI/180.0f);
    float sr = sinf(roll * PI/180.0f);
    float cy = cosf(yaw * PI/180.0f);
    float sy = sinf(yaw * PI/180.0f);

    /* Rotation Matrix R_body_to_world */
    *wx = cy*(cp*ax + sp*sr*ay + sp*cr*az) - sy*(cr*ay - sr*az);
    *wy = sy*(cp*ax + sp*sr*ay + sp*cr*az) + cy*(cr*ay - sr*az);
    *wz = -sp*ax + cp*sr*ay + cp*cr*az;
}

/* Helper: Haversine Distance in meters */
double haversine_distance(double lat1, double lon1, double lat2, double lon2)
{
    double R = 6371000.0; // Earth radius in meters
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

/* USER CODE BEGIN Header_StartIMU_Task */
/**
  * @brief  Function implementing the IMU_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartIMU_Task */
void StartIMU_Task(void const * argument)
{
  /* USER CODE BEGIN StartIMU_Task */
  TaskWatchdog_Register("IMU_Task");
  
  char diag[256];
  int len;
  
  /* Perform full I2C bus scan to see what devices are present */
  len = sprintf(diag, "[IMU_TASK] Starting I2C bus diagnostics...\r\n");
  Log_Msg(diag, len);
  osDelay(100);
  
  I2C_BusScan(&hi2c2, &huart3);
  
  HAL_StatusTypeDef imu_init_status = HAL_ERROR;
  uint8_t init_retry_count = 0;
  const uint8_t MAX_INIT_RETRIES = ERROR_RECOVERY_RETRIES;
  MPU6050_RawData_t local_imu = {0}; /* Initialize to zero to prevent undefined behavior */
  TickType_t last_filter_update = xTaskGetTickCount();
  
  /* Try initialization with retries (no mutex - direct I2C access) */
  len = sprintf(diag, "[IMU_TASK] Attempting MPU6050 initialization (up to %d retries)...\r\n", MAX_INIT_RETRIES);
  Log_Msg(diag, len);
  
  while (imu_init_status != HAL_OK && init_retry_count < MAX_INIT_RETRIES) {
    imu_init_status = MPU6050_Init(&hi2c2);
    
    if (imu_init_status != HAL_OK) {
      init_retry_count++;
      len = sprintf(diag, "[IMU_TASK] Init attempt %d failed, Status=%d\r\n", init_retry_count, imu_init_status);
      Log_Msg(diag, len);
      
      if (init_retry_count < MAX_INIT_RETRIES) {
        osDelay(500);
      }
    }
  }
  
  /* Check WHO_AM_I register to verify I2C communication */
  if (imu_init_status == HAL_OK) {
    uint8_t whoami = MPU6050_WhoAmI(&hi2c2);
    len = sprintf(diag, "[IMU_TASK] WHO_AM_I register = 0x%02X (expected 0x68)\r\n", whoami);
    Log_Msg(diag, len);
    
    if (whoami != 0x68) {
      imu_init_status = HAL_ERROR;  /* Wrong device */
    }
  }
  
  /* Report final status */
  if (imu_init_status != HAL_OK) {
    sys_status.imu_ok = 0;
    len = sprintf(diag, "[IMU_TASK] ✗ INITIALIZATION FAILED - All retries exhausted!\r\n");
    Log_Msg(diag, len);
    
    /* Dump raw I2C register reads for debugging */
    len = sprintf(diag, "[IMU_TASK] === RAW I2C REGISTER DUMP ===\r\n");
    Log_Msg(diag, len);
    
    uint8_t reg_val = 0;
    HAL_StatusTypeDef reg_status;
    for (uint8_t addr = 0x3B; addr <= 0x48; addr++) {
      reg_status = HAL_I2C_Master_Receive(&hi2c2, 0xD0, &reg_val, 1, 50);
      if (reg_status == HAL_OK) {
        len = sprintf(diag, "[RAW] Reg 0x%02X = 0x%02X\r\n", addr, reg_val);
      } else {
        len = sprintf(diag, "[RAW] Reg 0x%02X = ERROR (Status=%d)\r\n", addr, reg_status);
      }
      Log_Msg(diag, len);
      osDelay(10);
    }
    len = sprintf(diag, "[IMU_TASK]   Check: MPU6050 power, I2C wiring (PB10=SCL, PB11=SDA), AD0 pin\r\n");
    Log_Msg(diag, len);
  } else {
    sys_status.imu_ok = 1;
    len = sprintf(diag, "[IMU_TASK] ✓ MPU6050 Initialized successfully!\r\n");
    Log_Msg(diag, len);
    
    /* Perform Gyro Calibration */
    len = sprintf(diag, "[IMU_TASK] Calibrating Gyro... KEEP STILL for 1 second...\r\n");
    Log_Msg(diag, len);
    
    MPU6050_CalibrateGyro(&hi2c2, &local_imu, 300); /* 300 samples ~1 sec */
    
    len = sprintf(diag, "[IMU_TASK] Calibration Done! Offsets: X=%.2f Y=%.2f Z=%.2f\r\n", 
                  local_imu.gx_offset, local_imu.gy_offset, local_imu.gz_offset);
    Log_Msg(diag, len);
    
    /* Perform Accelerometer Calibration */
    len = sprintf(diag, "[IMU_TASK] Calibrating Accel... KEEP FLAT & STILL...\r\n");
    Log_Msg(diag, len);
    
    MPU6050_CalibrateAccel(&hi2c2, &local_imu, 300);
    
    len = sprintf(diag, "[IMU_TASK] Accel Cal Done! Offsets: X=%.2f Y=%.2f Z=%.2f\r\n", 
                  local_imu.ax_offset, local_imu.ay_offset, local_imu.az_offset);
    Log_Msg(diag, len);
  }
  
  /* Main task loop */
  for(;;) {
    TickType_t current_time = xTaskGetTickCount();
    float dt = (float)(current_time - last_filter_update) / configTICK_RATE_HZ;
    
    if (dt > 0.1f) dt = 0.1f; /* Limit maximum dt */
    
    /* Read IMU data - direct I2C without mutex (no blocking) */
    if (MPU6050_Read_All(&hi2c2, &local_imu) == HAL_OK) {
      /* Update complementary filter */
      ComplementaryFilter_Update(&imu_filter,
                                local_imu.ax, local_imu.ay, local_imu.az,
                                local_imu.gx, local_imu.gy, local_imu.gz,
                                dt);
      
      /* Note: Kalman prediction moved below to use World Frame accel */
      
      /* Get filtered angles */
      float pitch = ComplementaryFilter_GetPitch(&imu_filter);
      float roll = ComplementaryFilter_GetRoll(&imu_filter);
      float yaw = ComplementaryFilter_GetYaw(&imu_filter);
      
      /* Calculate G-Forces */
      float g_total = sqrtf(local_imu.ax*local_imu.ax + local_imu.ay*local_imu.ay + local_imu.az*local_imu.az) / 9.8f;
      
      /* Calculate Longitudinal and Lateral Acceleration (Body Frame) */
      /* Note: This is a simplification. Ideally, remove gravity component first. */
      /* For now, using raw accel projected onto yaw plane as requested */
      /* But wait, local_imu.ax IS in body frame. */
      /* The user requested:
         a_long = ax*cos(yaw) + ay*sin(yaw);
         a_lat  = -ax*sin(yaw) + ay*cos(yaw);
         BUT this formula assumes ax/ay are in WORLD frame? No.
         If ax is forward and ay is right in BODY frame, then:
         Longitudinal Accel = ax
         Lateral Accel = ay
         The user's formula seems to be trying to rotate WORLD accel back to body?
         OR they mean rotating BODY accel to WORLD horizontal plane?
         Let's look at the user request again:
         "Transformándola al marco del vehículo:
          a_long = ax*cos(yaw) + ay*sin(yaw);
          a_lat  = -ax*sin(yaw) + ay*cos(yaw);"
         
         If ax/ay are already in vehicle frame (from sensor), then a_long IS ax.
         UNLESS the sensor is not aligned with the vehicle.
         Assuming sensor X is forward, Y is right.
         
         However, the user's code snippet:
         imu_to_world(ax, ay, az, pitch, roll, yaw, &wx, &wy, &wz);
         KalmanVelocity_UpdateIMU(&kv, wx, wy, wz, dt);
         
         This part is for VELOCITY estimation (World Frame).
         
         For G-Force, they want "real" Gs.
         Let's calculate World Frame acceleration first for Kalman.
      */
      
      float wx, wy, wz;
      imu_to_world(local_imu.ax, local_imu.ay, local_imu.az, pitch, roll, yaw, &wx, &wy, &wz);
      
      /* Update Kalman filter with WORLD frame acceleration */
      /* Subtract gravity from vertical component? Kalman expects linear acceleration? */
      /* wz includes gravity (~9.8 m/s²). Subtract it for velocity integration. */
      /* Update Kalman filter with WORLD frame acceleration (removing gravity) */
      if (osMutexWait(kalmanMutexHandle, pdMS_TO_TICKS(10)) == osOK) {
          KalmanVelocity_UpdateIMU(&velocity_filter, wx, wy, wz - 9.8f, dt);
          osMutexRelease(kalmanMutexHandle);
      }
      
      /* Calculate Vibration (High-pass filter on Z-axis) */
      static float az_avg = 9.8f;
      az_avg = VIBRATION_FILTER_ALPHA * az_avg + (1.0f - VIBRATION_FILTER_ALPHA) * local_imu.az;
      float instant_vibration = fabsf(local_imu.az - az_avg);
      
      static float vib_level = 0.0f;
      vib_level = VIBRATION_SMOOTH_ALPHA * vib_level + (1.0f - VIBRATION_SMOOTH_ALPHA) * instant_vibration;
      /* Scale to 0-10 range */
      float vib_scaled = vib_level * VIBRATION_SCALE_FACTOR; 
      if(vib_scaled > VIBRATION_MAX_LEVEL) vib_scaled = VIBRATION_MAX_LEVEL;
      
      float yaw_rad = yaw * PI / 180.0f;
      float g_long = (wx * cosf(yaw_rad) + wy * sinf(yaw_rad)) / 9.8f;
      float g_lat  = (-wx * sinf(yaw_rad) + wy * cosf(yaw_rad)) / 9.8f;
      float g_vert = wz / 9.8f; /* World Z includes gravity */

      /* Update shared data with mutex protection to prevent race conditions */
      if (osMutexWait(MUTEX_TELEMETRYHandle, pdMS_TO_TICKS(10)) == osOK) {
        /* Basic Raw Data */
        imu_data.ax = local_imu.ax;
        imu_data.ay = local_imu.ay;
        imu_data.az = local_imu.az;
        imu_data.gx = local_imu.gx;
        imu_data.gy = local_imu.gy;
        imu_data.gz = local_imu.gz;
        
        /* Explicit Rates */
        imu_data.pitch_rate = local_imu.gx;
        imu_data.roll_rate = local_imu.gy;
        imu_data.yaw_rate = local_imu.gz;
        
        /* Orientation */
        imu_data.pitch = pitch;
        imu_data.roll = roll;
        imu_data.yaw = yaw;
        
        /* Global Acceleration (World Frame) */
        imu_data.ax_global = wx;
        imu_data.ay_global = wy;
        imu_data.az_global = wz;
        
        /* G-Forces */
        imu_data.g_total = g_total;
        imu_data.g_long = g_long;
        imu_data.g_lat = g_lat;
        imu_data.g_vert = g_vert;
        
        /* Advanced */
        imu_data.vibration_level = vib_scaled;
        
        osMutexRelease(MUTEX_TELEMETRYHandle);
      }
      
      /* Notify health system */
      Health_NotifyIMUUpdated();
      sys_status.imu_ok = 1;
    } else {
      sys_status.imu_ok = 0;
    }
    
    last_filter_update = current_time;
    TaskWatchdog_Feed("IMU_Task");
    osDelay(pdMS_TO_TICKS(1000 / IMU_TASK_FREQ_HZ));
  }
  /* USER CODE END StartIMU_Task */
}

/* USER CODE BEGIN Header_StartGPS_Task */
/**
  * @brief  Function implementing the GPS_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGPS_Task */
void StartGPS_Task(void const * argument)
{
  /* USER CODE BEGIN StartGPS_Task */
  TaskWatchdog_Register("GPS_Task");
  
  /* Diagnostic: Send startup message */
  char diag[256];
  int len = sprintf(diag, "\r\n[GPS_TASK] Starting GPS initialization...\r\n");
  Log_Msg(diag, len);
  
  osDelay(500);  /* Wait for system to settle */
  
  /* Start GPS DMA reception now that stream buffer is ready */
  len = sprintf(diag, "[GPS_TASK] Starting USART2 DMA reception...\r\n");
  Log_Msg(diag, len);
  
  HAL_StatusTypeDef dma_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_rx_buf, GPS_RX_BUF_LEN);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);  /* Disable half-transfer interrupt - critical for proper GPS reception */
  
  if (dma_status != HAL_OK) {
    /* DMA start failed - send diagnostic */
    len = sprintf(diag, "[GPS_TASK] *** DMA FAILED: Status=%d ***\r\n", dma_status);
    Log_Msg(diag, len);
  } else {
    len = sprintf(diag, "[GPS_TASK] DMA Started successfully!\r\n");
    Log_Msg(diag, len);
  }
  
  len = sprintf(diag, "[GPS_TASK] Waiting for GPS data...\r\n\r\n");
  Log_Msg(diag, len);
  
  osDelay(1000);
  
  uint8_t rx_buf[256];
  char line_buf[128];
  size_t line_len = 0;
  GPS_ParsedData_t parsed_gps;
  memset(&parsed_gps, 0, sizeof(GPS_ParsedData_t)); /* Initialize all fields to zero */
  TickType_t current_time;
  float vx, vy;
  
  /* EMA Filter state for GPS position */
  double lat_filtered = 0.0;
  double lon_filtered = 0.0;
  uint8_t gps_filter_initialized = 0;
  /* GPS_EMA_ALPHA now defined in system_config.h */
  
  for(;;) {
    /* Receive data from stream buffer */
    size_t len = xStreamBufferReceive(SB_GPS, rx_buf, sizeof(rx_buf), 
                                       pdMS_TO_TICKS(100));
    
    if(len > 0) {
      /* Process each character */
      for(size_t i = 0; i < len; i++) {
        char c = rx_buf[i];
        
        if(c == '\n' || c == '\r') {
          if(line_len > 0) {
            line_buf[line_len] = '\0';
            
            /* DEBUG: Print every received sentence (COMMENTED OUT FOR CLEANER OUTPUT) */
            // char debug_head[64];
            // snprintf(debug_head, sizeof(debug_head), "[GPS] RX: %s\r\n", line_buf);
            // Log_Msg(debug_head, strlen(debug_head));

            /* Parse complete NMEA sentence */
            if (GPS_ParseBuffer((uint8_t*)line_buf, line_len, &parsed_gps) == HAL_OK) {
              /* Use direct speed and course from GPS (RMC sentence) */
              if (parsed_gps.fix) {
                /* 1. Sanity Check: Haversine Distance */
                /* If jump is too large (> 20m) in 1 second (approx), ignore it */
                if (gps_filter_initialized) {
                   double dist = haversine_distance(last_latitude, last_longitude, 
                                                    parsed_gps.latitude, parsed_gps.longitude);
                   if (dist > 20.0) {
                       /* Jump too large - ignore this point */
                       // Log_Msg("[GPS] Jump detected! Ignoring.\r\n", 30);
                       goto skip_gps_update;
                   }
                }
                
                /* 2. EMA Smoothing */
                if (!gps_filter_initialized) {
                    lat_filtered = parsed_gps.latitude;
                    lon_filtered = parsed_gps.longitude;
                    gps_filter_initialized = 1;
                } else {
                    lat_filtered = GPS_EMA_ALPHA * lat_filtered + (1.0f - GPS_EMA_ALPHA) * parsed_gps.latitude;
                    lon_filtered = GPS_EMA_ALPHA * lon_filtered + (1.0f - GPS_EMA_ALPHA) * parsed_gps.longitude;
                }
                
                /* Update parsed struct with filtered values for downstream use */
                parsed_gps.latitude = lat_filtered;
                parsed_gps.longitude = lon_filtered;

                /* Convert course to radians */
                float course_rad = parsed_gps.course_deg * (PI / 180.0f);
                
                /* Calculate velocity components
                   vx = North = Speed * cos(Course)
                   vy = East  = Speed * sin(Course) 
                */
                vx = parsed_gps.speed_ms * cosf(course_rad);
                vy = parsed_gps.speed_ms * sinf(course_rad);
                
                /* Update Kalman filter (correction step) */
                if (osMutexWait(kalmanMutexHandle, pdMS_TO_TICKS(10)) == osOK) {
                    /* ZUPT: Zero Velocity Update when almost stationary */
                    if (parsed_gps.speed_ms < 0.5f) {
                        KalmanVelocity_Reset(&velocity_filter);
                    } else {
                        KalmanVelocity_UpdateGPS(&velocity_filter, vx, vy, parsed_gps.fix);
                    }
                    osMutexRelease(kalmanMutexHandle);
                }
                
                /* 3. Fuse GPS Course with IMU Yaw */
                /* Only if speed is sufficient (> 1 m/s) to ensure course is valid */
                if (parsed_gps.speed_ms > 1.0f) {
                    ComplementaryFilter_CorrectYaw(&imu_filter, parsed_gps.course_deg);
                }
              }
              
              skip_gps_update:
              
              /* Read Kalman velocity OUTSIDE telemetry mutex to avoid nested lock */
              float kv_vx = 0, kv_vy = 0, kv_speed = 0;
              if (osMutexWait(kalmanMutexHandle, pdMS_TO_TICKS(10)) == osOK) {
                  KalmanVelocity_GetVelocity(&velocity_filter, &kv_vx, &kv_vy, NULL, &kv_speed);
                  osMutexRelease(kalmanMutexHandle);
              }
              
              /* Update shared data (single mutex, no nesting) */
              if (osMutexWait(MUTEX_TELEMETRYHandle, pdMS_TO_TICKS(50)) == osOK) {
                gps_data.latitude = parsed_gps.latitude;
                gps_data.longitude = parsed_gps.longitude;
                gps_data.altitude = parsed_gps.altitude;
                gps_data.fix = parsed_gps.fix;
                
                /* New Fields */
                gps_data.hdop = parsed_gps.hdop;
                gps_data.vdop = parsed_gps.vdop; /* Note: VDOP not currently parsed, will be 0 */
                gps_data.course = parsed_gps.course_deg;
                gps_data.satellites = parsed_gps.satellites;
                gps_data.confidence = parsed_gps.confidence;
                
                /* Use pre-fetched Kalman velocity (no nested mutex) */
                gps_data.velocity_x = kv_vx;
                gps_data.velocity_y = kv_vy;
                gps_data.speed = kv_speed;
                
                osMutexRelease(MUTEX_TELEMETRYHandle);
              }
              
              /* Update previous position */
              last_latitude = parsed_gps.latitude;
              last_longitude = parsed_gps.longitude;
              current_time = xTaskGetTickCount();
              last_gps_time = current_time;
              
              /* Notify health system */
              Health_NotifyGPSUpdated();
              sys_status.gps_ok = (parsed_gps.fix > 0) ? 1 : 0;
            }
            line_len = 0;
          }
        } else if(c >= 32 && c < 127) {
          /* Valid character - add to line buffer */
          if(line_len < sizeof(line_buf) - 1) {
            line_buf[line_len++] = c;
          }
        }
      }
    }
    
    TaskWatchdog_Feed("GPS_Task");
    osDelay(10);
  }
  /* USER CODE END StartGPS_Task */
}

/* USER CODE BEGIN Header_StartTelemetry_Task */
/**
  * @brief  Function implementing the Telemetry_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTelemetry_Task */
void StartTelemetry_Task(void const * argument)
{
  /* USER CODE BEGIN StartTelemetry_Task */
  TaskWatchdog_Register("Telemetry_Task");
  
  /* Wait for SPI to be ready */
  osDelay(100);
  
  /* Increased buffer size for extended telemetry */
  /* Packet Payload: ~130 bytes. Total: ~140 bytes */
  uint8_t txData[256];
  TickType_t last_telemetry_send = 0;
  
  /* Local copies of all data */
  IMU_Data_t local_imu_data;
  GPS_Data_t local_gps_data;
  System_Status_t local_sys_status;
  
  char debug_msg[256];
  
  for(;;) {
    TickType_t now = xTaskGetTickCount();
    
    /* Send Telemetry at 10 Hz */
    if ((now - last_telemetry_send) >= pdMS_TO_TICKS(1000 / TELEMETRY_TASK_FREQ_HZ)) {
      
      /* Get shared data */
      if (osMutexWait(MUTEX_TELEMETRYHandle, pdMS_TO_TICKS(50)) == osOK) {
        local_imu_data = imu_data;
        local_gps_data = gps_data;
        local_sys_status = sys_status;
        osMutexRelease(MUTEX_TELEMETRYHandle);
      } else {
        /* Timeout - use zeros */
        memset(&local_imu_data, 0, sizeof(IMU_Data_t));
        memset(&local_gps_data, 0, sizeof(GPS_Data_t));
        memset(&local_sys_status, 0, sizeof(System_Status_t));
      }
      
      /* --- Vehicle State Logic --- */
      uint8_t state = 0; // Default: Stop
      
      if (local_gps_data.speed > STATE_MIN_SPEED_MPS) {
          if (local_imu_data.g_long > STATE_ACCEL_THRESHOLD) state = 1; // Accel
          else if (local_imu_data.g_long < STATE_BRAKE_THRESHOLD) state = 2; // Brake
          else if (fabsf(local_imu_data.g_lat) > STATE_TURN_THRESHOLD) state = 3; // Turn
          else state = 1; // Cruising/Moving
      }
      
      if (local_imu_data.g_vert > STATE_AIRBORNE_G_HIGH || local_imu_data.g_vert < STATE_AIRBORNE_G_LOW) {
           /* Simple airborne detection (high bounce or freefall) */
           state = 4; // Airborne
      }
      local_sys_status.state = state;
      
      /* Slip Angle Estimation (Simplified) */
      /* Slip ~ Lateral Velocity / Longitudinal Velocity */
      /* We only have global velocities. Need to rotate to body frame. */
      /* Or just send 0 for now as it requires complex tire models or dual gps */
      local_sys_status.slip_angle = 0.0f;
      
      /* Wheel Contact (Simulation) */
      local_sys_status.wheel_contact = 0x0F; // All wheels grounded
      if (state == 4) local_sys_status.wheel_contact = 0x00; // Airborne
      
      /* --- SPI Packing (NEW LAYOUT) --- */
      static uint16_t seq_count = 0;
      uint32_t t_ms = xTaskGetTickCount();
      
      uint16_t idx = 0;
      txData[idx++] = SPI_PACKET_START_BYTE;
      txData[idx++] = SPI_PACKET_TYPE_TELEMETRY;
      txData[idx++] = SPI_PACKET_PAYLOAD_LEN; 
      
      /* 1. Basic Info (6 bytes) */
      memcpy(&txData[idx], &t_ms, 4); idx+=4;
      memcpy(&txData[idx], &seq_count, 2); idx+=2;
      seq_count++;
      
      /* 2. Orientation (12 bytes) */
      memcpy(&txData[idx], &local_imu_data.pitch, 4); idx+=4;
      memcpy(&txData[idx], &local_imu_data.roll, 4); idx+=4;
      memcpy(&txData[idx], &local_imu_data.yaw, 4); idx+=4;
      
      /* 3. Raw Accel (12 bytes) */
      memcpy(&txData[idx], &local_imu_data.ax, 4); idx+=4;
      memcpy(&txData[idx], &local_imu_data.ay, 4); idx+=4;
      memcpy(&txData[idx], &local_imu_data.az, 4); idx+=4;
      
      /* 4. Raw Gyro (12 bytes) */
      memcpy(&txData[idx], &local_imu_data.gx, 4); idx+=4;
      memcpy(&txData[idx], &local_imu_data.gy, 4); idx+=4;
      memcpy(&txData[idx], &local_imu_data.gz, 4); idx+=4;
      
      /* 5. GPS Ground Speed (4 bytes) */
      memcpy(&txData[idx], &local_gps_data.speed, 4); idx+=4;
      
      /* 6. GPS Position (20 bytes) */
      memcpy(&txData[idx], &local_gps_data.latitude, 8); idx+=8;
      memcpy(&txData[idx], &local_gps_data.longitude, 8); idx+=8;
      memcpy(&txData[idx], &local_gps_data.altitude, 4); idx+=4;
      
      /* 7. Status & Quality (8 bytes) */
      txData[idx++] = local_sys_status.imu_ok;
      txData[idx++] = local_sys_status.gps_ok;
      txData[idx++] = local_gps_data.fix;
      txData[idx++] = local_gps_data.satellites;
      memcpy(&txData[idx], &local_gps_data.hdop, 4); idx+=4;
      
      /* Checksum (XOR of bytes 0 through 76) */
      uint8_t checksum = 0;
      for(int i = 0; i < idx; i++) {
        checksum ^= txData[i];
      }
      txData[idx++] = checksum;
      txData[idx++] = SPI_PACKET_END_BYTE;
      
      /* Padding */
      txData[idx++] = 0x55;
      txData[idx++] = 0x55;
      
      /* Transmit */
      uint16_t total_len = idx;
      
      /* Sanity check: ensure we haven't overflowed the buffer */
      if (total_len > sizeof(txData)) {
        int len = sprintf(debug_msg, "[TEL_ERR] SPI buffer overflow! Size=%u, Buffer=%u\r\n", 
                         total_len, (unsigned int)sizeof(txData));
        Log_Msg(debug_msg, len);
        goto skip_spi_transmit;
      }
      
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); /* CS Low */
      
      if (HAL_SPI_Transmit(&hspi2, txData, total_len, SPI_TIMEOUT_MS) == HAL_OK) {
        spi_tx_success_count++;
        /* Update last packet buffer for diagnostics */
        memcpy(last_spi_packet, txData, (total_len < SPI_PACKET_TOTAL_LEN) ? total_len : SPI_PACKET_TOTAL_LEN);
      } else {
        spi_tx_error_count++;
         /* Only log error occasionally to avoid flooding */
        if (spi_tx_error_count % 100 == 0) {
            int len = sprintf(debug_msg, "[TEL_ERR] SPI Transmit Failed! ErrCnt=%lu\r\n", spi_tx_error_count);
            Log_Msg(debug_msg, len);
        }
      }
      
      while(__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_BSY));
      for(volatile int i=0; i<2000; i++) { __NOP(); } /* Short delay */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); /* CS High */
      
      skip_spi_transmit:
      last_telemetry_send = now;
    }
    
    TaskWatchdog_Feed("Telemetry_Task");
    osDelay(10);
  }
  /* USER CODE END StartTelemetry_Task */
}

/* USER CODE BEGIN Header_StartUART_Debug_Task */
/**
  * @brief  Function implementing the UART_Debug_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUART_Debug_Task */
void StartUART_Debug_Task(void const * argument)
{
  /* USER CODE BEGIN StartUART_Debug_Task */
  TaskWatchdog_Register("UART_Debug_Task");
  
  /* Wait for system to settle */
  osDelay(1000);
  
  char msg[512];
  uint32_t counter = 0;
  uint32_t health_report_counter = 0;
  static uint8_t diag_sent = 0;
  
  for(;;) {
    /* Send hardware diagnostics on first iteration */
    if (!diag_sent) {
      diag_sent = 1;
      osDelay(500);  /* Wait for other tasks to start */
      
      int len = sprintf(msg, "\r\n=============== HARDWARE DIAGNOSTICS ===============\r\n");
      Log_Msg(msg, len);
      
      len = sprintf(msg, "[DIAG] System started - checking peripherals...\r\n");
      Log_Msg(msg, len);
      
      len = sprintf(msg, "[DIAG] UART3 (Debug):   ON at 115200 baud\r\n");
      Log_Msg(msg, len);
      
      len = sprintf(msg, "[DIAG] I2C2 (IMU):      %s\r\n", sys_status.imu_ok ? "RESPONDING" : "NOT RESPONDING");
      Log_Msg(msg, len);
      
      len = sprintf(msg, "[DIAG] UART2 (GPS):     %s\r\n", sys_status.gps_ok ? "RECEIVING DATA" : "WAITING FOR DATA");
      Log_Msg(msg, len);
      
      len = sprintf(msg, "======================================================\r\n\r\n");
      Log_Msg(msg, len);
      
      osDelay(1000);  /* Space before telemetry starts */
    }
    
    /* Read sensor data directly (no mutex to avoid blocking) */
    float ax = imu_data.ax;
    float ay = imu_data.ay;
    float az = imu_data.az;
    float gx = imu_data.gx;
    float gy = imu_data.gy;
    float gz = imu_data.gz;
    float pitch = imu_data.pitch;
    float roll = imu_data.roll;
    float yaw = imu_data.yaw;
    
    double lat = gps_data.latitude;
    double lon = gps_data.longitude;
    float alt = gps_data.altitude;
    float speed = gps_data.speed;
    uint8_t fix = gps_data.fix;
    
    uint8_t imu_ok = sys_status.imu_ok;
    uint8_t gps_ok = sys_status.gps_ok;
    
    /* Line 1: Counter and timestamps */
    int len = sprintf(msg, "\r\n[%03lu] ========== TELEMETRY REPORT ==========\r\n", counter++);
    Log_Msg(msg, len);
    
    /* Line 2: IMU Status Header */
    len = sprintf(msg, "IMU: %s\r\n", imu_ok ? "[OK]" : "[ERROR]");
    Log_Msg(msg, len);
    
    /* Line 3: Accelerometer */
    len = sprintf(msg, "  Accel (m/s²): Ax=%+7.3f  Ay=%+7.3f  Az=%+7.3f\r\n", ax, ay, az);
    Log_Msg(msg, len);
    
    /* Line 4: Gyroscope */
    len = sprintf(msg, "  Gyro (°/s):  Gx=%+7.2f  Gy=%+7.2f  Gz=%+7.2f\r\n", gx, gy, gz);
    Log_Msg(msg, len);
    
    /* Line 5: Euler Angles */
    len = sprintf(msg, "  Angles (°):  Pitch=%+6.1f  Roll=%+6.1f  Yaw=%+6.1f\r\n", pitch, roll, yaw);
    Log_Msg(msg, len);
    
    /* Line 6: GPS Status Header */
    len = sprintf(msg, "GPS: %s (Fix=%d, Sats=%d)\r\n", gps_ok ? "[OK]" : "[WAIT]", fix, gps_data.satellites);
    Log_Msg(msg, len);
    
    /* Line 7: GPS Position */
    len = sprintf(msg, "  Position: Lat=%+10.5f°  Lon=%+11.5f°\r\n", lat, lon);
    Log_Msg(msg, len);
    
    /* Line 8: GPS Altitude & Speed */
    len = sprintf(msg, "  Alt=%7.1f m  Speed=%6.2f m/s\r\n", alt, speed);
    Log_Msg(msg, len);
    
    /* Health report every 10 telemetry updates */
    health_report_counter++;
    if (health_report_counter >= 10) {
      health_report_counter = 0;
      
      uint8_t imu_healthy = Health_IsIMUHealthy();
      uint8_t gps_healthy = Health_IsGPSHealthy();
      uint8_t system_healthy = Health_IsSystemHealthy();
      uint32_t uptime_ms = xTaskGetTickCount();
      uint32_t uptime_sec = uptime_ms / 1000;
      uint32_t uptime_ms_rem = uptime_ms % 1000;
      
      len = sprintf(msg,
        "\r\n>>> SYSTEM HEALTH <<<\r\n"
        "  Status:   %s\r\n"
        "  IMU:      %s\r\n"
        "  GPS:      %s\r\n"
        "  Uptime:   %lu.%03lu sec\r\n",
        system_healthy ? "HEALTHY" : "DEGRADED",
        imu_healthy ? "HEALTHY" : "STALE/ERROR",
        gps_healthy ? "ACQUIRING" : "STALE",
        uptime_sec, uptime_ms_rem);
      
      if (len > 0 && huart3.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, UART_TIMEOUT_MS);
      }
      
      /* Add SPI statistics */
      len = sprintf(msg,
        ">>> SPI STATISTICS <<<\r\n"
        "  TX Success: %lu\r\n"
        "  TX Errors:  %lu\r\n"
        "  Success Rate: %lu%%\r\n\r\n",
        spi_tx_success_count,
        spi_tx_error_count,
        (spi_tx_success_count + spi_tx_error_count) > 0 
          ? (spi_tx_success_count * 100) / (spi_tx_success_count + spi_tx_error_count)
          : 0);
      
      if (len > 0 && huart3.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, UART_TIMEOUT_MS);
      }
      
      /* Dump last SPI packet */
      len = sprintf(msg, "  Last Packet: ");
      for(int i=0; i<SPI_PACKET_TOTAL_LEN; i++) {
        len += sprintf(msg + len, "%02X ", last_spi_packet[i]);
      }
      len += sprintf(msg + len, "\r\n\r\n");
      
      if (huart3.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, UART_TIMEOUT_MS);
      }
    }
    
    TaskWatchdog_Feed("UART_Debug_Task");
    osDelay(pdMS_TO_TICKS(1000 / UART_DEBUG_TASK_FREQ_HZ));
  }
  /* USER CODE END StartUART_Debug_Task */
}

/* USER CODE BEGIN Header_StartLED_Task */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void const * argument)
{
  /* USER CODE BEGIN StartLED_Task */
  TaskWatchdog_Register("LED_Task");
  
  /* Simple LED blink - start immediately */
  for(;;) {
    /* Normal operation: slow blink (1 Hz) */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    osDelay(pdMS_TO_TICKS(1000 / LED_TASK_FREQ_HZ));
    
    /* Check for dead tasks (non-blocking) */
    uint8_t dead_tasks = TaskWatchdog_GetDeadTaskCount();
    
    if (dead_tasks > 0) {
      /* Rapid blink pattern to indicate dead tasks */
      for (int i = 0; i < dead_tasks && i < 5; i++) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        osDelay(100);
      }
      osDelay(500);
    }
    
    TaskWatchdog_Feed("LED_Task");
  }
  /* USER CODE END StartLED_Task */
}

/* USER CODE BEGIN Header_StartHealth_Task */
/**
  * @brief  Function implementing the Health_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHealth_Task */
void StartHealth_Task(void const * argument)
{
  /* USER CODE BEGIN StartHealth_Task */
  TaskWatchdog_Register("Health_Task");
  
  for(;;) {
    /* Update health status */
    Health_Update();
    
    TaskWatchdog_Feed("Health_Task");
    osDelay(pdMS_TO_TICKS(1000 / HEALTH_TASK_FREQ_HZ));
  }
  /* USER CODE END StartHealth_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */





/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Stack overflow hook function
  * @param  xTask: Task handle
  * @param  pcTaskName: Task name
  * @retval None
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  /* Stack overflow detected - blink LED rapidly */
  for(;;) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    for(volatile int i=0; i<100000; i++);
  }
}


/* USER CODE END Application */
