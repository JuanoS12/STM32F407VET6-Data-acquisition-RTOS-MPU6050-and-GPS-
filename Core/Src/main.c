/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USB_ForceReEnumeration(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure PA12 (USB DP) as Output Push-Pull */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Drive PA12 Low to force disconnect */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(100); // Wait 100ms
    
    /* De-initialize PA12 to let USB peripheral take over */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* CRITICAL: Initialize LED BEFORE anything else for early diagnostics */
  /* This runs before HAL_Init(), using direct register access */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  /* Enable GPIOD clock */
  GPIOD->MODER |= (1 << 24);  /* PD12 as output (MODER[25:24] = 01) */
  GPIOD->MODER &= ~(1 << 25);
  
  /* Blink 3 times VERY FAST to show we reached main() */
  for(int i = 0; i < 3; i++) {
    GPIOD->BSRR = (1 << 12);  /* Set PD12 HIGH */
    for(volatile int d = 0; d < 100000; d++);  /* Short delay */
    GPIOD->BSRR = (1 << (12+16));  /* Set PD12 LOW */
    for(volatile int d = 0; d < 100000; d++);
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Blink 2 times to show HAL_Init() succeeded */
  for(int i = 0; i < 2; i++) {
    GPIOD->BSRR = (1 << 12);
    for(volatile int d = 0; d < 200000; d++);
    GPIOD->BSRR = (1 << (12+16));
    for(volatile int d = 0; d < 200000; d++);
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Blink 1 long time to show SystemClock_Config() succeeded */
  GPIOD->BSRR = (1 << 12);
  for(volatile int d = 0; d < 1000000; d++);
  GPIOD->BSRR = (1 << (12+16));
  for(volatile int d = 0; d < 500000; d++);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  USB_ForceReEnumeration();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  /* Simple LED test before FreeRTOS starts */
  for(int i = 0; i < 5; i++) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   /* LED ON */
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);  /* LED OFF */
    HAL_Delay(200);
  }

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  
  /* CRITICAL FIX: Try HSE first, fallback to HSI if HSE fails */
  /* This prevents boot hang if external crystal is missing/faulty */
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // PLLQ must be set so that VCO/PLLQ = 48MHz for USB
  // VCO = HSE_VALUE / PLLM * PLLN = 8MHz / 4 * 72 = 144MHz
  // 144MHz / 3 = 48MHz (USB clock OK)
  // 144MHz / 2 = 72MHz (System clock)
  RCC_OscInitStruct.PLL.PLLM = 4;   // 8MHz / 4 = 2MHz
  RCC_OscInitStruct.PLL.PLLN = 72;  // 2MHz * 72 = 144MHz VCO
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 144MHz / 2 = 72MHz
  RCC_OscInitStruct.PLL.PLLQ = 3; // 144MHz / 3 = 48MHz
  
  HAL_StatusTypeDef hse_status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  if (hse_status != HAL_OK)
  {
    /* HSE FAILED - Blink LED rapidly 10 times to indicate HSE failure */
    for(int i = 0; i < 10; i++) {
      GPIOD->BSRR = (1 << 12);
      for(volatile int d = 0; d < 50000; d++);
      GPIOD->BSRR = (1 << (12+16));
      for(volatile int d = 0; d < 50000; d++);
    }
    
    /* Fallback to HSI (internal 16MHz oscillator) */
    /* HSI PLL configuration for 168MHz system clock */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    // VCO = HSI / PLLM * PLLN = 16MHz / 8 * 168 = 336MHz
    // SYSCLK = VCO / PLLP = 336MHz / 2 = 168MHz
    // USB = VCO / PLLQ = 336MHz / 7 = 48MHz
    RCC_OscInitStruct.PLL.PLLM = 8;    // 16MHz / 8 = 2MHz
    RCC_OscInitStruct.PLL.PLLN = 168;  // 2MHz * 168 = 336MHz VCO
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 336MHz / 2 = 168MHz
    RCC_OscInitStruct.PLL.PLLQ = 7;    // 336MHz / 7 = 48MHz
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    
    /* Blink LED slowly 3 times to indicate HSI mode */
    for(int i = 0; i < 3; i++) {
      GPIOD->BSRR = (1 << 12);
      for(volatile int d = 0; d < 300000; d++);
      GPIOD->BSRR = (1 << (12+16));
      for(volatile int d = 0; d < 300000; d++);
    }
  }
  else
  {
    /* HSE SUCCESS - Blink LED 2 times to indicate HSE mode */
    for(int i = 0; i < 2; i++) {
      GPIOD->BSRR = (1 << 12);
      for(volatile int d = 0; d < 300000; d++);
      GPIOD->BSRR = (1 << (12+16));
      for(volatile int d = 0; d < 300000; d++);
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  // Increased divider for higher clock
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  // Increased divider for higher clock

  /* Adjust flash latency based on actual clock speed */
  uint32_t flash_latency = (hse_status == HAL_OK) ? FLASH_LATENCY_2 : FLASH_LATENCY_5;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
  /* Initialize LED Pins (PD12, PA6, PA7) just in case */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN;
  
  /* PD12 as output */
  GPIOD->MODER |= (1 << 24);
  GPIOD->MODER &= ~(1 << 25);
  
  /* PA6, PA7 as outputs */
  GPIOA->MODER |= (1 << 12) | (1 << 14);
  GPIOA->MODER &= ~((1 << 13) | (1 << 15));
  
  while (1)
  {
    /* Fast Blink (100ms) = Error_Handler */
    GPIOD->ODR ^= (1 << 12);  /* Toggle PD12 */
    GPIOA->ODR ^= (1 << 6);   /* Toggle PA6 */
    GPIOA->ODR ^= (1 << 7);   /* Toggle PA7 */
    for(volatile int d = 0; d < 1000000; d++);  /* Delay ~100ms */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
