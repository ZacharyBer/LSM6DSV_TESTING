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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "lsm6dsv_reg.h"
#include "platform_i2c.h"
#include "sensor_manager.h"
#include "comm_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LSM6DSV I2C Addresses */
#define LSM6DSV_I2C_ADDR_LOW  (0x6A << 1)  // SDO/SA0 pin low (default)
#define LSM6DSV_I2C_ADDR_HIGH (0x6B << 1)  // SDO/SA0 pin high

/* Default I2C address - can be changed via command */
#define LSM6DSV_I2C_ADDR_DEFAULT  LSM6DSV_I2C_ADDR_HIGH

/* UART TX Buffer Size */
#define UART_TX_BUFFER_SIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* LSM6DSV Sensor Context */
static stmdev_ctx_t lsm6dsv_ctx;
static platform_ctx_t platform_ctx __attribute__((unused)); /* Initialized for future use */

/* Sensor Data Buffers */
static int16_t accel_raw[3] = {0};
static int16_t gyro_raw[3] = {0};
static float accel_mg[3] = {0.0f, 0.0f, 0.0f};
static float gyro_mdps[3] = {0.0f, 0.0f, 0.0f};
static uint8_t data_valid = 0;  /* Flag to indicate if sensor data has been read at least once */
static int16_t temperature_raw __attribute__((unused)); /* Reserved for future use */

/* SFLP Data (Sensor Fusion) */
static float sflp_quat[4];  // w, x, y, z
static uint8_t sflp_enabled = 0;

/* Streaming Control */
static uint8_t streaming_enabled = 0;

/* UART Buffers */
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart_rx_byte;  // Single byte for UART RX interrupt

/* Sensor Status */
static uint8_t sensor_initialized = 0;
static uint8_t current_i2c_address = LSM6DSV_I2C_ADDR_DEFAULT;

/* Interrupt Flags */
static volatile uint8_t int1_flag = 0;
static volatile uint8_t int2_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* LSM6DSV Functions */
static int32_t LSM6DSV_AutoDetectI2C(void);
static int32_t LSM6DSV_Init(void);
static void LSM6DSV_ReadData(void);
static void LSM6DSV_ReadSFLP(void);
static void LSM6DSV_PrintData(void);

/* Utility Functions */
static void UART_Transmit(uint8_t *data, uint16_t len);
static uint32_t Get_Microseconds(void);

/* I2C Platform Functions */
static int32_t platform_write_wrapper(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read_wrapper(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 * I2C Platform Functions
 * ============================================================================ */

/**
 * @brief  Write data to I2C device
 */
static int32_t platform_write_wrapper(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t device_addr = (uint8_t)(uintptr_t)handle;

    if (HAL_I2C_Mem_Write(&hi2c2, device_addr, reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)bufp, len, 1000) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief  Read data from I2C device
 */
static int32_t platform_read_wrapper(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t device_addr = (uint8_t)(uintptr_t)handle;

    if (HAL_I2C_Mem_Read(&hi2c2, device_addr, reg, I2C_MEMADD_SIZE_8BIT,
                         bufp, len, 1000) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief  Get microsecond timestamp from TIM2
 */
static uint32_t Get_Microseconds(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief  Transmit data via UART
 */
static void UART_Transmit(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, data, len, 1000);
}

/* ============================================================================
 * LSM6DSV Functions
 * ============================================================================ */

/**
 * @brief  Auto-detect LSM6DSV I2C address
 * @retval 0 on success with address detected, -1 if not found
 */
static int32_t LSM6DSV_AutoDetectI2C(void)
{
    uint8_t whoami;
    uint8_t addresses[] = {LSM6DSV_I2C_ADDR_LOW, LSM6DSV_I2C_ADDR_HIGH};

    /* Try both possible I2C addresses */
    for (int i = 0; i < 2; i++) {
        /* Setup temporary context for this address */
        stmdev_ctx_t temp_ctx;
        temp_ctx.write_reg = platform_write_wrapper;
        temp_ctx.read_reg = platform_read_wrapper;
        temp_ctx.handle = (void*)(uintptr_t)addresses[i];

        /* Try to read WHO_AM_I register (0x0F) */
        if (lsm6dsv_device_id_get(&temp_ctx, &whoami) == 0) {
            /* Verify it's actually an LSM6DSV (WHO_AM_I should be 0x70) */
            if (whoami == LSM6DSV_ID) {
                current_i2c_address = addresses[i];
                return 0;  /* Success! */
            }
        }
    }

    /* No LSM6DSV found at either address */
    return -1;
}

/**
 * @brief  Initialize LSM6DSV sensor
 * @retval 0 on success, -1 on error
 */
static int32_t LSM6DSV_Init(void)
{
    uint8_t whoami;
    int32_t ret;

    /* Setup platform context */
    lsm6dsv_ctx.write_reg = platform_write_wrapper;
    lsm6dsv_ctx.read_reg = platform_read_wrapper;
    lsm6dsv_ctx.handle = (void*)(uintptr_t)current_i2c_address;

    /* Check WHO_AM_I */
    ret = lsm6dsv_device_id_get(&lsm6dsv_ctx, &whoami);
    if (ret != 0 || whoami != LSM6DSV_ID) {
        return -1;
    }

    /* Software reset */
    lsm6dsv_reset_set(&lsm6dsv_ctx, LSM6DSV_RESTORE_CTRL_REGS);
    HAL_Delay(10);

    /* Wait for reset to complete */
    lsm6dsv_reset_t rst;
    do {
        lsm6dsv_reset_get(&lsm6dsv_ctx, &rst);
        HAL_Delay(1);
    } while (rst != LSM6DSV_READY);

    /* Enable Block Data Update */
    lsm6dsv_block_data_update_set(&lsm6dsv_ctx, PROPERTY_ENABLE);

    /* Set accelerometer: MODE first, then full scale, then ODR (turns on sensor) */
    lsm6dsv_xl_mode_set(&lsm6dsv_ctx, LSM6DSV_XL_HIGH_PERFORMANCE_MD);
    lsm6dsv_xl_full_scale_set(&lsm6dsv_ctx, LSM6DSV_4g);
    lsm6dsv_xl_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_120Hz);

    /* Set gyroscope: MODE first, then full scale, then ODR (turns on sensor) */
    lsm6dsv_gy_mode_set(&lsm6dsv_ctx, LSM6DSV_GY_HIGH_PERFORMANCE_MD);
    lsm6dsv_gy_full_scale_set(&lsm6dsv_ctx, LSM6DSV_2000dps);
    lsm6dsv_gy_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_120Hz);

    /* Wait for sensor to stabilize and start sampling */
    HAL_Delay(10);

    sensor_initialized = 1;

    return 0;
}

/**
 * @brief  Read accelerometer and gyroscope data
 * @note   Event-driven: only prints when both accel AND gyro data are ready
 */
static void LSM6DSV_ReadData(void)
{
    lsm6dsv_data_ready_t drdy;

    /* Check if data is ready */
    lsm6dsv_flag_data_ready_get(&lsm6dsv_ctx, &drdy);

    /* Only process and print if BOTH accelerometer AND gyroscope data are ready */
    if (drdy.drdy_xl && drdy.drdy_gy) {
        /* Read accelerometer data */
        lsm6dsv_acceleration_raw_get(&lsm6dsv_ctx, accel_raw);

        /* Convert to mg (±4g range) */
        accel_mg[0] = lsm6dsv_from_fs4_to_mg(accel_raw[0]);
        accel_mg[1] = lsm6dsv_from_fs4_to_mg(accel_raw[1]);
        accel_mg[2] = lsm6dsv_from_fs4_to_mg(accel_raw[2]);

        /* Read gyroscope data */
        lsm6dsv_angular_rate_raw_get(&lsm6dsv_ctx, gyro_raw);

        /* Convert to mdps (±2000dps range) */
        gyro_mdps[0] = lsm6dsv_from_fs2000_to_mdps(gyro_raw[0]);
        gyro_mdps[1] = lsm6dsv_from_fs2000_to_mdps(gyro_raw[1]);
        gyro_mdps[2] = lsm6dsv_from_fs2000_to_mdps(gyro_raw[2]);

        /* Mark data as valid */
        data_valid = 1;

        /* Print data immediately when ready (event-driven) */
        if (streaming_enabled) {
            LSM6DSV_PrintData();
            BSP_LED_Toggle(LED_BLUE);  /* Visual feedback: blink blue LED on data output */
        }
    }
}

/**
 * @brief  Read SFLP (Sensor Fusion) data
 * @note   SFLP quaternion reading not implemented - driver API not available
 */
static void LSM6DSV_ReadSFLP(void)
{
    /* SFLP quaternion data reading would go here */
    /* This driver version doesn't expose the quaternion data registers */
    /* SFLP can still be enabled via commands for future use */
    (void)sflp_enabled;  /* Suppress warning */
}

/**
 * @brief  Print sensor data via UART in CSV format
 */
static void LSM6DSV_PrintData(void)
{
    uint32_t timestamp = Get_Microseconds();
    int len;

    /* Only print if we have valid data */
    if (!data_valid) {
        return;
    }

    /* Clear buffer to ensure clean state */
    memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE);

    /* Print accelerometer and gyroscope data */
    len = sprintf((char*)uart_tx_buffer,
                  "LSM6DSV,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                  timestamp,
                  accel_mg[0], accel_mg[1], accel_mg[2],
                  gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);

    /* Validate sprintf succeeded */
    if (len > 0 && len < UART_TX_BUFFER_SIZE) {
        UART_Transmit(uart_tx_buffer, len);
    } else {
        /* sprintf failed - send error message */
        const char *error_msg = "ERROR: sprintf failed in LSM6DSV_PrintData\r\n";
        UART_Transmit((uint8_t*)error_msg, strlen(error_msg));
    }

    /* Print SFLP data if enabled */
    if (sflp_enabled) {
        memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE);
        len = sprintf((char*)uart_tx_buffer,
                      "LSM6DSV_SFLP,%lu,%.4f,%.4f,%.4f,%.4f\r\n",
                      timestamp,
                      sflp_quat[0], sflp_quat[1], sflp_quat[2], sflp_quat[3]);

        if (len > 0 && len < UART_TX_BUFFER_SIZE) {
            UART_Transmit(uart_tx_buffer, len);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_ICACHE_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Start TIM2 for microsecond timestamps */
  HAL_TIM_Base_Start(&htim2);

  /* Initialize LEDs */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Auto-detect LSM6DSV I2C address */
  HAL_Delay(100);  /* Wait for sensor power-up */

  if (LSM6DSV_AutoDetectI2C() == 0) {
      /* Initialize LSM6DSV sensor at detected address */
      if (LSM6DSV_Init() == 0) {
          BSP_LED_On(LED_GREEN);  /* Green LED = sensor OK */
          streaming_enabled = 1;   /* Start streaming by default */
      } else {
          BSP_LED_On(LED_RED);    /* Red LED = sensor init error */
          streaming_enabled = 0;
      }
  } else {
      BSP_LED_On(LED_RED);    /* Red LED = sensor not found */
      streaming_enabled = 0;
  }

  /* Start UART RX interrupt for command reception */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Read sensor data (now event-driven: prints when data ready) */
    if (sensor_initialized) {
        LSM6DSV_ReadData();
        LSM6DSV_ReadSFLP();
    }

    /* Note: Data printing now happens inside LSM6DSV_ReadData() when DRDY flags are set */
    /* This provides more accurate timing and ensures we only print valid data */

    /* Process incoming commands */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 8;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0010061A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_DBn_GPIO_Port, UCPD_DBn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_DBn_Pin */
  GPIO_InitStruct.Pin = UCPD_DBn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_DBn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI10_IRQn);

  HAL_NVIC_SetPriority(EXTI11_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
