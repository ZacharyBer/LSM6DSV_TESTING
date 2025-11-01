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
#include <string.h>
#include <stdio.h>
#include "sensor_manager.h"
#include "comm_protocol.h"
#include "data_formatter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LSM6DSV I2C Addresses (7-bit, unshifted) */
#define LSM6DSV_I2C_ADDR_LOW  0x6A  // SDO/SA0 pin low (default)
#define LSM6DSV_I2C_ADDR_HIGH 0x6B  // SDO/SA0 pin high

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

/* Sensor Status */
static uint8_t sensor_initialized = 0;
static uint8_t streaming_enabled = 0;
static uint8_t current_i2c_address = LSM6DSV_I2C_ADDR_DEFAULT;

/* UART RX Buffer */
uint8_t uart_rx_byte;

/* New layer instances */
static sensor_manager_t sensor_mgr;
comm_protocol_t comm_ctx;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Send startup banner to confirm UART is working */
  const char* banner = "\r\n=== LSM6DSV Firmware v2.0 ===\r\n"
                       "UART: OK (921600 baud)\r\n"
                       "Initializing sensor...\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)banner, strlen(banner), 1000);

  /* Wait for sensor power-up */
  HAL_Delay(100);

  /* Scan I2C bus to see what devices are present */
  char scan_msg[128];
  int scan_len = snprintf(scan_msg, sizeof(scan_msg), "I2C Bus Scan:\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, scan_len, 1000);

  for (uint8_t addr = 1; addr < 128; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK) {
          scan_len = snprintf(scan_msg, sizeof(scan_msg), "  Found device at 0x%02X\r\n", addr);
          HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, scan_len, 1000);
      }
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)"I2C scan complete.\r\n\r\n", 21, 1000);

  /* Try both I2C addresses */
  uint8_t addresses[] = {LSM6DSV_I2C_ADDR_LOW, LSM6DSV_I2C_ADDR_HIGH};
  for (int i = 0; i < 2; i++) {
      current_i2c_address = addresses[i];

      /* Report which address we're trying */
      char probe_msg[128];
      int msg_len = snprintf(probe_msg, sizeof(probe_msg),
                             "Probing LSM6DSV at I2C address 0x%02X...\r\n",
                             current_i2c_address);
      HAL_UART_Transmit(&huart1, (uint8_t*)probe_msg, msg_len, 1000);

      /* Debug: Read WHO_AM_I register directly to see what's there */
      uint8_t whoami_test = 0;
      HAL_StatusTypeDef whoami_status;
      char whoami_msg[256];

      whoami_status = HAL_I2C_Mem_Read(&hi2c2, current_i2c_address << 1, 0x0F,
                                        I2C_MEMADD_SIZE_8BIT, &whoami_test, 1, 1000);

      uint32_t i2c_error = HAL_I2C_GetError(&hi2c2);
      int whoami_len = snprintf(whoami_msg, sizeof(whoami_msg),
                                "  WHO_AM_I(0x0F): HAL_status=%d, I2C_ErrorCode=0x%04lX, value=0x%02X\r\n"
                                "    (expect status=0, value=0x70 for LSM6DSV)\r\n",
                                whoami_status, i2c_error, whoami_test);
      HAL_UART_Transmit(&huart1, (uint8_t*)whoami_msg, whoami_len, 1000);

      /* Decode I2C error codes */
      if (i2c_error != HAL_I2C_ERROR_NONE) {
          char error_detail[256];
          int error_len = snprintf(error_detail, sizeof(error_detail),
                                    "    I2C Errors:%s%s%s%s%s\r\n",
                                    (i2c_error & HAL_I2C_ERROR_BERR) ? " BERR" : "",
                                    (i2c_error & HAL_I2C_ERROR_ARLO) ? " ARLO" : "",
                                    (i2c_error & HAL_I2C_ERROR_AF) ? " AF(NACK)" : "",
                                    (i2c_error & HAL_I2C_ERROR_OVR) ? " OVR" : "",
                                    (i2c_error & HAL_I2C_ERROR_TIMEOUT) ? " TIMEOUT" : "");
          HAL_UART_Transmit(&huart1, (uint8_t*)error_detail, error_len, 1000);
      }

      if (sensor_manager_init(&sensor_mgr, &hi2c2, current_i2c_address) == 0) {
          /* Success! */
          msg_len = snprintf(probe_msg, sizeof(probe_msg),
                            "  Result: SUCCESS - Sensor initialized\r\n"
                            "Starting data streaming...\r\n\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)probe_msg, msg_len, 1000);

          BSP_LED_On(LED_GREEN);
          sensor_initialized = 1;
          streaming_enabled = 1;

          /* Initialize comm_protocol */
          comm_protocol_init(&comm_ctx, &huart1, &sensor_mgr);
          break;
      } else {
          /* Failed */
          HAL_UART_Transmit(&huart1, (uint8_t*)"  Result: FAILED - Init failed\r\n\r\n", 35, 1000);
      }
  }

  if (!sensor_initialized) {
      /* Send detailed error message */
      const char* error_msg = "\r\n*** ERROR: LSM6DSV sensor not found! ***\r\n"
                              "Possible causes:\r\n"
                              "  1. Sensor not connected to I2C2 (PB10=SCL, PB11=SDA)\r\n"
                              "  2. Wrong I2C address (check SDO/SA0 pin)\r\n"
                              "  3. Missing I2C pull-up resistors\r\n"
                              "  4. Sensor not powered (check VDD)\r\n"
                              "  5. I2C wiring issue\r\n\r\n"
                              "Expected addresses: 0x6A (SDO=GND) or 0x6B (SDO=VDD)\r\n"
                              "Check I2C scan results above.\r\n\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), 1000);

      BSP_LED_On(LED_RED);
      streaming_enabled = 0;
  }

  /* Start UART RX interrupt for commands */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Process incoming UART commands */
    comm_protocol_process(&comm_ctx);

    /* Stream sensor data if enabled */
    if (sensor_initialized && streaming_enabled) {
        sensor_data_t data;

        /* Read sensor data */
        if (sensor_manager_read_data(&sensor_mgr, &data) == 0) {
            /* Format as CSV */
            char buffer[256];
            int len = data_formatter_csv_data(buffer, sizeof(buffer), &data, NULL, NULL);

            if (len > 0) {
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
                BSP_LED_Toggle(LED_BLUE);
            }

            /* Read SFLP if enabled */
            if (sensor_mgr.config.sflp_game_en) {
                sflp_data_t sflp;
                if (sensor_manager_read_sflp(&sensor_mgr, &sflp) == 0) {
                    len = data_formatter_csv_sflp(buffer, sizeof(buffer), &sflp, data.timestamp);
                    if (len > 0) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
                    }
                }
            }
        }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Small delay to prevent excessive polling and allow sensor data to be ready
     * At 120Hz ODR, new data available every ~8.3ms
     * 1ms delay gives good responsiveness while preventing CPU thrashing */
    HAL_Delay(1);
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
