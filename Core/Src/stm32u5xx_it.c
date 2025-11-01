/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32u5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm_protocol.h"
#include "sensor_manager.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern DMA_NodeTypeDef Node_GPDMA1_Channel0;
extern DMA_QListTypeDef List_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern comm_protocol_t comm_ctx;  /* Communication protocol context from main */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI Line10 interrupt.
  */
void EXTI10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI10_IRQn 0 */

  /* USER CODE END EXTI10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_Pin);
  /* USER CODE BEGIN EXTI10_IRQn 1 */

  /* USER CODE END EXTI10_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line11 interrupt.
  */
void EXTI11_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI11_IRQn 0 */

  /* USER CODE END EXTI11_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT2_Pin);
  /* USER CODE BEGIN EXTI11_IRQn 1 */

  /* USER CODE END EXTI11_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line13 interrupt.
  */
void EXTI13_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI13_IRQn 0 */

  /* USER CODE END EXTI13_IRQn 0 */
  BSP_PB_IRQHandler(BUTTON_USER);
  /* USER CODE BEGIN EXTI13_IRQn 1 */

  /* USER CODE END EXTI13_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* Check if UART IDLE line detected */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);  // Clear the IDLE flag

    /* IDLE detected - update write position from DMA counter */
    uart_rx_write_pos = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel0);
  }

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief  UART RX Complete Callback
 * @note   Called when a byte is received via UART interrupt
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        /* Buffer wrapped - DMA completed full 256-byte transfer */
        /* Update write position (DMA wrapped to beginning) */
        uart_rx_write_pos = UART_RX_BUFFER_SIZE;  // Wrapped, now at end

        /* Toggle LED to indicate buffer wrap (useful for debugging) */
        BSP_LED_Toggle(LED_BLUE);
    }
}

/**
 * @brief  GPIO EXTI Callback for LSM6DSV interrupts
 * @note   Called when INT1 or INT2 pin from LSM6DSV triggers
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_Pin || GPIO_Pin == INT2_Pin) {
        /* Read interrupt sources from sensor */
        sensor_manager_t *mgr = comm_ctx.sensor_mgr;

        if (mgr != NULL && mgr->initialized) {
            char event_msg[64];
            lsm6dsv_wake_up_src_t wake_src;
            lsm6dsv_tap_src_t tap_src;
            lsm6dsv_d6d_src_t d6d_src;
            lsm6dsv_embedded_status_t emb_status;

            /* Read wake-up source */
            if (lsm6dsv_read_reg(&mgr->ctx, LSM6DSV_WAKE_UP_SRC, (uint8_t*)&wake_src, 1) == 0) {
                if (wake_src.wu_ia) {
                    snprintf(event_msg, sizeof(event_msg), "INT:WAKE_UP\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
                if (wake_src.ff_ia) {
                    snprintf(event_msg, sizeof(event_msg), "INT:FREE_FALL\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
                if (wake_src.sleep_change_ia) {
                    snprintf(event_msg, sizeof(event_msg), "INT:SLEEP_CHANGE\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
            }

            /* Read tap source */
            if (lsm6dsv_read_reg(&mgr->ctx, LSM6DSV_TAP_SRC, (uint8_t*)&tap_src, 1) == 0) {
                if (tap_src.single_tap) {
                    snprintf(event_msg, sizeof(event_msg), "INT:SINGLE_TAP\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
                if (tap_src.double_tap) {
                    snprintf(event_msg, sizeof(event_msg), "INT:DOUBLE_TAP\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
            }

            /* Read 6D source */
            if (lsm6dsv_read_reg(&mgr->ctx, LSM6DSV_D6D_SRC, (uint8_t*)&d6d_src, 1) == 0) {
                if (d6d_src.d6d_ia) {
                    snprintf(event_msg, sizeof(event_msg), "INT:6D_ORIENT\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
            }

            /* Read embedded function status */
            if (lsm6dsv_embedded_status_get(&mgr->ctx, &emb_status) == 0) {
                if (emb_status.step_detector) {
                    snprintf(event_msg, sizeof(event_msg), "INT:STEP_DET\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
                if (emb_status.tilt) {
                    snprintf(event_msg, sizeof(event_msg), "INT:TILT\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
                if (emb_status.sig_mot) {
                    snprintf(event_msg, sizeof(event_msg), "INT:SIG_MOT\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)event_msg, strlen(event_msg), 100);
                }
            }

            /* Toggle LED to indicate interrupt processed */
            BSP_LED_Toggle(LED_BLUE);
        }
    }
}

/**
 * @brief  UART Error Callback
 * @note   Called when UART error occurs (framing, overrun, etc.)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        BSP_LED_Toggle(LED_RED);  // Debug: rapid red flashing = UART error
        /* Re-enable RX DMA after error with large circular buffer */
        uart_rx_write_pos = 0;  // Reset positions
        uart_rx_read_pos = 0;
        HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, UART_RX_BUFFER_SIZE);
    }
}

/* USER CODE END 1 */
