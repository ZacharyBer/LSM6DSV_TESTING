/**
  ******************************************************************************
  * @file    platform_i2c.c
  * @brief   Platform-specific I2C functions for LSM6DSV sensor
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform_i2c.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/* Function implementations --------------------------------------------------*/

/**
  * @brief  Platform specific read function
  * @param  handle  Platform specific handler (platform_ctx_t)
  * @param  reg     Register address
  * @param  data    Data buffer for read data
  * @param  len     Number of bytes to read
  * @retval 0 if success, non-zero otherwise
  */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    platform_ctx_t *pctx = (platform_ctx_t *)handle;
    HAL_StatusTypeDef status;
    uint8_t retry = 0;

    if (pctx == NULL || data == NULL) {
        return -1;
    }

    /* Perform I2C read with retry mechanism */
    do {
        /* HAL expects 7-bit address shifted left by 1 */
        status = HAL_I2C_Mem_Read(pctx->hi2c, pctx->address << 1, reg,
                                  I2C_MEMADD_SIZE_8BIT, data, len,
                                  pctx->timeout);
        if (status == HAL_OK) {
            return 0;
        }
        retry++;
        if (retry < LSM6DSV_MAX_RETRY) {
            platform_delay(1);
        }
    } while (retry < LSM6DSV_MAX_RETRY);

    /* Handle I2C error */
    if (pctx->hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
        /* Reset I2C peripheral if error occurred */
        HAL_I2C_DeInit(pctx->hi2c);
        HAL_I2C_Init(pctx->hi2c);
    }

    return -1;
}

/**
  * @brief  Platform specific write function
  * @param  handle  Platform specific handler (platform_ctx_t)
  * @param  reg     Register address
  * @param  data    Data to write
  * @param  len     Number of bytes to write
  * @retval 0 if success, non-zero otherwise
  */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    platform_ctx_t *pctx = (platform_ctx_t *)handle;
    HAL_StatusTypeDef status;
    uint8_t retry = 0;

    if (pctx == NULL || data == NULL) {
        return -1;
    }

    /* Perform I2C write with retry mechanism */
    do {
        /* HAL expects 7-bit address shifted left by 1 */
        status = HAL_I2C_Mem_Write(pctx->hi2c, pctx->address << 1, reg,
                                   I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len,
                                   pctx->timeout);
        if (status == HAL_OK) {
            return 0;
        }
        retry++;
        if (retry < LSM6DSV_MAX_RETRY) {
            platform_delay(1);
        }
    } while (retry < LSM6DSV_MAX_RETRY);

    /* Handle I2C error */
    if (pctx->hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
        /* Reset I2C peripheral if error occurred */
        HAL_I2C_DeInit(pctx->hi2c);
        HAL_I2C_Init(pctx->hi2c);
    }

    return -1;
}

/**
  * @brief  Platform specific delay function
  * @param  ms  Delay in milliseconds
  * @retval None
  */
void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief  Get platform timestamp in microseconds
  * @retval Timestamp in microseconds
  * @note   TIM2 is configured with prescaler=16-1 for 1MHz tick rate (1 tick = 1us)
  */
uint32_t platform_get_timestamp(void)
{
    /* TIM2 counter directly provides microseconds */
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
  * @brief  Initialize platform context
  * @param  pctx     Platform context
  * @param  hi2c     I2C handle
  * @param  address  I2C device address (7-bit, unshifted)
  * @retval 0 if success, non-zero otherwise
  */
int32_t platform_init(platform_ctx_t *pctx, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    if (pctx == NULL || hi2c == NULL) {
        return -1;
    }

    pctx->hi2c = hi2c;
    pctx->address = address;
    pctx->timeout = LSM6DSV_I2C_TIMEOUT_MS;

    return 0;
}

/**
  * @brief  Check if device is present on I2C bus
  * @param  pctx  Platform context
  * @retval 0 if device found, non-zero otherwise
  */
int32_t platform_i2c_check(platform_ctx_t *pctx)
{
    HAL_StatusTypeDef status;

    if (pctx == NULL) {
        return -1;
    }

    /* Check if device is ready on I2C bus */
    /* HAL expects 7-bit address shifted left by 1 */
    status = HAL_I2C_IsDeviceReady(pctx->hi2c, pctx->address << 1, 3, pctx->timeout);

    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  Direct register read for debugging
  * @param  pctx   Platform context
  * @param  reg    Register address
  * @param  value  Register value
  * @retval 0 if success, non-zero otherwise
  */
int32_t platform_read_register(platform_ctx_t *pctx, uint8_t reg, uint8_t *value)
{
    return platform_read(pctx, reg, value, 1);
}

/**
  * @brief  Direct register write for debugging
  * @param  pctx   Platform context
  * @param  reg    Register address
  * @param  value  Register value
  * @retval 0 if success, non-zero otherwise
  */
int32_t platform_write_register(platform_ctx_t *pctx, uint8_t reg, uint8_t value)
{
    return platform_write(pctx, reg, &value, 1);
}