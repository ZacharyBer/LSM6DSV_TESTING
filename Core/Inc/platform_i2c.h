/**
  ******************************************************************************
  * @file    platform_i2c.h
  * @brief   Platform-specific I2C functions for LSM6DSV sensor
  ******************************************************************************
  */

#ifndef PLATFORM_I2C_H
#define PLATFORM_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lsm6dsv_reg.h"
#include <stdint.h>

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;

/* Defines -------------------------------------------------------------------*/
#define LSM6DSV_I2C_TIMEOUT_MS      1000
#define LSM6DSV_MAX_RETRY           3

/* Platform specific context structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint32_t timeout;
} platform_ctx_t;

/* Function prototypes -------------------------------------------------------*/
/* Platform specific read function */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

/* Platform specific write function */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len);

/* Platform specific delay function */
void platform_delay(uint32_t ms);

/* Platform specific timestamp function */
uint32_t platform_get_timestamp(void);

/* Platform initialization */
int32_t platform_init(platform_ctx_t *pctx, I2C_HandleTypeDef *hi2c, uint8_t address);

/* I2C bus check */
int32_t platform_i2c_check(platform_ctx_t *pctx);

/* Direct register read (for debugging) */
int32_t platform_read_register(platform_ctx_t *pctx, uint8_t reg, uint8_t *value);

/* Direct register write (for debugging) */
int32_t platform_write_register(platform_ctx_t *pctx, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_I2C_H */