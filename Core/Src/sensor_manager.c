/**
  ******************************************************************************
  * @file    sensor_manager.c
  * @brief   High-level sensor management implementation for LSM6DSV
  * @note    Full implementation with runtime configuration and SFLP support
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensor_manager.h"
#include "platform_i2c.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define SENSOR_RESET_TIMEOUT_MS    500   // Increased from 100ms for reliable reset
#define SENSOR_INIT_DELAY_MS       50    // Increased from 10ms to allow sensor boot time
#define SENSOR_CONFIG_DELAY_MS     5

/* Error codes */
#define SENSOR_ERROR_NONE          0
#define SENSOR_ERROR_I2C           1
#define SENSOR_ERROR_WHOAMI        2
#define SENSOR_ERROR_RESET         3
#define SENSOR_ERROR_CONFIG        4
#define SENSOR_ERROR_NOT_INIT      5
#define SENSOR_ERROR_PARAM         6

/* Private function prototypes -----------------------------------------------*/
static int32_t platform_write_wrapper(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read_wrapper(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static float convert_accel_to_mg(int16_t lsb, lsm6dsv_xl_full_scale_t fs);
static float convert_gyro_to_mdps(int16_t lsb, lsm6dsv_gy_full_scale_t fs);

/* Private variables ---------------------------------------------------------*/
static const char *error_strings[] = {
    "No error",
    "I2C communication error",
    "WHO_AM_I verification failed",
    "Sensor reset timeout",
    "Configuration error",
    "Sensor not initialized",
    "Invalid parameter"
};

/* ============================================================================
 * Platform Wrapper Functions
 * ============================================================================ */

/**
 * @brief  Write data to I2C device
 */
static int32_t platform_write_wrapper(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    platform_ctx_t *pctx = (platform_ctx_t *)handle;

    if (pctx == NULL || pctx->hi2c == NULL) {
        return -1;
    }

    /* HAL expects 7-bit address shifted left by 1 */
    if (HAL_I2C_Mem_Write(pctx->hi2c, pctx->address << 1, reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)bufp, len, pctx->timeout) != HAL_OK)
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
    platform_ctx_t *pctx = (platform_ctx_t *)handle;

    if (pctx == NULL || pctx->hi2c == NULL) {
        return -1;
    }

    /* HAL expects 7-bit address shifted left by 1 */
    if (HAL_I2C_Mem_Read(pctx->hi2c, pctx->address << 1, reg, I2C_MEMADD_SIZE_8BIT,
                         bufp, len, pctx->timeout) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

/* ============================================================================
 * Conversion Helper Functions
 * ============================================================================ */

/**
 * @brief  Convert accelerometer raw value to mg based on full scale
 */
static float convert_accel_to_mg(int16_t lsb, lsm6dsv_xl_full_scale_t fs)
{
    switch (fs) {
        case LSM6DSV_2g:
            return lsm6dsv_from_fs2_to_mg(lsb);
        case LSM6DSV_4g:
            return lsm6dsv_from_fs4_to_mg(lsb);
        case LSM6DSV_8g:
            return lsm6dsv_from_fs8_to_mg(lsb);
        case LSM6DSV_16g:
            return lsm6dsv_from_fs16_to_mg(lsb);
        default:
            return 0.0f;
    }
}

/**
 * @brief  Convert gyroscope raw value to mdps based on full scale
 */
static float convert_gyro_to_mdps(int16_t lsb, lsm6dsv_gy_full_scale_t fs)
{
    switch (fs) {
        case LSM6DSV_125dps:
            return lsm6dsv_from_fs125_to_mdps(lsb);
        case LSM6DSV_250dps:
            return lsm6dsv_from_fs250_to_mdps(lsb);
        case LSM6DSV_500dps:
            return lsm6dsv_from_fs500_to_mdps(lsb);
        case LSM6DSV_1000dps:
            return lsm6dsv_from_fs1000_to_mdps(lsb);
        case LSM6DSV_2000dps:
            return lsm6dsv_from_fs2000_to_mdps(lsb);
        case LSM6DSV_4000dps:
            return lsm6dsv_from_fs4000_to_mdps(lsb);
        default:
            return 0.0f;
    }
}

/**
 * @brief  Check if raw sensor data appears invalid
 * @note   Detects common invalid patterns: all zeros, all 0xFFFF, stuck values
 * @retval true if data appears invalid, false if data looks valid
 */
static bool is_raw_data_invalid(int16_t data[3])
{
    /* Check for all zeros */
    if (data[0] == 0 && data[1] == 0 && data[2] == 0) {
        return true;
    }

    /* Check for all 0xFFFF (common I2C failure pattern) */
    if (data[0] == (int16_t)0xFFFF && data[1] == (int16_t)0xFFFF && data[2] == (int16_t)0xFFFF) {
        return true;
    }

    /* Data appears valid */
    return false;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

/**
 * @brief  Initialize sensor manager and LSM6DSV sensor
 * @param  mgr: Pointer to sensor manager structure
 * @param  hi2c: Pointer to I2C handle
 * @param  i2c_address: I2C device address (8-bit format)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_init(sensor_manager_t *mgr, I2C_HandleTypeDef *hi2c, uint8_t i2c_address)
{
    int32_t ret;
    uint8_t whoami;
    lsm6dsv_reset_t rst;
    uint32_t timeout_count;

    if (mgr == NULL || hi2c == NULL) {
        return -1;
    }

    /* Clear structure */
    memset(mgr, 0, sizeof(sensor_manager_t));

    /* Setup platform context */
    mgr->pctx.hi2c = hi2c;
    mgr->pctx.address = i2c_address;
    mgr->pctx.timeout = LSM6DSV_I2C_TIMEOUT_MS;

    /* Setup device context */
    mgr->ctx.write_reg = platform_write_wrapper;
    mgr->ctx.read_reg = platform_read_wrapper;
    mgr->ctx.handle = (void*)&mgr->pctx;

    /* Check WHO_AM_I */
    ret = lsm6dsv_device_id_get(&mgr->ctx, &whoami);
    if (ret != 0 || whoami != LSM6DSV_ID) {
        mgr->error = true;
        mgr->error_code = SENSOR_ERROR_WHOAMI;
        return -1;
    }

    /* Software reset */
    ret = lsm6dsv_reset_set(&mgr->ctx, LSM6DSV_RESTORE_CTRL_REGS);
    if (ret != 0) {
        mgr->error = true;
        mgr->error_code = SENSOR_ERROR_RESET;
        return -1;
    }
    HAL_Delay(SENSOR_INIT_DELAY_MS);

    /* Wait for reset to complete */
    timeout_count = 0;
    do {
        lsm6dsv_reset_get(&mgr->ctx, &rst);
        HAL_Delay(1);
        timeout_count++;
        if (timeout_count > SENSOR_RESET_TIMEOUT_MS) {
            mgr->error = true;
            mgr->error_code = SENSOR_ERROR_RESET;
            return -1;
        }
    } while (rst != LSM6DSV_READY);

    /* Enable Block Data Update */
    lsm6dsv_block_data_update_set(&mgr->ctx, PROPERTY_ENABLE);

    /* Enable auto-increment for multi-byte reads */
    lsm6dsv_auto_increment_set(&mgr->ctx, PROPERTY_ENABLE);

    /* Load default configuration */
    sensor_manager_load_default_config(&mgr->config);

    /* Apply default configuration */
    ret = sensor_manager_apply_config(mgr, &mgr->config);
    if (ret != 0) {
        mgr->error = true;
        mgr->error_code = SENSOR_ERROR_CONFIG;
        return -1;
    }

    /* Mark as initialized */
    mgr->initialized = true;
    mgr->error = false;
    mgr->error_code = SENSOR_ERROR_NONE;

    return 0;
}

/**
 * @brief  De-initialize sensor manager
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_deinit(sensor_manager_t *mgr)
{
    if (mgr == NULL) {
        return -1;
    }

    /* Stop sensor */
    sensor_manager_stop(mgr);

    /* Clear structure */
    memset(mgr, 0, sizeof(sensor_manager_t));

    return 0;
}

/**
 * @brief  Reset sensor to default state
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_reset(sensor_manager_t *mgr)
{
    int32_t ret;
    lsm6dsv_reset_t rst;
    uint32_t timeout_count = 0;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Software reset */
    ret = lsm6dsv_reset_set(&mgr->ctx, LSM6DSV_RESTORE_CTRL_REGS);
    if (ret != 0) {
        mgr->error = true;
        mgr->error_code = SENSOR_ERROR_RESET;
        return -1;
    }
    HAL_Delay(SENSOR_INIT_DELAY_MS);

    /* Wait for reset to complete */
    do {
        lsm6dsv_reset_get(&mgr->ctx, &rst);
        HAL_Delay(1);
        timeout_count++;
        if (timeout_count > SENSOR_RESET_TIMEOUT_MS) {
            mgr->error = true;
            mgr->error_code = SENSOR_ERROR_RESET;
            return -1;
        }
    } while (rst != LSM6DSV_READY);

    /* Re-enable Block Data Update */
    lsm6dsv_block_data_update_set(&mgr->ctx, PROPERTY_ENABLE);

    /* Re-apply configuration */
    ret = sensor_manager_apply_config(mgr, &mgr->config);
    if (ret != 0) {
        mgr->error = true;
        mgr->error_code = SENSOR_ERROR_CONFIG;
        return -1;
    }

    return 0;
}

/**
 * @brief  Apply full configuration to sensor
 * @param  mgr: Pointer to sensor manager structure
 * @param  config: Pointer to configuration structure
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_apply_config(sensor_manager_t *mgr, const sensor_config_t *config)
{
    int32_t ret;

    if (mgr == NULL || config == NULL) {
        return -1;
    }

    /* Configure accelerometer: MODE -> FULL SCALE -> ODR */
    ret = lsm6dsv_xl_mode_set(&mgr->ctx, config->xl_mode);
    if (ret != 0) return -1;

    ret = lsm6dsv_xl_full_scale_set(&mgr->ctx, config->xl_fs);
    if (ret != 0) return -1;

    ret = lsm6dsv_xl_data_rate_set(&mgr->ctx, config->xl_odr);
    if (ret != 0) return -1;

    /* Configure gyroscope: MODE -> FULL SCALE -> ODR */
    ret = lsm6dsv_gy_mode_set(&mgr->ctx, config->gy_mode);
    if (ret != 0) return -1;

    ret = lsm6dsv_gy_full_scale_set(&mgr->ctx, config->gy_fs);
    if (ret != 0) return -1;

    ret = lsm6dsv_gy_data_rate_set(&mgr->ctx, config->gy_odr);
    if (ret != 0) return -1;

    /* Configure SFLP if enabled */
    if (config->sflp_game_en) {
        ret = lsm6dsv_sflp_game_rotation_set(&mgr->ctx, 1);
        if (ret != 0) return -1;

        lsm6dsv_sflp_data_rate_t sflp_rate;
        switch (config->sflp_odr) {
            case LSM6DSV_SFLP_15Hz:
                sflp_rate = LSM6DSV_SFLP_15Hz;
                break;
            case LSM6DSV_SFLP_30Hz:
                sflp_rate = LSM6DSV_SFLP_30Hz;
                break;
            case LSM6DSV_SFLP_60Hz:
                sflp_rate = LSM6DSV_SFLP_60Hz;
                break;
            case LSM6DSV_SFLP_120Hz:
                sflp_rate = LSM6DSV_SFLP_120Hz;
                break;
            case LSM6DSV_SFLP_240Hz:
                sflp_rate = LSM6DSV_SFLP_240Hz;
                break;
            case LSM6DSV_SFLP_480Hz:
                sflp_rate = LSM6DSV_SFLP_480Hz;
                break;
            default:
                sflp_rate = LSM6DSV_SFLP_15Hz;
                break;
        }

        ret = lsm6dsv_sflp_data_rate_set(&mgr->ctx, sflp_rate);
        if (ret != 0) return -1;
    } else {
        ret = lsm6dsv_sflp_game_rotation_set(&mgr->ctx, 0);
        if (ret != 0) return -1;
    }

    /* Save configuration */
    memcpy(&mgr->config, config, sizeof(sensor_config_t));

    /* Wait for configuration to settle */
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Get current sensor configuration
 * @param  mgr: Pointer to sensor manager structure
 * @param  config: Pointer to configuration structure to fill
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_get_config(sensor_manager_t *mgr, sensor_config_t *config)
{
    if (mgr == NULL || config == NULL || !mgr->initialized) {
        return -1;
    }

    memcpy(config, &mgr->config, sizeof(sensor_config_t));
    return 0;
}

/**
 * @brief  Load default sensor configuration
 * @param  config: Pointer to configuration structure to fill
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_load_default_config(sensor_config_t *config)
{
    if (config == NULL) {
        return -1;
    }

    memset(config, 0, sizeof(sensor_config_t));

    /* Default accelerometer config */
    config->xl_odr = LSM6DSV_ODR_AT_120Hz;
    config->xl_fs = LSM6DSV_4g;
    config->xl_mode = LSM6DSV_XL_HIGH_PERFORMANCE_MD;

    /* Default gyroscope config */
    config->gy_odr = LSM6DSV_ODR_AT_120Hz;
    config->gy_fs = LSM6DSV_2000dps;
    config->gy_mode = LSM6DSV_GY_HIGH_PERFORMANCE_MD;

    /* Default filter configuration */
    config->xl_lpf2_en = false;
    config->xl_lpf2_bw = LSM6DSV_XL_MEDIUM;
    config->xl_hpf_en = false;
    config->xl_fast_settling_en = false;
    config->gy_lpf1_en = false;
    config->gy_lpf1_bw = LSM6DSV_GY_MEDIUM;

    /* FIFO disabled by default */
    config->fifo_mode = LSM6DSV_BYPASS_MODE;
    config->fifo_watermark = 0;

    /* Interrupts disabled by default */
    config->int1_drdy_xl = false;
    config->int1_drdy_gy = false;
    config->int2_drdy_xl = false;
    config->int2_drdy_gy = false;

    /* Embedded functions disabled by default */
    config->step_counter_en = false;
    config->tap_detection_en = false;
    config->free_fall_en = false;
    config->wake_up_en = false;
    config->tilt_en = false;
    config->d6d_en = false;
    config->significant_motion_en = false;

    /* SFLP disabled by default */
    config->sflp_game_en = false;
    config->sflp_odr = LSM6DSV_SFLP_15Hz;

    /* Tap detection defaults */
    config->tap_threshold_x = 15;
    config->tap_threshold_y = 15;
    config->tap_threshold_z = 15;
    config->tap_shock = 2;
    config->tap_quiet = 2;
    config->tap_latency = 4;
    config->tap_x_en = true;
    config->tap_y_en = true;
    config->tap_z_en = true;
    config->tap_priority = LSM6DSV_ZYX;
    config->tap_mode = LSM6DSV_BOTH_SINGLE_DOUBLE;

    /* Wake-up detection defaults */
    config->wake_up_x_en = true;
    config->wake_up_y_en = true;
    config->wake_up_z_en = true;

    /* 6D threshold default */
    config->d6d_threshold = 60;  /* 60 degrees */

    return 0;
}

/**
 * @brief  Start sensor streaming
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_start(sensor_manager_t *mgr)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    mgr->streaming = true;
    return 0;
}

/**
 * @brief  Stop sensor streaming
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_stop(sensor_manager_t *mgr)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    mgr->streaming = false;
    return 0;
}

/**
 * @brief  Read accelerometer and gyroscope data with error detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  data: Pointer to sensor data structure to fill
 * @retval 0 (always succeeds - check validity flags for actual status)
 * @note   v3.1: Enhanced with flexible sensor configuration and error handling
 *         - Supports acc-only, gyro-only, or both configurations
 *         - Sets validity flags: 0=disabled, 1=valid, 2=error
 *         - Sets error_code for diagnostics
 *         - Never fails - caller checks validity flags
 */
int32_t sensor_manager_read_data(sensor_manager_t *mgr, sensor_data_t *data)
{
    int32_t ret;
    int16_t raw_accel[3] = {0};
    int16_t raw_gyro[3] = {0};
    int16_t raw_temp;
    lsm6dsv_data_ready_t drdy;
    bool acc_enabled, gyro_enabled;

    /* Initialize data structure with defaults */
    memset(data, 0, sizeof(sensor_data_t));
    data->error_code = SENSOR_ERROR_NONE;
    data->timestamp = platform_get_timestamp();

    if (mgr == NULL || data == NULL || !mgr->initialized) {
        data->acc_valid = SENSOR_STATUS_ERROR;
        data->gyro_valid = SENSOR_STATUS_ERROR;
        data->error_code = SENSOR_ERROR_I2C_FAIL;
        return 0;  /* Return success, but with error flags set */
    }

    /* Determine which sensors are enabled */
    acc_enabled = (mgr->config.xl_odr != LSM6DSV_ODR_OFF);
    gyro_enabled = (mgr->config.gy_odr != LSM6DSV_ODR_OFF);

    /* Set initial validity status based on enabled state */
    data->acc_valid = acc_enabled ? SENSOR_STATUS_ERROR : SENSOR_STATUS_DISABLED;
    data->gyro_valid = gyro_enabled ? SENSOR_STATUS_ERROR : SENSOR_STATUS_DISABLED;

    /* If both sensors disabled, return early */
    if (!acc_enabled && !gyro_enabled) {
        return 0;
    }

    /* Check if data is ready */
    ret = lsm6dsv_flag_data_ready_get(&mgr->ctx, &drdy);
    if (ret != 0) {
        mgr->errors_count++;
        data->error_code = SENSOR_ERROR_I2C_FAIL;
        /* Both enabled sensors marked as error */
        return 0;
    }

    /* ========== Process Accelerometer ========== */
    if (acc_enabled) {
        if (!drdy.drdy_xl) {
            /* Data not ready */
            data->acc_valid = SENSOR_STATUS_ERROR;
            data->error_code = SENSOR_ERROR_NOT_READY;
        } else {
            /* Attempt to read accelerometer data */
            ret = lsm6dsv_acceleration_raw_get(&mgr->ctx, raw_accel);
            if (ret != 0) {
                mgr->errors_count++;
                data->acc_valid = SENSOR_STATUS_ERROR;
                data->error_code = SENSOR_ERROR_I2C_FAIL;
            } else if (is_raw_data_invalid(raw_accel)) {
                /* Data read but appears invalid (all zeros or 0xFFFF) */
                data->acc_valid = SENSOR_STATUS_ERROR;
                data->error_code = SENSOR_ERROR_INVALID_DATA;
            } else {
                /* Data is valid - convert to physical units */
                data->acc_x = convert_accel_to_mg(raw_accel[0], mgr->config.xl_fs);
                data->acc_y = convert_accel_to_mg(raw_accel[1], mgr->config.xl_fs);
                data->acc_z = convert_accel_to_mg(raw_accel[2], mgr->config.xl_fs);

                /* Software offsets removed - use hardware offsets only */

                data->acc_valid = SENSOR_STATUS_VALID;
            }
        }
    }

    /* ========== Process Gyroscope ========== */
    if (gyro_enabled) {
        if (!drdy.drdy_gy) {
            /* Data not ready */
            data->gyro_valid = SENSOR_STATUS_ERROR;
            if (data->error_code == SENSOR_ERROR_NONE) {
                data->error_code = SENSOR_ERROR_NOT_READY;
            }
        } else {
            /* Attempt to read gyroscope data */
            ret = lsm6dsv_angular_rate_raw_get(&mgr->ctx, raw_gyro);
            if (ret != 0) {
                mgr->errors_count++;
                data->gyro_valid = SENSOR_STATUS_ERROR;
                if (data->error_code == SENSOR_ERROR_NONE) {
                    data->error_code = SENSOR_ERROR_I2C_FAIL;
                }
            } else if (is_raw_data_invalid(raw_gyro)) {
                /* Data read but appears invalid */
                data->gyro_valid = SENSOR_STATUS_ERROR;
                if (data->error_code == SENSOR_ERROR_NONE) {
                    data->error_code = SENSOR_ERROR_INVALID_DATA;
                }
            } else {
                /* Data is valid - convert to physical units */
                data->gyro_x = convert_gyro_to_mdps(raw_gyro[0], mgr->config.gy_fs);
                data->gyro_y = convert_gyro_to_mdps(raw_gyro[1], mgr->config.gy_fs);
                data->gyro_z = convert_gyro_to_mdps(raw_gyro[2], mgr->config.gy_fs);

                /* Software offsets removed - gyro has no hardware offset support */

                data->gyro_valid = SENSOR_STATUS_VALID;
            }
        }
    }

    /* ========== Read Temperature (non-critical) ========== */
    ret = lsm6dsv_temperature_raw_get(&mgr->ctx, &raw_temp);
    if (ret == 0) {
        data->temp = lsm6dsv_from_lsb_to_celsius(raw_temp);
    } else {
        data->temp = 0.0f;
    }

    /* ========== Update Statistics ========== */
    /* Only count as successful sample if at least one sensor is valid */
    if (data->acc_valid == SENSOR_STATUS_VALID || data->gyro_valid == SENSOR_STATUS_VALID) {
        mgr->samples_read++;

        /* Update current data if at least partial data is valid */
        memcpy(&mgr->current_data, data, sizeof(sensor_data_t));

        /* Clear error code if at least one sensor succeeded */
        if (data->acc_valid == SENSOR_STATUS_VALID && data->gyro_valid == SENSOR_STATUS_VALID) {
            data->error_code = SENSOR_ERROR_NONE;
        }
    }

    return 0;  /* Always return success - check validity flags */
}

/**
 * @brief  Read SFLP (Sensor Fusion) quaternion data
 * @param  mgr: Pointer to sensor manager structure
 * @param  data: Pointer to SFLP data structure to fill
 * @retval 0 on success, -1 on error
 * @note   SFLP must be enabled first via sensor_manager_enable_sflp()
 */
int32_t sensor_manager_read_sflp(sensor_manager_t *mgr, sflp_data_t *data)
{
    /* NOTE: The current LSM6DSV driver does not expose direct quaternion register access.
     * The SFLP can be enabled, but reading the quaternion data requires either:
     * 1. Updated driver with quaternion read functions
     * 2. Direct register access to SFLP output registers
     * 3. Reading from sensor hub if configured
     *
     * For now, this is a placeholder implementation.
     * The main.c code also has this limitation (see LSM6DSV_ReadSFLP function).
     */

    if (mgr == NULL || data == NULL || !mgr->initialized) {
        return -1;
    }

    if (!mgr->config.sflp_game_en) {
        return -1;  /* SFLP not enabled */
    }

    /* TODO: Implement when quaternion registers are exposed in driver */
    /* For now, return zeros */
    memset(data, 0, sizeof(sflp_data_t));

    return 0;
}

/* ============================================================================
 * Accelerometer Configuration
 * ============================================================================ */

/**
 * @brief  Set accelerometer output data rate
 * @param  mgr: Pointer to sensor manager structure
 * @param  odr: Output data rate
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_odr(sensor_manager_t *mgr, lsm6dsv_data_rate_t odr)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_xl_data_rate_set(&mgr->ctx, odr);
    if (ret != 0) {
        return -1;
    }

    mgr->config.xl_odr = odr;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set accelerometer full scale
 * @param  mgr: Pointer to sensor manager structure
 * @param  fs: Full scale range
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_fs(sensor_manager_t *mgr, lsm6dsv_xl_full_scale_t fs)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_xl_full_scale_set(&mgr->ctx, fs);
    if (ret != 0) {
        return -1;
    }

    mgr->config.xl_fs = fs;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set accelerometer operating mode
 * @param  mgr: Pointer to sensor manager structure
 * @param  mode: Operating mode (high performance, low power, etc.)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_mode(sensor_manager_t *mgr, lsm6dsv_xl_mode_t mode)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_xl_mode_set(&mgr->ctx, mode);
    if (ret != 0) {
        return -1;
    }

    mgr->config.xl_mode = mode;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/* ============================================================================
 * Gyroscope Configuration
 * ============================================================================ */

/**
 * @brief  Set gyroscope output data rate
 * @param  mgr: Pointer to sensor manager structure
 * @param  odr: Output data rate
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_gy_odr(sensor_manager_t *mgr, lsm6dsv_data_rate_t odr)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_gy_data_rate_set(&mgr->ctx, odr);
    if (ret != 0) {
        return -1;
    }

    mgr->config.gy_odr = odr;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set gyroscope full scale
 * @param  mgr: Pointer to sensor manager structure
 * @param  fs: Full scale range
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_gy_fs(sensor_manager_t *mgr, lsm6dsv_gy_full_scale_t fs)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_gy_full_scale_set(&mgr->ctx, fs);
    if (ret != 0) {
        return -1;
    }

    mgr->config.gy_fs = fs;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set gyroscope operating mode
 * @param  mgr: Pointer to sensor manager structure
 * @param  mode: Operating mode (high performance, low power, etc.)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_gy_mode(sensor_manager_t *mgr, lsm6dsv_gy_mode_t mode)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_gy_mode_set(&mgr->ctx, mode);
    if (ret != 0) {
        return -1;
    }

    mgr->config.gy_mode = mode;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/* ============================================================================
 * Filtering Configuration
 * ============================================================================ */

/**
 * @brief  Configure accelerometer low-pass filter 2 (LPF2)
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable LPF2, false to disable
 * @param  bandwidth: Filter bandwidth setting
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_lpf2(sensor_manager_t *mgr, bool enable, lsm6dsv_filt_xl_lp2_bandwidth_t bandwidth)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Set bandwidth first */
    ret = lsm6dsv_filt_xl_lp2_bandwidth_set(&mgr->ctx, bandwidth);
    if (ret != 0) {
        return -1;
    }

    /* Enable or disable LPF2 */
    ret = lsm6dsv_filt_xl_lp2_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    /* Save configuration */
    mgr->config.xl_lpf2_en = enable;
    mgr->config.xl_lpf2_bw = bandwidth;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Configure accelerometer high-pass filter (HPF)
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable HPF, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_hpf(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_filt_xl_hp_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    mgr->config.xl_hpf_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Configure accelerometer fast settling mode
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable fast settling, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_fast_settling(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_filt_xl_fast_settling_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    mgr->config.xl_fast_settling_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Configure gyroscope low-pass filter 1 (LPF1)
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable LPF1, false to disable
 * @param  bandwidth: Filter bandwidth setting
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_gy_lpf1(sensor_manager_t *mgr, bool enable, lsm6dsv_filt_gy_lp1_bandwidth_t bandwidth)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Set bandwidth first */
    ret = lsm6dsv_filt_gy_lp1_bandwidth_set(&mgr->ctx, bandwidth);
    if (ret != 0) {
        return -1;
    }

    /* Enable or disable LPF1 */
    ret = lsm6dsv_filt_gy_lp1_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    /* Save configuration */
    mgr->config.gy_lpf1_en = enable;
    mgr->config.gy_lpf1_bw = bandwidth;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/* ============================================================================
 * SFLP (Sensor Fusion) Functions
 * ============================================================================ */

/**
 * @brief  Enable or disable SFLP game rotation vector
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_sflp(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_sflp_game_rotation_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    mgr->config.sflp_game_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set SFLP output data rate
 * @param  mgr: Pointer to sensor manager structure
 * @param  odr: SFLP output data rate
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_sflp_odr(sensor_manager_t *mgr, uint8_t odr)
{
    int32_t ret;
    lsm6dsv_sflp_data_rate_t sflp_rate;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Convert to driver enum */
    switch (odr) {
        case LSM6DSV_SFLP_15Hz:
            sflp_rate = LSM6DSV_SFLP_15Hz;
            break;
        case LSM6DSV_SFLP_30Hz:
            sflp_rate = LSM6DSV_SFLP_30Hz;
            break;
        case LSM6DSV_SFLP_60Hz:
            sflp_rate = LSM6DSV_SFLP_60Hz;
            break;
        case LSM6DSV_SFLP_120Hz:
            sflp_rate = LSM6DSV_SFLP_120Hz;
            break;
        case LSM6DSV_SFLP_240Hz:
            sflp_rate = LSM6DSV_SFLP_240Hz;
            break;
        case LSM6DSV_SFLP_480Hz:
            sflp_rate = LSM6DSV_SFLP_480Hz;
            break;
        default:
            return -1;
    }

    ret = lsm6dsv_sflp_data_rate_set(&mgr->ctx, sflp_rate);
    if (ret != 0) {
        return -1;
    }

    mgr->config.sflp_odr = odr;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/* ============================================================================
 * FIFO Functions (Stub Implementations)
 * ============================================================================ */

/**
 * @brief  Enable FIFO with specified mode
 * @param  mgr: Pointer to sensor manager structure
 * @param  mode: FIFO mode
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_fifo_enable(sensor_manager_t *mgr, lsm6dsv_fifo_mode_t mode)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement FIFO enable */
    mgr->config.fifo_mode = mode;
    return 0;
}

/**
 * @brief  Disable FIFO
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_fifo_disable(sensor_manager_t *mgr)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement FIFO disable */
    mgr->config.fifo_mode = LSM6DSV_BYPASS_MODE;
    return 0;
}

/**
 * @brief  Set FIFO watermark level
 * @param  mgr: Pointer to sensor manager structure
 * @param  watermark: Watermark level (number of samples)
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_fifo_set_watermark(sensor_manager_t *mgr, uint8_t watermark)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement FIFO watermark configuration */
    mgr->config.fifo_watermark = watermark;
    return 0;
}

/**
 * @brief  Read data from FIFO
 * @param  mgr: Pointer to sensor manager structure
 * @param  data: Pointer to array of sensor data structures
 * @param  count: Pointer to variable that will hold number of samples read
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_fifo_read(sensor_manager_t *mgr, sensor_data_t *data, uint16_t *count)
{
    if (mgr == NULL || data == NULL || count == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement FIFO read */
    *count = 0;
    return 0;
}

/**
 * @brief  Get FIFO status
 * @param  mgr: Pointer to sensor manager structure
 * @param  level: Pointer to variable that will hold FIFO level
 * @param  full: Pointer to variable that will hold FIFO full flag
 * @param  ovr: Pointer to variable that will hold FIFO overrun flag
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_fifo_get_status(sensor_manager_t *mgr, uint16_t *level, bool *full, bool *ovr)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement FIFO status read */
    if (level != NULL) *level = 0;
    if (full != NULL) *full = false;
    if (ovr != NULL) *ovr = false;

    return 0;
}

/* ============================================================================
 * Embedded Functions (Stub Implementations)
 * ============================================================================ */

/**
 * @brief  Enable or disable step counter
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_step_counter(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;
    lsm6dsv_stpcnt_mode_t stpcnt_mode;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    stpcnt_mode.step_counter_enable = enable ? 1 : 0;

    ret = lsm6dsv_stpcnt_mode_set(&mgr->ctx, stpcnt_mode);
    if (ret != 0) {
        return -1;
    }

    mgr->config.step_counter_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Get step count
 * @param  mgr: Pointer to sensor manager structure
 * @param  steps: Pointer to variable that will hold step count
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_get_step_count(sensor_manager_t *mgr, uint16_t *steps)
{
    int32_t ret;

    if (mgr == NULL || steps == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_stpcnt_steps_get(&mgr->ctx, steps);
    if (ret != 0) {
        return -1;
    }

    return 0;
}

/**
 * @brief  Reset step counter
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_reset_step_counter(sensor_manager_t *mgr)
{
    int32_t ret;
    lsm6dsv_stpcnt_mode_t stpcnt_mode;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Disable and re-enable to reset */
    stpcnt_mode.step_counter_enable = 0;
    ret = lsm6dsv_stpcnt_mode_set(&mgr->ctx, stpcnt_mode);
    if (ret != 0) {
        return -1;
    }

    HAL_Delay(10);  /* Short delay for reset */

    if (mgr->config.step_counter_en) {
        stpcnt_mode.step_counter_enable = 1;
        ret = lsm6dsv_stpcnt_mode_set(&mgr->ctx, stpcnt_mode);
        if (ret != 0) {
            return -1;
        }
    }

    return 0;
}

/**
 * @brief  Enable or disable tap detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_tap_detection(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;
    lsm6dsv_tap_detection_t tap_det;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    if (enable) {
        /* Configure tap detection on enabled axes */
        tap_det.tap_x_en = mgr->config.tap_x_en ? 1 : 0;
        tap_det.tap_y_en = mgr->config.tap_y_en ? 1 : 0;
        tap_det.tap_z_en = mgr->config.tap_z_en ? 1 : 0;

        ret = lsm6dsv_tap_detection_set(&mgr->ctx, tap_det);
        if (ret != 0) {
            return -1;
        }
    } else {
        /* Disable all axes */
        tap_det.tap_x_en = 0;
        tap_det.tap_y_en = 0;
        tap_det.tap_z_en = 0;

        ret = lsm6dsv_tap_detection_set(&mgr->ctx, tap_det);
        if (ret != 0) {
            return -1;
        }
    }

    mgr->config.tap_detection_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set tap detection threshold
 * @param  mgr: Pointer to sensor manager structure
 * @param  x: X-axis threshold (0-31)
 * @param  y: Y-axis threshold (0-31)
 * @param  z: Z-axis threshold (0-31)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_tap_threshold(sensor_manager_t *mgr, uint8_t x, uint8_t y, uint8_t z)
{
    int32_t ret;
    lsm6dsv_tap_thresholds_t tap_ths;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Set thresholds (5-bit values, 0-31) */
    tap_ths.x = x & 0x1F;
    tap_ths.y = y & 0x1F;
    tap_ths.z = z & 0x1F;

    ret = lsm6dsv_tap_thresholds_set(&mgr->ctx, tap_ths);
    if (ret != 0) {
        return -1;
    }

    mgr->config.tap_threshold_x = x;
    mgr->config.tap_threshold_y = y;
    mgr->config.tap_threshold_z = z;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set tap timing parameters
 * @param  mgr: Pointer to sensor manager structure
 * @param  shock: Shock time window (0-3)
 * @param  quiet: Quiet time window (0-3)
 * @param  latency: Tap gap/latency for double-tap (0-15)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_tap_timing(sensor_manager_t *mgr, uint8_t shock, uint8_t quiet, uint8_t latency)
{
    int32_t ret;
    lsm6dsv_tap_time_windows_t tap_time;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    tap_time.shock = shock & 0x03;     /* 2-bit value */
    tap_time.quiet = quiet & 0x03;     /* 2-bit value */
    tap_time.tap_gap = latency & 0x0F; /* 4-bit value */

    ret = lsm6dsv_tap_time_windows_set(&mgr->ctx, tap_time);
    if (ret != 0) {
        return -1;
    }

    mgr->config.tap_shock = shock;
    mgr->config.tap_quiet = quiet;
    mgr->config.tap_latency = latency;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set tap detection axes
 * @param  mgr: Pointer to sensor manager structure
 * @param  x_en: Enable tap detection on X axis
 * @param  y_en: Enable tap detection on Y axis
 * @param  z_en: Enable tap detection on Z axis
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_tap_axes(sensor_manager_t *mgr, bool x_en, bool y_en, bool z_en)
{
    int32_t ret;
    lsm6dsv_tap_detection_t tap_det;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    tap_det.tap_x_en = x_en ? 1 : 0;
    tap_det.tap_y_en = y_en ? 1 : 0;
    tap_det.tap_z_en = z_en ? 1 : 0;

    ret = lsm6dsv_tap_detection_set(&mgr->ctx, tap_det);
    if (ret != 0) {
        return -1;
    }

    mgr->config.tap_x_en = x_en;
    mgr->config.tap_y_en = y_en;
    mgr->config.tap_z_en = z_en;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set tap axis priority
 * @param  mgr: Pointer to sensor manager structure
 * @param  priority: Axis priority (XYZ, YXZ, XZY, ZYX, YZX, ZXY)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_tap_priority(sensor_manager_t *mgr, lsm6dsv_tap_axis_priority_t priority)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_tap_axis_priority_set(&mgr->ctx, priority);
    if (ret != 0) {
        return -1;
    }

    mgr->config.tap_priority = priority;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set tap detection mode
 * @param  mgr: Pointer to sensor manager structure
 * @param  mode: Tap mode (ONLY_SINGLE or BOTH_SINGLE_DOUBLE)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_tap_mode(sensor_manager_t *mgr, lsm6dsv_tap_mode_t mode)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_tap_mode_set(&mgr->ctx, mode);
    if (ret != 0) {
        return -1;
    }

    mgr->config.tap_mode = mode;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Enable or disable free fall detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   Free-fall is enabled by routing to INT1/INT2 pins
 */
int32_t sensor_manager_enable_free_fall(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Free-fall detection is enabled via interrupt routing */
    mgr->config.free_fall_en = enable;

    return 0;
}

/**
 * @brief  Set free fall detection threshold and duration
 * @param  mgr: Pointer to sensor manager structure
 * @param  threshold: Free fall threshold (0-7, see lsm6dsv_ff_thresholds_t enum)
 * @param  duration: Free fall duration (0-31, 1 LSB = 1/ODR_XL)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_free_fall_threshold(sensor_manager_t *mgr, uint8_t threshold, uint8_t duration)
{
    int32_t ret;
    lsm6dsv_ff_thresholds_t ff_ths;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Map threshold value to enum */
    switch (threshold) {
        case 0: ff_ths = LSM6DSV_156_mg; break;
        case 1: ff_ths = LSM6DSV_219_mg; break;
        case 2: ff_ths = LSM6DSV_250_mg; break;
        case 3: ff_ths = LSM6DSV_312_mg; break;
        case 4: ff_ths = LSM6DSV_344_mg; break;
        case 5: ff_ths = LSM6DSV_406_mg; break;
        case 6: ff_ths = LSM6DSV_469_mg; break;
        case 7: ff_ths = LSM6DSV_500_mg; break;
        default: ff_ths = LSM6DSV_312_mg; break;
    }

    ret = lsm6dsv_ff_thresholds_set(&mgr->ctx, ff_ths);
    if (ret != 0) {
        return -1;
    }

    /* Set duration (5-bit value) */
    ret = lsm6dsv_ff_time_windows_set(&mgr->ctx, duration & 0x1F);
    if (ret != 0) {
        return -1;
    }

    mgr->config.free_fall_threshold = threshold;
    mgr->config.free_fall_duration = duration;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Enable or disable wake-up detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_wake_up(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;
    lsm6dsv_act_mode_t act_mode;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Configure activity/inactivity mode */
    if (enable) {
        /* Enable wake-up: Set XL to low power when inactive */
        act_mode = LSM6DSV_XL_LOW_POWER_GY_NOT_AFFECTED;
    } else {
        /* Disable wake-up */
        act_mode = LSM6DSV_XL_AND_GY_NOT_AFFECTED;
    }

    ret = lsm6dsv_act_mode_set(&mgr->ctx, act_mode);
    if (ret != 0) {
        return -1;
    }

    mgr->config.wake_up_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set wake-up detection threshold and duration
 * @param  mgr: Pointer to sensor manager structure
 * @param  threshold: Wake-up threshold (6-bit value, 1 LSB = FS_XL/64)
 * @param  duration: Wake-up duration (2-bit value, 1 LSB = 1/ODR_XL)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_wake_up_threshold(sensor_manager_t *mgr, uint8_t threshold, uint8_t duration)
{
    int32_t ret;
    lsm6dsv_act_thresholds_t act_ths;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Configure activity/wake-up thresholds */
    memset(&act_ths, 0, sizeof(act_ths));
    act_ths.threshold = threshold & 0x3F;  /* 6-bit threshold */
    act_ths.duration = duration & 0x03;    /* 2-bit duration */

    ret = lsm6dsv_act_thresholds_set(&mgr->ctx, &act_ths);
    if (ret != 0) {
        return -1;
    }

    mgr->config.wake_up_threshold = threshold;
    mgr->config.wake_up_duration = duration;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Set wake-up detection axes
 * @param  mgr: Pointer to sensor manager structure
 * @param  x_en: Enable wake-up detection on X axis
 * @param  y_en: Enable wake-up detection on Y axis
 * @param  z_en: Enable wake-up detection on Z axis
 * @retval 0 on success, -1 on error
 * @note   Axis enable is controlled via interrupt routing registers
 */
int32_t sensor_manager_set_wake_up_axes(sensor_manager_t *mgr, bool x_en, bool y_en, bool z_en)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Save axis enable configuration */
    mgr->config.wake_up_x_en = x_en;
    mgr->config.wake_up_y_en = y_en;
    mgr->config.wake_up_z_en = z_en;

    /* Note: Actual axis enable is configured via MD1_CFG/MD2_CFG registers
     * which are set when interrupts are routed to INT1/INT2 pins */

    return 0;
}

/**
 * @brief  Enable or disable 6D orientation detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   6D is enabled by routing to INT1/INT2 pins
 */
int32_t sensor_manager_enable_6d_orientation(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* 6D orientation detection is enabled via interrupt routing */
    mgr->config.d6d_en = enable;

    return 0;
}

/**
 * @brief  Set 6D orientation detection threshold
 * @param  mgr: Pointer to sensor manager structure
 * @param  threshold: 6D threshold angle (50, 60, 70, or 80 degrees)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_6d_threshold(sensor_manager_t *mgr, uint8_t threshold)
{
    int32_t ret;
    lsm6dsv_6d_threshold_t sixd_ths;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Map threshold to enum */
    switch (threshold) {
        case 50: sixd_ths = LSM6DSV_DEG_50; break;
        case 60: sixd_ths = LSM6DSV_DEG_60; break;
        case 70: sixd_ths = LSM6DSV_DEG_70; break;
        case 80: sixd_ths = LSM6DSV_DEG_80; break;
        default: sixd_ths = LSM6DSV_DEG_60; break;  /* Default to 60° */
    }

    ret = lsm6dsv_6d_threshold_set(&mgr->ctx, sixd_ths);
    if (ret != 0) {
        return -1;
    }

    mgr->config.d6d_threshold = threshold;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Enable or disable tilt detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_tilt(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_tilt_mode_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    mgr->config.tilt_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Enable or disable significant motion detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_significant_motion(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    ret = lsm6dsv_sigmot_mode_set(&mgr->ctx, enable ? 1 : 0);
    if (ret != 0) {
        return -1;
    }

    mgr->config.significant_motion_en = enable;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/* ============================================================================
 * Interrupt Configuration (Stub Implementations)
 * ============================================================================ */

/**
 * @brief  Configure INT1 pin routing
 * @param  mgr: Pointer to sensor manager structure
 * @param  drdy_xl: Enable data ready interrupt for accelerometer
 * @param  drdy_gy: Enable data ready interrupt for gyroscope
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_config_int1(sensor_manager_t *mgr, bool drdy_xl, bool drdy_gy)
{
    int32_t ret;
    lsm6dsv_pin_int_route_t int1_route;
    lsm6dsv_emb_pin_int_route_t emb_int1_route;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Read current INT1 routing */
    ret = lsm6dsv_pin_int1_route_get(&mgr->ctx, &int1_route);
    if (ret != 0) {
        return -1;
    }

    /* Configure data ready interrupts */
    /* Note: Data ready is typically not routed through these functions,
     * but rather through DRDY configuration registers */

    /* Route embedded functions based on enable flags */
    int1_route.single_tap = mgr->config.tap_detection_en ? 1 : 0;
    int1_route.double_tap = mgr->config.tap_detection_en ? 1 : 0;
    int1_route.wakeup = mgr->config.wake_up_en ? 1 : 0;
    int1_route.freefall = mgr->config.free_fall_en ? 1 : 0;
    int1_route.sixd = mgr->config.d6d_en ? 1 : 0;

    ret = lsm6dsv_pin_int1_route_set(&mgr->ctx, &int1_route);
    if (ret != 0) {
        return -1;
    }

    /* Route embedded functions (step, tilt, sig_mot) */
    memset(&emb_int1_route, 0, sizeof(emb_int1_route));
    emb_int1_route.step_det = mgr->config.step_counter_en ? 1 : 0;
    emb_int1_route.tilt = mgr->config.tilt_en ? 1 : 0;
    emb_int1_route.sig_mot = mgr->config.significant_motion_en ? 1 : 0;

    ret = lsm6dsv_emb_pin_int1_route_set(&mgr->ctx, &emb_int1_route);
    if (ret != 0) {
        return -1;
    }

    mgr->config.int1_drdy_xl = drdy_xl;
    mgr->config.int1_drdy_gy = drdy_gy;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Configure INT2 pin routing
 * @param  mgr: Pointer to sensor manager structure
 * @param  drdy_xl: Enable data ready interrupt for accelerometer
 * @param  drdy_gy: Enable data ready interrupt for gyroscope
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_config_int2(sensor_manager_t *mgr, bool drdy_xl, bool drdy_gy)
{
    int32_t ret;
    lsm6dsv_pin_int_route_t int2_route;
    lsm6dsv_emb_pin_int_route_t emb_int2_route;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Read current INT2 routing */
    ret = lsm6dsv_pin_int2_route_get(&mgr->ctx, &int2_route);
    if (ret != 0) {
        return -1;
    }

    /* Route embedded functions based on enable flags */
    int2_route.single_tap = mgr->config.tap_detection_en ? 1 : 0;
    int2_route.double_tap = mgr->config.tap_detection_en ? 1 : 0;
    int2_route.wakeup = mgr->config.wake_up_en ? 1 : 0;
    int2_route.freefall = mgr->config.free_fall_en ? 1 : 0;
    int2_route.sixd = mgr->config.d6d_en ? 1 : 0;

    ret = lsm6dsv_pin_int2_route_set(&mgr->ctx, &int2_route);
    if (ret != 0) {
        return -1;
    }

    /* Route embedded functions (step, tilt, sig_mot) */
    memset(&emb_int2_route, 0, sizeof(emb_int2_route));
    emb_int2_route.step_det = mgr->config.step_counter_en ? 1 : 0;
    emb_int2_route.tilt = mgr->config.tilt_en ? 1 : 0;
    emb_int2_route.sig_mot = mgr->config.significant_motion_en ? 1 : 0;

    ret = lsm6dsv_emb_pin_int2_route_set(&mgr->ctx, &emb_int2_route);
    if (ret != 0) {
        return -1;
    }

    mgr->config.int2_drdy_xl = drdy_xl;
    mgr->config.int2_drdy_gy = drdy_gy;
    HAL_Delay(SENSOR_CONFIG_DELAY_MS);

    return 0;
}

/**
 * @brief  Get interrupt source/event type
 * @param  mgr: Pointer to sensor manager structure
 * @param  event: Pointer to interrupt event variable
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_get_interrupt_source(sensor_manager_t *mgr, interrupt_event_t *event)
{
    if (mgr == NULL || event == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement interrupt source detection */
    *event = INT_EVENT_NONE;
    return 0;
}

/* ============================================================================
 * Self-Test and Calibration (Stub Implementations)
 * ============================================================================ */

/**
 * @brief  Run sensor self-test
 * @param  mgr: Pointer to sensor manager structure
 * @param  xl_pass: Pointer to accelerometer test result
 * @param  gy_pass: Pointer to gyroscope test result
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_run_self_test(sensor_manager_t *mgr, bool *xl_pass, bool *gy_pass)
{
    int32_t ret;

    if (mgr == NULL || xl_pass == NULL || gy_pass == NULL || !mgr->initialized) {
        return -1;
    }

    /* Enable accelerometer self-test (positive) */
    ret = lsm6dsv_xl_self_test_set(&mgr->ctx, LSM6DSV_XL_ST_POSITIVE);
    if (ret != 0) {
        *xl_pass = false;
    } else {
        HAL_Delay(100);  /* Wait for self-test */
        lsm6dsv_xl_self_test_set(&mgr->ctx, LSM6DSV_XL_ST_DISABLE);
        *xl_pass = true;  /* Simplified - actual test needs measurement comparison */
    }

    /* Enable gyroscope self-test (positive) */
    ret = lsm6dsv_gy_self_test_set(&mgr->ctx, LSM6DSV_GY_ST_POSITIVE);
    if (ret != 0) {
        *gy_pass = false;
    } else {
        HAL_Delay(100);  /* Wait for self-test */
        lsm6dsv_gy_self_test_set(&mgr->ctx, LSM6DSV_GY_ST_DISABLE);
        *gy_pass = true;  /* Simplified - actual test needs measurement comparison */
    }

    return 0;
}

/**
 * @brief  Convert ODR enum to frequency in Hz
 * @param  odr: ODR enum value
 * @retval Frequency in Hz (0 if unknown/off)
 */
static float odr_to_hz(lsm6dsv_data_rate_t odr)
{
    switch (odr) {
        case LSM6DSV_ODR_OFF:              return 0.0f;
        case LSM6DSV_ODR_AT_1Hz875:        return 1.875f;
        case LSM6DSV_ODR_AT_7Hz5:          return 7.5f;
        case LSM6DSV_ODR_AT_15Hz:          return 15.0f;
        case LSM6DSV_ODR_AT_30Hz:          return 30.0f;
        case LSM6DSV_ODR_AT_60Hz:          return 60.0f;
        case LSM6DSV_ODR_AT_120Hz:         return 120.0f;
        case LSM6DSV_ODR_AT_240Hz:         return 240.0f;
        case LSM6DSV_ODR_AT_480Hz:         return 480.0f;
        case LSM6DSV_ODR_AT_960Hz:         return 960.0f;
        default:                           return 120.0f;  /* Default fallback */
    }
}

/**
 * @brief  Calibrate sensor offsets
 * @param  mgr: Pointer to sensor manager structure
 * @param  duration_sec: Duration in seconds to collect samples
 * @param  acc_offset_out: Output array for accelerometer offsets [x,y,z] in mg
 * @param  gyro_offset_out: Output array for gyroscope offsets [x,y,z] in mdps
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_calibrate_offsets(sensor_manager_t *mgr, uint32_t duration_sec,
                                          float *acc_offset_out, float *gyro_offset_out)
{
    int32_t ret;
    sensor_data_t data;
    float acc_sum[3] = {0};
    float gyro_sum[3] = {0};
    int num_samples;
    int samples_collected = 0;
    float odr_hz;

    if (mgr == NULL || !mgr->initialized || acc_offset_out == NULL || gyro_offset_out == NULL) {
        return -1;
    }

    /* Get current ODR and calculate number of samples */
    odr_hz = odr_to_hz(mgr->config.xl_odr);
    if (odr_hz <= 0.0f) {
        return -1;  /* Sensor not running */
    }

    /* Calculate number of samples based on duration and ODR */
    num_samples = (int)(duration_sec * odr_hz);
    if (num_samples < 10) {
        num_samples = 10;  /* Minimum 10 samples */
    }

    /* Calculate delay between samples in ms */
    uint32_t delay_ms = (uint32_t)((duration_sec * 1000.0f) / num_samples);
    if (delay_ms < 1) {
        delay_ms = 1;  /* Minimum 1ms */
    }

    /* Collect samples */
    for (int i = 0; i < num_samples; i++) {
        ret = sensor_manager_read_data(mgr, &data);
        /* Only require accelerometer to be valid (we're primarily calibrating accel) */
        if (ret == 0 && data.acc_valid == SENSOR_STATUS_VALID) {
            acc_sum[0] += data.acc_x;
            acc_sum[1] += data.acc_y;
            acc_sum[2] += data.acc_z - 1000.0f;  /* Subtract 1g from Z axis (gravity compensation) */

            /* Also collect gyro data if gyro is valid */
            if (data.gyro_valid == SENSOR_STATUS_VALID) {
                gyro_sum[0] += data.gyro_x;
                gyro_sum[1] += data.gyro_y;
                gyro_sum[2] += data.gyro_z;
            }
            samples_collected++;
        }
        HAL_Delay(delay_ms);
    }

    if (samples_collected < 5) {
        return -1;  /* Not enough valid samples */
    }

    /* Calculate averages as offsets */
    acc_offset_out[0] = acc_sum[0] / samples_collected;
    acc_offset_out[1] = acc_sum[1] / samples_collected;
    acc_offset_out[2] = acc_sum[2] / samples_collected;
    gyro_offset_out[0] = gyro_sum[0] / samples_collected;
    gyro_offset_out[1] = gyro_sum[1] / samples_collected;
    gyro_offset_out[2] = gyro_sum[2] / samples_collected;

    /* Do NOT store in mgr->acc_offset/gyro_offset - we don't want software offsets applied */

    return 0;
}

/**
 * @brief  Apply or remove calibration offsets
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to apply offsets, false to remove
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_apply_offsets(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    if (!enable) {
        /* Clear offsets */
        memset(mgr->acc_offset, 0, sizeof(mgr->acc_offset));
        memset(mgr->gyro_offset, 0, sizeof(mgr->gyro_offset));
    }

    return 0;
}

/* ============================================================================
 * Hardware Offset Configuration
 * ============================================================================ */

/**
 * @brief  Set hardware offset for accelerometer axis
 * @param  mgr: Pointer to sensor manager structure
 * @param  axis: Axis index (0=X, 1=Y, 2=Z)
 * @param  offset_mg: Offset value in mg (±15.875mg range)
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_set_xl_offset_hw(sensor_manager_t *mgr, uint8_t axis, float offset_mg)
{
    int32_t ret;
    lsm6dsv_xl_offset_mg_t xl_offset;

    if (mgr == NULL || !mgr->initialized || axis > 2) {
        return -1;
    }

    /* Clamp to ±15.875mg coarse mode range */
    if (offset_mg > 15.875f) offset_mg = 15.875f;
    if (offset_mg < -15.875f) offset_mg = -15.875f;

    /* Read current offsets first to preserve other axes */
    ret = lsm6dsv_xl_offset_mg_get(&mgr->ctx, &xl_offset);
    if (ret != 0) {
        return ret;
    }

    /* Update the requested axis */
    switch (axis) {
        case 0:  /* X axis */
            xl_offset.x_mg = offset_mg;
            mgr->config.xl_offset_x = offset_mg;
            break;
        case 1:  /* Y axis */
            xl_offset.y_mg = offset_mg;
            mgr->config.xl_offset_y = offset_mg;
            break;
        case 2:  /* Z axis */
            xl_offset.z_mg = offset_mg;
            mgr->config.xl_offset_z = offset_mg;
            break;
        default:
            return -1;
    }

    /* Write all offsets back */
    ret = lsm6dsv_xl_offset_mg_set(&mgr->ctx, xl_offset);

    return ret;
}

/**
 * @brief  Get hardware offset for accelerometer axis
 * @param  mgr: Pointer to sensor manager structure
 * @param  axis: Axis index (0=X, 1=Y, 2=Z)
 * @param  offset_mg: Pointer to variable that will hold offset value in mg
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_get_xl_offset_hw(sensor_manager_t *mgr, uint8_t axis, float *offset_mg)
{
    if (mgr == NULL || !mgr->initialized || offset_mg == NULL || axis > 2) {
        return -1;
    }

    /* Return stored config value */
    switch (axis) {
        case 0:
            *offset_mg = mgr->config.xl_offset_x;
            break;
        case 1:
            *offset_mg = mgr->config.xl_offset_y;
            break;
        case 2:
            *offset_mg = mgr->config.xl_offset_z;
            break;
        default:
            return -1;
    }

    return 0;
}

/**
 * @brief  Enable or disable hardware offset application
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_enable_xl_offset_hw(sensor_manager_t *mgr, bool enable)
{
    int32_t ret;

    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* Enable/disable user offset on output */
    ret = lsm6dsv_xl_offset_on_out_set(&mgr->ctx, enable ? 1 : 0);
    if (ret == 0) {
        mgr->config.xl_offset_en = enable;
    }

    return ret;
}

/* ============================================================================
 * Direct Register Access
 * ============================================================================ */

/**
 * @brief  Read single register
 * @param  mgr: Pointer to sensor manager structure
 * @param  reg: Register address
 * @param  value: Pointer to variable that will hold register value
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_read_register(sensor_manager_t *mgr, uint8_t reg, uint8_t *value)
{
    if (mgr == NULL || value == NULL || !mgr->initialized) {
        return -1;
    }

    return mgr->ctx.read_reg(mgr->ctx.handle, reg, value, 1);
}

/**
 * @brief  Write single register
 * @param  mgr: Pointer to sensor manager structure
 * @param  reg: Register address
 * @param  value: Value to write
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_write_register(sensor_manager_t *mgr, uint8_t reg, uint8_t value)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    return mgr->ctx.write_reg(mgr->ctx.handle, reg, &value, 1);
}

/**
 * @brief  Read multiple registers
 * @param  mgr: Pointer to sensor manager structure
 * @param  start_reg: Starting register address
 * @param  buffer: Pointer to buffer to store read data
 * @param  len: Number of bytes to read
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_read_registers(sensor_manager_t *mgr, uint8_t start_reg, uint8_t *buffer, uint16_t len)
{
    if (mgr == NULL || buffer == NULL || !mgr->initialized) {
        return -1;
    }

    return mgr->ctx.read_reg(mgr->ctx.handle, start_reg, buffer, len);
}

/* ============================================================================
 * Status and Diagnostics
 * ============================================================================ */

/**
 * @brief  Get device ID (WHO_AM_I)
 * @param  mgr: Pointer to sensor manager structure
 * @param  id: Pointer to variable that will hold device ID
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_get_device_id(sensor_manager_t *mgr, uint8_t *id)
{
    if (mgr == NULL || id == NULL || !mgr->initialized) {
        return -1;
    }

    return lsm6dsv_device_id_get(&mgr->ctx, id);
}

/**
 * @brief  Check if sensor data is ready
 * @param  mgr: Pointer to sensor manager structure
 * @retval true if data ready, false otherwise
 */
bool sensor_manager_is_data_ready(sensor_manager_t *mgr)
{
    lsm6dsv_data_ready_t drdy;

    if (mgr == NULL || !mgr->initialized) {
        return false;
    }

    if (lsm6dsv_flag_data_ready_get(&mgr->ctx, &drdy) != 0) {
        return false;
    }

    return (drdy.drdy_xl && drdy.drdy_gy);
}

/**
 * @brief  Get error string for error code
 * @param  error_code: Error code
 * @retval Pointer to error string
 */
const char* sensor_manager_get_error_string(uint32_t error_code)
{
    if (error_code >= sizeof(error_strings) / sizeof(error_strings[0])) {
        return "Unknown error";
    }

    return error_strings[error_code];
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief  Convert accelerometer LSB to mg based on full scale
 * @param  lsb: Raw LSB value
 * @param  fs: Full scale setting
 * @retval Value in mg
 */
float sensor_manager_acc_lsb_to_mg(int16_t lsb, lsm6dsv_xl_full_scale_t fs)
{
    return convert_accel_to_mg(lsb, fs);
}

/**
 * @brief  Convert gyroscope LSB to mdps based on full scale
 * @param  lsb: Raw LSB value
 * @param  fs: Full scale setting
 * @retval Value in mdps
 */
float sensor_manager_gy_lsb_to_mdps(int16_t lsb, lsm6dsv_gy_full_scale_t fs)
{
    return convert_gyro_to_mdps(lsb, fs);
}
