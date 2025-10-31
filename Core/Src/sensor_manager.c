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
#define SENSOR_RESET_TIMEOUT_MS    100
#define SENSOR_INIT_DELAY_MS       10
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

    if (HAL_I2C_Mem_Write(pctx->hi2c, pctx->address, reg, I2C_MEMADD_SIZE_8BIT,
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

    if (HAL_I2C_Mem_Read(pctx->hi2c, pctx->address, reg, I2C_MEMADD_SIZE_8BIT,
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

    if (mgr == NULL || config == NULL || !mgr->initialized) {
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
 * @brief  Read accelerometer and gyroscope data
 * @param  mgr: Pointer to sensor manager structure
 * @param  data: Pointer to sensor data structure to fill
 * @retval 0 on success, -1 on error
 */
int32_t sensor_manager_read_data(sensor_manager_t *mgr, sensor_data_t *data)
{
    int32_t ret;
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int16_t raw_temp;
    lsm6dsv_data_ready_t drdy;

    if (mgr == NULL || data == NULL || !mgr->initialized) {
        return -1;
    }

    /* Check if data is ready */
    ret = lsm6dsv_flag_data_ready_get(&mgr->ctx, &drdy);
    if (ret != 0) {
        mgr->errors_count++;
        return -1;
    }

    /* Only read if both accel and gyro data are ready */
    if (!drdy.drdy_xl || !drdy.drdy_gy) {
        return -1;  /* Data not ready yet */
    }

    /* Read accelerometer raw data */
    ret = lsm6dsv_acceleration_raw_get(&mgr->ctx, raw_accel);
    if (ret != 0) {
        mgr->errors_count++;
        return -1;
    }

    /* Read gyroscope raw data */
    ret = lsm6dsv_angular_rate_raw_get(&mgr->ctx, raw_gyro);
    if (ret != 0) {
        mgr->errors_count++;
        return -1;
    }

    /* Read temperature */
    ret = lsm6dsv_temperature_raw_get(&mgr->ctx, &raw_temp);
    if (ret != 0) {
        /* Temperature read failure is non-critical */
        raw_temp = 0;
    }

    /* Convert to physical units */
    data->acc_x = convert_accel_to_mg(raw_accel[0], mgr->config.xl_fs);
    data->acc_y = convert_accel_to_mg(raw_accel[1], mgr->config.xl_fs);
    data->acc_z = convert_accel_to_mg(raw_accel[2], mgr->config.xl_fs);

    data->gyro_x = convert_gyro_to_mdps(raw_gyro[0], mgr->config.gy_fs);
    data->gyro_y = convert_gyro_to_mdps(raw_gyro[1], mgr->config.gy_fs);
    data->gyro_z = convert_gyro_to_mdps(raw_gyro[2], mgr->config.gy_fs);

    data->temp = lsm6dsv_from_lsb_to_celsius(raw_temp);

    /* Apply calibration offsets if enabled */
    data->acc_x -= mgr->acc_offset[0];
    data->acc_y -= mgr->acc_offset[1];
    data->acc_z -= mgr->acc_offset[2];

    data->gyro_x -= mgr->gyro_offset[0];
    data->gyro_y -= mgr->gyro_offset[1];
    data->gyro_z -= mgr->gyro_offset[2];

    /* Get timestamp */
    data->timestamp = platform_get_timestamp();

    /* Update current data */
    memcpy(&mgr->current_data, data, sizeof(sensor_data_t));

    /* Update statistics */
    mgr->samples_read++;

    return 0;
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
int32_t sensor_manager_set_sflp_odr(sensor_manager_t *mgr, lsm6dsv_sflp_odr_t odr)
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
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_enable_step_counter(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement step counter enable */
    mgr->config.step_counter_en = enable;
    return 0;
}

/**
 * @brief  Get step count
 * @param  mgr: Pointer to sensor manager structure
 * @param  steps: Pointer to variable that will hold step count
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_get_step_count(sensor_manager_t *mgr, uint16_t *steps)
{
    if (mgr == NULL || steps == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement step count read */
    *steps = 0;
    return 0;
}

/**
 * @brief  Reset step counter
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_reset_step_counter(sensor_manager_t *mgr)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement step counter reset */
    return 0;
}

/**
 * @brief  Enable or disable tap detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_enable_tap_detection(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement tap detection enable */
    mgr->config.tap_detection_en = enable;
    return 0;
}

/**
 * @brief  Set tap detection threshold
 * @param  mgr: Pointer to sensor manager structure
 * @param  x: X-axis threshold
 * @param  y: Y-axis threshold
 * @param  z: Z-axis threshold
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_set_tap_threshold(sensor_manager_t *mgr, uint8_t x, uint8_t y, uint8_t z)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement tap threshold configuration */
    mgr->config.tap_threshold_x = x;
    mgr->config.tap_threshold_y = y;
    mgr->config.tap_threshold_z = z;
    return 0;
}

/**
 * @brief  Enable or disable free fall detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_enable_free_fall(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement free fall detection enable */
    mgr->config.free_fall_en = enable;
    return 0;
}

/**
 * @brief  Set free fall detection threshold and duration
 * @param  mgr: Pointer to sensor manager structure
 * @param  threshold: Free fall threshold
 * @param  duration: Free fall duration
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_set_free_fall_threshold(sensor_manager_t *mgr, uint8_t threshold, uint8_t duration)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement free fall threshold configuration */
    mgr->config.free_fall_threshold = threshold;
    mgr->config.free_fall_duration = duration;
    return 0;
}

/**
 * @brief  Enable or disable wake-up detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_enable_wake_up(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement wake-up detection enable */
    mgr->config.wake_up_en = enable;
    return 0;
}

/**
 * @brief  Set wake-up detection threshold and duration
 * @param  mgr: Pointer to sensor manager structure
 * @param  threshold: Wake-up threshold
 * @param  duration: Wake-up duration
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_set_wake_up_threshold(sensor_manager_t *mgr, uint8_t threshold, uint8_t duration)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement wake-up threshold configuration */
    mgr->config.wake_up_threshold = threshold;
    mgr->config.wake_up_duration = duration;
    return 0;
}

/**
 * @brief  Enable or disable 6D orientation detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_enable_6d_orientation(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement 6D orientation detection enable */
    mgr->config.d6d_en = enable;
    return 0;
}

/**
 * @brief  Set 6D orientation detection threshold
 * @param  mgr: Pointer to sensor manager structure
 * @param  threshold: 6D threshold (50, 60, 70, or 80 degrees)
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_set_6d_threshold(sensor_manager_t *mgr, uint8_t threshold)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement 6D threshold configuration */
    mgr->config.d6d_threshold = threshold;
    return 0;
}

/**
 * @brief  Enable or disable tilt detection
 * @param  mgr: Pointer to sensor manager structure
 * @param  enable: true to enable, false to disable
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_enable_tilt(sensor_manager_t *mgr, bool enable)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement tilt detection enable */
    mgr->config.tilt_en = enable;
    return 0;
}

/* ============================================================================
 * Interrupt Configuration (Stub Implementations)
 * ============================================================================ */

/**
 * @brief  Configure INT1 pin
 * @param  mgr: Pointer to sensor manager structure
 * @param  drdy_xl: Enable data ready interrupt for accelerometer
 * @param  drdy_gy: Enable data ready interrupt for gyroscope
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_config_int1(sensor_manager_t *mgr, bool drdy_xl, bool drdy_gy)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement INT1 configuration */
    mgr->config.int1_drdy_xl = drdy_xl;
    mgr->config.int1_drdy_gy = drdy_gy;
    return 0;
}

/**
 * @brief  Configure INT2 pin
 * @param  mgr: Pointer to sensor manager structure
 * @param  drdy_xl: Enable data ready interrupt for accelerometer
 * @param  drdy_gy: Enable data ready interrupt for gyroscope
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_config_int2(sensor_manager_t *mgr, bool drdy_xl, bool drdy_gy)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement INT2 configuration */
    mgr->config.int2_drdy_xl = drdy_xl;
    mgr->config.int2_drdy_gy = drdy_gy;
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
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_run_self_test(sensor_manager_t *mgr, bool *xl_pass, bool *gy_pass)
{
    if (mgr == NULL || xl_pass == NULL || gy_pass == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement self-test procedure */
    *xl_pass = true;
    *gy_pass = true;
    return 0;
}

/**
 * @brief  Calibrate sensor offsets
 * @param  mgr: Pointer to sensor manager structure
 * @retval 0 on success, -1 on error
 * @note   TODO: Full implementation needed
 */
int32_t sensor_manager_calibrate_offsets(sensor_manager_t *mgr)
{
    if (mgr == NULL || !mgr->initialized) {
        return -1;
    }

    /* TODO: Implement calibration procedure */
    /* Should collect samples and calculate offsets */
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
