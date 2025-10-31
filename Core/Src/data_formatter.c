/**
  ******************************************************************************
  * @file    data_formatter.c
  * @brief   Data formatting utilities for sensor output
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "data_formatter.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static format_config_t default_config = {
    .include_timestamp = true,
    .include_acc = true,
    .include_gyro = true,
    .include_temperature = false,
    .include_sflp = false,
    .include_quaternion = false,
    .include_gravity = false,
    .include_units = false,
    .decimal_places = 2
};

/* Private function prototypes -----------------------------------------------*/
static int format_float(char *buffer, float value, uint8_t decimals);

/* Function implementations --------------------------------------------------*/

/**
 * @brief  Initialize data formatter with default configuration
 * @param  config: Pointer to format configuration structure
 * @retval 0 on success, -1 on error
 */
int32_t data_formatter_init(format_config_t *config)
{
    if (config == NULL) {
        return -1;
    }

    memcpy(config, &default_config, sizeof(format_config_t));
    return 0;
}

/**
 * @brief  Format sensor data as CSV string
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @param  data: Sensor data to format
 * @param  config: Format configuration (NULL for default)
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_csv_data(char *buffer, uint16_t size, const sensor_data_t *data, const format_config_t *config)
{
    if (buffer == NULL || data == NULL || size == 0) {
        return -1;
    }

    const format_config_t *cfg = (config != NULL) ? config : &default_config;
    int len = 0;
    char temp[32];

    /* Start with header */
    len = snprintf(buffer, size, "LSM6DSV");

    /* Add timestamp if configured */
    if (cfg->include_timestamp) {
        len += snprintf(buffer + len, size - len, ",%lu", data->timestamp);
    }

    /* Add accelerometer data */
    if (cfg->include_acc) {
        format_float(temp, data->acc_x, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);

        format_float(temp, data->acc_y, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);

        format_float(temp, data->acc_z, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);
    }

    /* Add gyroscope data */
    if (cfg->include_gyro) {
        format_float(temp, data->gyro_x, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);

        format_float(temp, data->gyro_y, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);

        format_float(temp, data->gyro_z, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);
    }

    /* Add temperature if configured */
    if (cfg->include_temperature) {
        format_float(temp, data->temp, cfg->decimal_places);
        len += snprintf(buffer + len, size - len, ",%s", temp);
    }

    /* Add line ending */
    len += snprintf(buffer + len, size - len, "\r\n");

    return len;
}

/**
 * @brief  Format SFLP (sensor fusion) data as CSV string
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @param  sflp: SFLP data to format
 * @param  timestamp: Timestamp in microseconds
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_csv_sflp(char *buffer, uint16_t size, const sflp_data_t *sflp, uint32_t timestamp)
{
    if (buffer == NULL || sflp == NULL || size == 0) {
        return -1;
    }

    char temp[32];
    int len = 0;

    /* Format: LSM6DSV_SFLP,timestamp,qw,qx,qy,qz */
    len = snprintf(buffer, size, "LSM6DSV_SFLP,%lu", timestamp);

    format_float(temp, sflp->quat_w, 4);
    len += snprintf(buffer + len, size - len, ",%s", temp);

    format_float(temp, sflp->quat_x, 4);
    len += snprintf(buffer + len, size - len, ",%s", temp);

    format_float(temp, sflp->quat_y, 4);
    len += snprintf(buffer + len, size - len, ",%s", temp);

    format_float(temp, sflp->quat_z, 4);
    len += snprintf(buffer + len, size - len, ",%s", temp);

    len += snprintf(buffer + len, size - len, "\r\n");

    return len;
}

/**
 * @brief  Format interrupt event as string
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @param  event: Interrupt event type
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_interrupt_event(char *buffer, uint16_t size, interrupt_event_t event)
{
    if (buffer == NULL || size == 0) {
        return -1;
    }

    const char *event_name;

    switch (event) {
        case INT_WAKE_UP:
            event_name = "WAKE_UP";
            break;
        case INT_FREE_FALL:
            event_name = "FREE_FALL";
            break;
        case INT_SINGLE_TAP:
            event_name = "SINGLE_TAP";
            break;
        case INT_DOUBLE_TAP:
            event_name = "DOUBLE_TAP";
            break;
        case INT_6D_ORIENTATION:
            event_name = "6D_ORIENTATION";
            break;
        case INT_TILT:
            event_name = "TILT";
            break;
        case INT_SIGNIFICANT_MOTION:
            event_name = "SIGNIFICANT_MOTION";
            break;
        case INT_STEP_DETECTED:
            event_name = "STEP_DETECTED";
            break;
        case INT_FIFO_FULL:
            event_name = "FIFO_FULL";
            break;
        case INT_FIFO_WATERMARK:
            event_name = "FIFO_WATERMARK";
            break;
        default:
            event_name = "UNKNOWN";
            break;
    }

    return snprintf(buffer, size, "INT:%s\r\n", event_name);
}

/**
 * @brief  Send OK response
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_send_ok(char *buffer, uint16_t size)
{
    if (buffer == NULL || size < 4) {
        return -1;
    }

    return snprintf(buffer, size, "OK\r\n");
}

/**
 * @brief  Send ERROR response with message
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @param  message: Error message (optional, can be NULL)
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_send_error(char *buffer, uint16_t size, const char *message)
{
    if (buffer == NULL || size == 0) {
        return -1;
    }

    if (message != NULL) {
        return snprintf(buffer, size, "ERROR:%s\r\n", message);
    } else {
        return snprintf(buffer, size, "ERROR\r\n");
    }
}

/**
 * @brief  Format sensor status as string
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @param  sensor_mgr: Sensor manager context
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_send_status(char *buffer, uint16_t size, const sensor_manager_t *sensor_mgr)
{
    if (buffer == NULL || sensor_mgr == NULL || size == 0) {
        return -1;
    }

    int len = snprintf(buffer, size, "STATUS:");

    /* Initialization status */
    len += snprintf(buffer + len, size - len, "init=%d,", sensor_mgr->initialized ? 1 : 0);

    /* Streaming status */
    len += snprintf(buffer + len, size - len, "stream=%d,", sensor_mgr->config.streaming_enabled ? 1 : 0);

    /* SFLP status */
    len += snprintf(buffer + len, size - len, "sflp=%d,", sensor_mgr->config.sflp_enabled ? 1 : 0);

    /* Accelerometer ODR and FS */
    len += snprintf(buffer + len, size - len, "acc_odr=%d,", (int)sensor_mgr->config.xl_odr);
    len += snprintf(buffer + len, size - len, "acc_fs=%d,", (int)sensor_mgr->config.xl_fs);

    /* Gyroscope ODR and FS */
    len += snprintf(buffer + len, size - len, "gyro_odr=%d,", (int)sensor_mgr->config.gy_odr);
    len += snprintf(buffer + len, size - len, "gyro_fs=%d", (int)sensor_mgr->config.gy_fs);

    len += snprintf(buffer + len, size - len, "\r\n");

    return len;
}

/**
 * @brief  Format sensor configuration as string
 * @param  buffer: Output buffer for formatted string
 * @param  size: Size of output buffer
 * @param  config: Sensor configuration
 * @retval Number of characters written, or -1 on error
 */
int32_t data_formatter_send_config(char *buffer, uint16_t size, const sensor_config_t *config)
{
    if (buffer == NULL || config == NULL || size == 0) {
        return -1;
    }

    int len = snprintf(buffer, size, "CONFIG:");

    /* Accelerometer configuration */
    len += snprintf(buffer + len, size - len, "ACC_ODR=%d,", (int)config->xl_odr);
    len += snprintf(buffer + len, size - len, "ACC_FS=%d,", (int)config->xl_fs);

    /* Gyroscope configuration */
    len += snprintf(buffer + len, size - len, "GYRO_ODR=%d,", (int)config->gy_odr);
    len += snprintf(buffer + len, size - len, "GYRO_FS=%d,", (int)config->gy_fs);

    /* SFLP configuration */
    len += snprintf(buffer + len, size - len, "SFLP_EN=%d", config->sflp_enabled ? 1 : 0);
    if (config->sflp_enabled) {
        len += snprintf(buffer + len, size - len, ",SFLP_ODR=%d", (int)config->sflp_odr);
    }

    len += snprintf(buffer + len, size - len, "\r\n");

    return len;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Format float with specified decimal places
 * @param  buffer: Output buffer
 * @param  value: Float value to format
 * @param  decimals: Number of decimal places
 * @retval Number of characters written
 */
static int format_float(char *buffer, float value, uint8_t decimals)
{
    int len;

    switch (decimals) {
        case 0:
            len = snprintf(buffer, 32, "%.0f", value);
            break;
        case 1:
            len = snprintf(buffer, 32, "%.1f", value);
            break;
        case 2:
            len = snprintf(buffer, 32, "%.2f", value);
            break;
        case 3:
            len = snprintf(buffer, 32, "%.3f", value);
            break;
        case 4:
            len = snprintf(buffer, 32, "%.4f", value);
            break;
        default:
            len = snprintf(buffer, 32, "%.2f", value);
            break;
    }

    return len;
}
