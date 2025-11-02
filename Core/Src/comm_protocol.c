/**
  ******************************************************************************
  * @file    comm_protocol.c
  * @brief   UART communication protocol implementation for LSM6DSV
  * @details Parses commands from Python GUI and executes them via sensor_manager
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "comm_protocol.h"
#include "sensor_manager.h"
#include "data_formatter.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define CMD_DELIMITER ":"
#define RESPONSE_OK "OK\r\n"
#define RESPONSE_ERROR_PREFIX "ERROR:"
#define VERSION_STRING "LSM6DSV_FW_v1.0.0"

/* Private function prototypes -----------------------------------------------*/
static command_type_t parse_command_type(const char *cmd_str);
static int32_t execute_ping(comm_protocol_t *comm);
static int32_t execute_status(comm_protocol_t *comm);
static int32_t execute_version(comm_protocol_t *comm);
static int32_t execute_reset(comm_protocol_t *comm);
static int32_t execute_enable(comm_protocol_t *comm, const char *param);
static int32_t execute_disable(comm_protocol_t *comm, const char *param);
static int32_t execute_get_config(comm_protocol_t *comm);
static lsm6dsv_data_rate_t parse_odr_value(float odr_hz);
static lsm6dsv_xl_full_scale_t parse_xl_fs_value(int fs_g);
static lsm6dsv_gy_full_scale_t parse_gy_fs_value(int fs_dps);
static int32_t parse_sflp_odr_value(float odr_hz);

/* Public function implementations -------------------------------------------*/

/**
 * @brief  Initialize communication protocol
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  huart: Pointer to UART handle
 * @param  sensor_mgr: Pointer to sensor manager
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_init(comm_protocol_t *comm, UART_HandleTypeDef *huart, sensor_manager_t *sensor_mgr)
{
    if (comm == NULL || huart == NULL || sensor_mgr == NULL) {
        return -1;
    }

    /* Initialize structure */
    memset(comm, 0, sizeof(comm_protocol_t));

    comm->huart = huart;
    comm->sensor_mgr = sensor_mgr;
    comm->streaming_enabled = true;  /* Streaming on by default */
    comm->stream_format = STREAM_FORMAT_CSV;
    comm->stream_interval_ms = 10;  /* 100Hz default */
    comm->include_timestamp = true;
    comm->include_temperature = false;
    comm->include_sflp = false;

    return 0;
}

/**
 * @brief  Deinitialize communication protocol
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_deinit(comm_protocol_t *comm)
{
    if (comm == NULL) {
        return -1;
    }

    memset(comm, 0, sizeof(comm_protocol_t));
    return 0;
}

/**
 * @brief  UART RX callback - called from interrupt handler
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  data: Received byte
 * @retval None
 */
void comm_protocol_rx_callback(comm_protocol_t *comm, uint8_t data)
{
    if (comm == NULL) {
        return;
    }

    /* Handle line endings */
    if (data == '\r' || data == '\n') {
        if (comm->cmd_index > 0) {
            /* Null-terminate the command */
            comm->cmd_buffer[comm->cmd_index] = '\0';
            comm->cmd_ready = true;
            comm->commands_received++;
            comm->cmd_index = 0;  // Reset buffer immediately to prevent concatenation
        }
        return;
    }

    /* Protect against buffer overwrite - reject data if command pending */
    if (comm->cmd_ready) {
        /* Buffer not yet processed - drop incoming data to prevent overwrite */
        comm->errors++;
        return;
    }

    /* Add character to buffer */
    if (comm->cmd_index < (COMM_MAX_CMD_LENGTH - 1)) {
        comm->cmd_buffer[comm->cmd_index++] = data;
    } else {
        /* Buffer overflow - reset */
        comm->cmd_index = 0;
        comm->errors++;
    }
}

/**
 * @brief  TX complete callback (optional, for future use)
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval None
 */
void comm_protocol_tx_complete_callback(comm_protocol_t *comm)
{
    /* Could be used for flow control or DMA transmission tracking */
    (void)comm;
}

/**
 * @brief  Process pending commands (call from main loop)
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval None
 */
void comm_protocol_process(comm_protocol_t *comm)
{
    if (comm == NULL || !comm->cmd_ready) {
        return;
    }

    command_t cmd;

    /* Debug: Echo received command to see exact format */
    char echo[150];
    int echo_len = snprintf(echo, sizeof(echo), "RX_CMD: [%s]\r\n", comm->cmd_buffer);
    HAL_UART_Transmit(comm->huart, (uint8_t*)echo, echo_len, 10);

    /* Parse the command string */
    if (comm_protocol_parse_command(comm, comm->cmd_buffer, &cmd) == 0) {
        /* Execute the command */
        comm_protocol_execute_command(comm, &cmd);
        comm->commands_processed++;
    } else {
        /* Parse error */
        comm_protocol_send_response(comm, RESP_INVALID_CMD, "Parse failed");
        comm->errors++;
    }

    /* Reset for next command */
    comm->cmd_index = 0;
    comm->cmd_ready = false;
}

/**
 * @brief  Parse command string into command structure
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  cmd_str: Command string to parse
 * @param  cmd: Pointer to command_t structure to fill
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_parse_command(comm_protocol_t *comm, const char *cmd_str, command_t *cmd)
{
    if (comm == NULL || cmd_str == NULL || cmd == NULL) {
        return -1;
    }

    /* Clear command structure */
    memset(cmd, 0, sizeof(command_t));

    /* Make a copy to work with (strtok modifies the string) */
    char cmd_copy[COMM_MAX_CMD_LENGTH];
    strncpy(cmd_copy, cmd_str, COMM_MAX_CMD_LENGTH - 1);
    cmd_copy[COMM_MAX_CMD_LENGTH - 1] = '\0';

    /* Parse command and parameters */
    char *token = strtok(cmd_copy, CMD_DELIMITER);
    if (token == NULL) {
        return -1;
    }

    /* Determine command type */
    cmd->type = parse_command_type(token);

    /* Parse parameters */
    while ((token = strtok(NULL, CMD_DELIMITER)) != NULL && cmd->param_count < COMM_MAX_PARAMS) {
        strncpy(cmd->params[cmd->param_count], token, 31);
        cmd->params[cmd->param_count][31] = '\0';
        cmd->param_count++;
    }

    return 0;
}

/**
 * @brief  Execute parsed command
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  cmd: Pointer to command structure
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_execute_command(comm_protocol_t *comm, const command_t *cmd)
{
    if (comm == NULL || cmd == NULL) {
        return -1;
    }

    int32_t result = -1;

    switch (cmd->type) {
        case CMD_PING:
            result = execute_ping(comm);
            break;

        case CMD_STATUS:
            result = execute_status(comm);
            break;

        case CMD_VERSION:
            result = execute_version(comm);
            break;

        case CMD_RESET:
            result = execute_reset(comm);
            break;

        case CMD_SET:
            if (cmd->param_count >= 2) {
                result = comm_protocol_handle_set(comm, cmd->params[0], cmd->params[1]);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "SET needs 2 params");
            }
            break;

        case CMD_GET:
            if (cmd->param_count >= 1) {
                result = comm_protocol_handle_get(comm, cmd->params[0]);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "GET needs param");
            }
            break;

        case CMD_SFLP_ENABLE:
            result = execute_enable(comm, "SFLP");
            break;

        case CMD_SFLP_DISABLE:
            result = execute_disable(comm, "SFLP");
            break;

        case CMD_ENABLE_FUNC:
            if (cmd->param_count >= 1) {
                result = execute_enable(comm, cmd->params[0]);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "ENABLE needs param");
            }
            break;

        case CMD_DISABLE_FUNC:
            if (cmd->param_count >= 1) {
                result = execute_disable(comm, cmd->params[0]);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "DISABLE needs param");
            }
            break;

        case CMD_STREAM_START:
            comm->streaming_enabled = true;
            result = 0;
            comm_protocol_send_response(comm, RESP_OK, NULL);
            break;

        case CMD_STREAM_STOP:
            comm->streaming_enabled = false;
            result = 0;
            comm_protocol_send_response(comm, RESP_OK, NULL);
            break;

        case CMD_GET_STEP_COUNT: {
            uint16_t steps = 0;
            result = sensor_manager_get_step_count(comm->sensor_mgr, &steps);
            if (result == 0) {
                char response[64];
                sprintf(response, "STEP_COUNT:%u\r\n", steps);
                comm_protocol_send_string(comm, response);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to get step count");
            }
            break;
        }

        case CMD_RESET_STEP_COUNT:
            result = sensor_manager_reset_step_counter(comm->sensor_mgr);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to reset step counter");
            }
            break;

        case CMD_REG_READ:
            if (cmd->param_count >= 1) {
                uint8_t reg = (uint8_t)strtol(cmd->params[0], NULL, 16);
                result = comm_protocol_read_register(comm, reg);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "REG_READ needs address");
            }
            break;

        case CMD_REG_WRITE:
            if (cmd->param_count >= 2) {
                uint8_t reg = (uint8_t)strtol(cmd->params[0], NULL, 16);
                uint8_t value = (uint8_t)strtol(cmd->params[1], NULL, 16);
                result = comm_protocol_write_register(comm, reg, value);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "REG_WRITE needs addr and value");
            }
            break;

        case CMD_SELF_TEST: {
            bool xl_pass = false;
            bool gy_pass = false;
            result = sensor_manager_run_self_test(comm->sensor_mgr, &xl_pass, &gy_pass);
            if (result == 0) {
                char response[64];
                sprintf(response, "SELF_TEST:XL=%s,GY=%s\r\n",
                        xl_pass ? "PASS" : "FAIL",
                        gy_pass ? "PASS" : "FAIL");
                comm_protocol_send_string(comm, response);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Self-test failed");
            }
            break;
        }

        case CMD_CALIBRATE: {
            uint32_t duration_sec = 1;  /* Default 1 second */
            float acc_offset[3], gyro_offset[3];
            char response[128];

            /* Parse duration parameter if provided */
            if (cmd->param_count > 0) {
                duration_sec = (uint32_t)atoi(cmd->params[0]);
                if (duration_sec < 1) duration_sec = 1;
                if (duration_sec > 30) duration_sec = 30;  /* Max 30 seconds */
            }

            result = sensor_manager_calibrate_offsets(comm->sensor_mgr, duration_sec, acc_offset, gyro_offset);
            if (result == 0) {
                /* Format response with offset values */
                snprintf(response, sizeof(response), "CALIBRATE:X=%.3f,Y=%.3f,Z=%.3f\r\n",
                         acc_offset[0], acc_offset[1], acc_offset[2]);
                comm_protocol_send_string(comm, response);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Calibration failed");
            }
            break;
        }

        default:
        case CMD_UNKNOWN:
            comm_protocol_send_response(comm, RESP_INVALID_CMD, "Unknown command");
            result = -1;
            break;
    }

    return result;
}

/**
 * @brief  Send response message
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  code: Response code
 * @param  message: Optional message (NULL for default)
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_send_response(comm_protocol_t *comm, response_code_t code, const char *message)
{
    if (comm == NULL) {
        return -1;
    }

    char response[128];
    int len;

    if (code == RESP_OK) {
        len = sprintf(response, RESPONSE_OK);
    } else {
        const char *msg = (message != NULL) ? message : comm_protocol_get_response_string(code);
        len = sprintf(response, "%s%s\r\n", RESPONSE_ERROR_PREFIX, msg);
    }

    if (len > 0) {
        return comm_protocol_send_string(comm, response);
    }

    return -1;
}

/**
 * @brief  Send raw data via UART
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  data: Data buffer to send
 * @param  length: Length of data
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_send_data(comm_protocol_t *comm, const uint8_t *data, uint16_t length)
{
    if (comm == NULL || data == NULL || length == 0) {
        return -1;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(comm->huart, (uint8_t*)data, length, 10);
    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  Send string via UART
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  str: Null-terminated string
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_send_string(comm_protocol_t *comm, const char *str)
{
    if (comm == NULL || str == NULL) {
        return -1;
    }

    uint16_t len = strlen(str);
    return comm_protocol_send_data(comm, (const uint8_t*)str, len);
}

/**
 * @brief  Handle SET commands (SET:PARAM:VALUE)
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  param: Parameter name
 * @param  value: Parameter value (as string)
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_handle_set(comm_protocol_t *comm, const char *param, const char *value)
{
    if (comm == NULL || param == NULL || value == NULL) {
        return -1;
    }

    parameter_type_t param_type = comm_protocol_get_param_type(param);
    int32_t result = -1;

    switch (param_type) {
        case PARAM_ACC_ODR: {
            float odr_hz = atof(value);
            lsm6dsv_data_rate_t odr = parse_odr_value(odr_hz);
            result = sensor_manager_set_xl_odr(comm->sensor_mgr, odr);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set ACC ODR");
            }
            break;
        }

        case PARAM_ACC_FS: {
            int fs_g = atoi(value);
            lsm6dsv_xl_full_scale_t fs = parse_xl_fs_value(fs_g);
            result = sensor_manager_set_xl_fs(comm->sensor_mgr, fs);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set ACC FS");
            }
            break;
        }

        case PARAM_GYRO_ODR: {
            float odr_hz = atof(value);
            lsm6dsv_data_rate_t odr = parse_odr_value(odr_hz);
            result = sensor_manager_set_gy_odr(comm->sensor_mgr, odr);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set GYRO ODR");
            }
            break;
        }

        case PARAM_GYRO_FS: {
            int fs_dps = atoi(value);
            lsm6dsv_gy_full_scale_t fs = parse_gy_fs_value(fs_dps);
            result = sensor_manager_set_gy_fs(comm->sensor_mgr, fs);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set GYRO FS");
            }
            break;
        }

        case PARAM_SFLP_ODR: {
            float odr_hz = atof(value);
            int32_t odr_val = parse_sflp_odr_value(odr_hz);
            if (odr_val >= 0) {
                /* SFLP ODR is set via game rotation vector config */
                /* For now, just acknowledge - full implementation would use sensor_manager API */
                result = 0;
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_INVALID_PARAM, "Invalid SFLP ODR");
            }
            break;
        }

        case PARAM_FIFO_WATERMARK: {
            uint8_t watermark = (uint8_t)atoi(value);
            result = sensor_manager_fifo_set_watermark(comm->sensor_mgr, watermark);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set FIFO watermark");
            }
            break;
        }

        case PARAM_WAKE_THRESHOLD: {
            uint8_t threshold = (uint8_t)atoi(value);
            uint8_t duration = comm->sensor_mgr->config.wake_up_duration;
            result = sensor_manager_set_wake_up_threshold(comm->sensor_mgr, threshold, duration);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set wake threshold");
            }
            break;
        }

        case PARAM_WAKE_DURATION: {
            uint8_t threshold = comm->sensor_mgr->config.wake_up_threshold;
            uint8_t duration = (uint8_t)atoi(value);
            result = sensor_manager_set_wake_up_threshold(comm->sensor_mgr, threshold, duration);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set wake duration");
            }
            break;
        }

        case PARAM_FF_THRESHOLD: {
            uint8_t threshold = (uint8_t)atoi(value);
            uint8_t duration = comm->sensor_mgr->config.free_fall_duration;
            result = sensor_manager_set_free_fall_threshold(comm->sensor_mgr, threshold, duration);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set FF threshold");
            }
            break;
        }

        case PARAM_FF_DURATION: {
            uint8_t threshold = comm->sensor_mgr->config.free_fall_threshold;
            uint8_t duration = (uint8_t)atoi(value);
            result = sensor_manager_set_free_fall_threshold(comm->sensor_mgr, threshold, duration);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set FF duration");
            }
            break;
        }

        case PARAM_6D_THRESHOLD: {
            uint8_t threshold = (uint8_t)atoi(value);
            result = sensor_manager_set_6d_threshold(comm->sensor_mgr, threshold);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set 6D threshold");
            }
            break;
        }

        case PARAM_ACC_MODE: {
            int mode_val = atoi(value);
            lsm6dsv_xl_mode_t mode;
            // Map: 0=HIGH_PERFORMANCE, 1=HIGH_ACCURACY, 3=ODR_TRIGGERED, 4=LOW_POWER_2_AVG, 5=LOW_POWER_4_AVG, 6=LOW_POWER_8_AVG, 7=NORMAL
            switch (mode_val) {
                case 0: mode = LSM6DSV_XL_HIGH_PERFORMANCE_MD; break;
                case 1: mode = LSM6DSV_XL_HIGH_ACCURACY_ODR_MD; break;
                case 3: mode = LSM6DSV_XL_ODR_TRIGGERED_MD; break;
                case 4: mode = LSM6DSV_XL_LOW_POWER_2_AVG_MD; break;
                case 5: mode = LSM6DSV_XL_LOW_POWER_4_AVG_MD; break;
                case 6: mode = LSM6DSV_XL_LOW_POWER_8_AVG_MD; break;
                case 7: mode = LSM6DSV_XL_NORMAL_MD; break;
                default: mode = LSM6DSV_XL_HIGH_PERFORMANCE_MD; break;
            }
            result = sensor_manager_set_xl_mode(comm->sensor_mgr, mode);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set ACC mode");
            }
            break;
        }

        case PARAM_GYRO_MODE: {
            int mode_val = atoi(value);
            lsm6dsv_gy_mode_t mode;
            // Map: 0=HIGH_PERFORMANCE, 1=HIGH_ACCURACY, 4=SLEEP, 5=LOW_POWER
            switch (mode_val) {
                case 0: mode = LSM6DSV_GY_HIGH_PERFORMANCE_MD; break;
                case 1: mode = LSM6DSV_GY_HIGH_ACCURACY_ODR_MD; break;
                case 4: mode = LSM6DSV_GY_SLEEP_MD; break;
                case 5: mode = LSM6DSV_GY_LOW_POWER_MD; break;
                default: mode = LSM6DSV_GY_HIGH_PERFORMANCE_MD; break;
            }
            result = sensor_manager_set_gy_mode(comm->sensor_mgr, mode);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set GYRO mode");
            }
            break;
        }

        case PARAM_XL_LPF2: {
            int enable = atoi(value);
            lsm6dsv_filt_xl_lp2_bandwidth_t bw = comm->sensor_mgr->config.xl_lpf2_bw;
            result = sensor_manager_set_xl_lpf2(comm->sensor_mgr, enable != 0, bw);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set XL LPF2");
            }
            break;
        }

        case PARAM_XL_LPF2_BW: {
            int bw_val = atoi(value);
            lsm6dsv_filt_xl_lp2_bandwidth_t bw;
            // Map 0-7 to bandwidth enum
            bw = (lsm6dsv_filt_xl_lp2_bandwidth_t)(bw_val & 0x07);
            bool enable = comm->sensor_mgr->config.xl_lpf2_en;
            result = sensor_manager_set_xl_lpf2(comm->sensor_mgr, enable, bw);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set XL LPF2 bandwidth");
            }
            break;
        }

        case PARAM_XL_HPF: {
            int enable = atoi(value);
            result = sensor_manager_set_xl_hpf(comm->sensor_mgr, enable != 0);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set XL HPF");
            }
            break;
        }

        case PARAM_XL_FAST_SETTLING: {
            int enable = atoi(value);
            result = sensor_manager_set_xl_fast_settling(comm->sensor_mgr, enable != 0);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set XL fast settling");
            }
            break;
        }

        case PARAM_GY_LPF1: {
            int enable = atoi(value);
            lsm6dsv_filt_gy_lp1_bandwidth_t bw = comm->sensor_mgr->config.gy_lpf1_bw;
            result = sensor_manager_set_gy_lpf1(comm->sensor_mgr, enable != 0, bw);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set GY LPF1");
            }
            break;
        }

        case PARAM_GY_LPF1_BW: {
            int bw_val = atoi(value);
            lsm6dsv_filt_gy_lp1_bandwidth_t bw;
            // Map 0-7 to bandwidth enum
            bw = (lsm6dsv_filt_gy_lp1_bandwidth_t)(bw_val & 0x07);
            bool enable = comm->sensor_mgr->config.gy_lpf1_en;
            result = sensor_manager_set_gy_lpf1(comm->sensor_mgr, enable, bw);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set GY LPF1 bandwidth");
            }
            break;
        }

        case PARAM_TAP_THRESHOLD_X: {
            uint8_t x = (uint8_t)atoi(value);
            uint8_t y = comm->sensor_mgr->config.tap_threshold_y;
            uint8_t z = comm->sensor_mgr->config.tap_threshold_z;
            result = sensor_manager_set_tap_threshold(comm->sensor_mgr, x, y, z);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap threshold X");
            }
            break;
        }

        case PARAM_TAP_THRESHOLD_Y: {
            uint8_t x = comm->sensor_mgr->config.tap_threshold_x;
            uint8_t y = (uint8_t)atoi(value);
            uint8_t z = comm->sensor_mgr->config.tap_threshold_z;
            result = sensor_manager_set_tap_threshold(comm->sensor_mgr, x, y, z);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap threshold Y");
            }
            break;
        }

        case PARAM_TAP_THRESHOLD_Z: {
            uint8_t x = comm->sensor_mgr->config.tap_threshold_x;
            uint8_t y = comm->sensor_mgr->config.tap_threshold_y;
            uint8_t z = (uint8_t)atoi(value);
            result = sensor_manager_set_tap_threshold(comm->sensor_mgr, x, y, z);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap threshold Z");
            }
            break;
        }

        case PARAM_TAP_SHOCK: {
            uint8_t shock = (uint8_t)atoi(value);
            uint8_t quiet = comm->sensor_mgr->config.tap_quiet;
            uint8_t latency = comm->sensor_mgr->config.tap_latency;
            result = sensor_manager_set_tap_timing(comm->sensor_mgr, shock, quiet, latency);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap shock");
            }
            break;
        }

        case PARAM_TAP_QUIET: {
            uint8_t shock = comm->sensor_mgr->config.tap_shock;
            uint8_t quiet = (uint8_t)atoi(value);
            uint8_t latency = comm->sensor_mgr->config.tap_latency;
            result = sensor_manager_set_tap_timing(comm->sensor_mgr, shock, quiet, latency);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap quiet");
            }
            break;
        }

        case PARAM_TAP_LATENCY: {
            uint8_t shock = comm->sensor_mgr->config.tap_shock;
            uint8_t quiet = comm->sensor_mgr->config.tap_quiet;
            uint8_t latency = (uint8_t)atoi(value);
            result = sensor_manager_set_tap_timing(comm->sensor_mgr, shock, quiet, latency);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap latency");
            }
            break;
        }

        case PARAM_TAP_AXES: {
            // Value format: "111" for XYZ enabled, "100" for X only, etc.
            bool x_en = (value[0] == '1');
            bool y_en = (value[1] == '1');
            bool z_en = (value[2] == '1');
            result = sensor_manager_set_tap_axes(comm->sensor_mgr, x_en, y_en, z_en);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap axes");
            }
            break;
        }

        case PARAM_TAP_PRIORITY: {
            int priority_val = atoi(value);
            lsm6dsv_tap_axis_priority_t priority;
            // Map: 0=XYZ, 1=YXZ, 2=XZY, 3=ZYX, 4=YZX, 5=ZXY
            switch (priority_val) {
                case 0: priority = LSM6DSV_XYZ; break;
                case 1: priority = LSM6DSV_YXZ; break;
                case 2: priority = LSM6DSV_XZY; break;
                case 3: priority = LSM6DSV_ZYX; break;
                case 4: priority = LSM6DSV_YZX; break;
                case 5: priority = LSM6DSV_ZXY; break;
                default: priority = LSM6DSV_ZYX; break;
            }
            result = sensor_manager_set_tap_priority(comm->sensor_mgr, priority);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap priority");
            }
            break;
        }

        case PARAM_TAP_MODE: {
            int mode_val = atoi(value);
            lsm6dsv_tap_mode_t mode;
            // Map: 0=ONLY_SINGLE, 1=BOTH_SINGLE_DOUBLE
            mode = (mode_val == 0) ? LSM6DSV_ONLY_SINGLE : LSM6DSV_BOTH_SINGLE_DOUBLE;
            result = sensor_manager_set_tap_mode(comm->sensor_mgr, mode);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set tap mode");
            }
            break;
        }

        case PARAM_WAKE_AXES: {
            // Value format: "111" for XYZ enabled, "100" for X only, etc.
            bool x_en = (value[0] == '1');
            bool y_en = (value[1] == '1');
            bool z_en = (value[2] == '1');
            result = sensor_manager_set_wake_up_axes(comm->sensor_mgr, x_en, y_en, z_en);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set wake axes");
            }
            break;
        }

        case PARAM_XL_OFFSET_X: {
            float offset_mg = atof(value);
            result = sensor_manager_set_xl_offset_hw(comm->sensor_mgr, 0, offset_mg);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set X offset");
            }
            break;
        }

        case PARAM_XL_OFFSET_Y: {
            float offset_mg = atof(value);
            result = sensor_manager_set_xl_offset_hw(comm->sensor_mgr, 1, offset_mg);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set Y offset");
            }
            break;
        }

        case PARAM_XL_OFFSET_Z: {
            float offset_mg = atof(value);
            result = sensor_manager_set_xl_offset_hw(comm->sensor_mgr, 2, offset_mg);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set Z offset");
            }
            break;
        }

        case PARAM_XL_OFFSET_ENABLE: {
            bool enable = (atoi(value) != 0);
            result = sensor_manager_enable_xl_offset_hw(comm->sensor_mgr, enable);
            if (result == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to set offset enable");
            }
            break;
        }

        default:
        case PARAM_UNKNOWN:
            comm_protocol_send_response(comm, RESP_INVALID_PARAM, "Unknown parameter");
            result = -1;
            break;
    }

    return result;
}

/**
 * @brief  Handle GET commands (GET:PARAM)
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  param: Parameter name
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_handle_get(comm_protocol_t *comm, const char *param)
{
    if (comm == NULL || param == NULL) {
        return -1;
    }

    /* Check for CONFIG request */
    if (strcmp(param, "CONFIG") == 0) {
        return execute_get_config(comm);
    }

    /* For other parameters, send current value */
    char response[128];
    parameter_type_t param_type = comm_protocol_get_param_type(param);

    switch (param_type) {
        case PARAM_ACC_ODR:
            sprintf(response, "ACC_ODR:%d\r\n", (int)comm->sensor_mgr->config.xl_odr);
            break;

        case PARAM_ACC_FS:
            sprintf(response, "ACC_FS:%d\r\n", (int)comm->sensor_mgr->config.xl_fs);
            break;

        case PARAM_GYRO_ODR:
            sprintf(response, "GYRO_ODR:%d\r\n", (int)comm->sensor_mgr->config.gy_odr);
            break;

        case PARAM_GYRO_FS:
            sprintf(response, "GYRO_FS:%d\r\n", (int)comm->sensor_mgr->config.gy_fs);
            break;

        case PARAM_SFLP_ODR:
            sprintf(response, "SFLP_ODR:%d\r\n", (int)comm->sensor_mgr->config.sflp_odr);
            break;

        default:
            comm_protocol_send_response(comm, RESP_INVALID_PARAM, "Unknown parameter");
            return -1;
    }

    comm_protocol_send_string(comm, response);
    return 0;
}

/**
 * @brief  Read register and send value
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  reg: Register address
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_read_register(comm_protocol_t *comm, uint8_t reg)
{
    if (comm == NULL) {
        return -1;
    }

    uint8_t value = 0;
    int32_t result = sensor_manager_read_register(comm->sensor_mgr, reg, &value);

    if (result == 0) {
        char response[32];
        sprintf(response, "REG:0x%02X=0x%02X\r\n", reg, value);
        comm_protocol_send_string(comm, response);
    } else {
        comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Register read failed");
    }

    return result;
}

/**
 * @brief  Write register
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  reg: Register address
 * @param  value: Value to write
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_write_register(comm_protocol_t *comm, uint8_t reg, uint8_t value)
{
    if (comm == NULL) {
        return -1;
    }

    int32_t result = sensor_manager_write_register(comm->sensor_mgr, reg, value);

    if (result == 0) {
        comm_protocol_send_response(comm, RESP_OK, NULL);
    } else {
        comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Register write failed");
    }

    return result;
}

/**
 * @brief  Dump all registers (placeholder)
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_dump_registers(comm_protocol_t *comm)
{
    if (comm == NULL) {
        return -1;
    }

    /* Not implemented - would iterate through all registers */
    comm_protocol_send_response(comm, RESP_ERROR, "Register dump not implemented");
    return -1;
}

/**
 * @brief  Stream update function (call periodically from main loop)
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval None
 */
void comm_protocol_stream_update(comm_protocol_t *comm)
{
    /* This function is for future use if streaming is controlled by comm_protocol */
    /* Currently, streaming happens directly in main.c */
    (void)comm;
}

/**
 * @brief  Send stream data packet
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  data: Sensor data
 * @param  sflp: SFLP data (can be NULL)
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_send_stream_data(comm_protocol_t *comm, const sensor_data_t *data, const sflp_data_t *sflp)
{
    if (comm == NULL || data == NULL) {
        return -1;
    }

    if (!comm->streaming_enabled) {
        return 0;  /* Streaming disabled, not an error */
    }

    /* Use data_formatter to format and send */
    int32_t result = data_formatter_stream(comm, data, sflp);

    if (result == 0) {
        comm->stream_packets_sent++;
    }

    return result;
}

/**
 * @brief  Start streaming
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  format: Stream format
 * @param  interval_ms: Streaming interval in milliseconds
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_start_streaming(comm_protocol_t *comm, stream_format_t format, uint32_t interval_ms)
{
    if (comm == NULL) {
        return -1;
    }

    comm->streaming_enabled = true;
    comm->stream_format = format;
    comm->stream_interval_ms = interval_ms;

    return 0;
}

/**
 * @brief  Stop streaming
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success, negative: Error
 */
int32_t comm_protocol_stop_streaming(comm_protocol_t *comm)
{
    if (comm == NULL) {
        return -1;
    }

    comm->streaming_enabled = false;
    return 0;
}

/* Utility functions ---------------------------------------------------------*/

/**
 * @brief  Get parameter type from string
 * @param  param: Parameter string
 * @retval parameter_type_t enum value
 */
parameter_type_t comm_protocol_get_param_type(const char *param)
{
    if (param == NULL) {
        return PARAM_UNKNOWN;
    }

    if (strcmp(param, "ACC_ODR") == 0) return PARAM_ACC_ODR;
    if (strcmp(param, "ACC_FS") == 0) return PARAM_ACC_FS;
    if (strcmp(param, "ACC_MODE") == 0) return PARAM_ACC_MODE;
    if (strcmp(param, "GYRO_ODR") == 0) return PARAM_GYRO_ODR;
    if (strcmp(param, "GYRO_FS") == 0) return PARAM_GYRO_FS;
    if (strcmp(param, "GYRO_MODE") == 0) return PARAM_GYRO_MODE;
    if (strcmp(param, "FIFO_MODE") == 0) return PARAM_FIFO_MODE;
    if (strcmp(param, "FIFO_WATERMARK") == 0) return PARAM_FIFO_WATERMARK;
    if (strcmp(param, "SFLP_ODR") == 0) return PARAM_SFLP_ODR;
    if (strcmp(param, "WAKE_THRESHOLD") == 0) return PARAM_WAKE_THRESHOLD;
    if (strcmp(param, "WAKE_DURATION") == 0) return PARAM_WAKE_DURATION;
    if (strcmp(param, "WAKE_AXES") == 0) return PARAM_WAKE_AXES;
    if (strcmp(param, "FF_THRESHOLD") == 0) return PARAM_FF_THRESHOLD;
    if (strcmp(param, "FF_DURATION") == 0) return PARAM_FF_DURATION;
    if (strcmp(param, "TAP_THRESHOLD_X") == 0) return PARAM_TAP_THRESHOLD_X;
    if (strcmp(param, "TAP_THRESHOLD_Y") == 0) return PARAM_TAP_THRESHOLD_Y;
    if (strcmp(param, "TAP_THRESHOLD_Z") == 0) return PARAM_TAP_THRESHOLD_Z;
    if (strcmp(param, "TAP_SHOCK") == 0) return PARAM_TAP_SHOCK;
    if (strcmp(param, "TAP_QUIET") == 0) return PARAM_TAP_QUIET;
    if (strcmp(param, "TAP_LATENCY") == 0) return PARAM_TAP_LATENCY;
    if (strcmp(param, "TAP_AXES") == 0) return PARAM_TAP_AXES;
    if (strcmp(param, "TAP_PRIORITY") == 0) return PARAM_TAP_PRIORITY;
    if (strcmp(param, "TAP_MODE") == 0) return PARAM_TAP_MODE;
    if (strcmp(param, "6D_THRESHOLD") == 0) return PARAM_6D_THRESHOLD;
    if (strcmp(param, "XL_LPF2") == 0) return PARAM_XL_LPF2;
    if (strcmp(param, "XL_LPF2_BW") == 0) return PARAM_XL_LPF2_BW;
    if (strcmp(param, "XL_HPF") == 0) return PARAM_XL_HPF;
    if (strcmp(param, "XL_FAST_SETTLING") == 0) return PARAM_XL_FAST_SETTLING;
    if (strcmp(param, "GY_LPF1") == 0) return PARAM_GY_LPF1;
    if (strcmp(param, "GY_LPF1_BW") == 0) return PARAM_GY_LPF1_BW;
    if (strcmp(param, "XL_OFFSET_X") == 0) return PARAM_XL_OFFSET_X;
    if (strcmp(param, "XL_OFFSET_Y") == 0) return PARAM_XL_OFFSET_Y;
    if (strcmp(param, "XL_OFFSET_Z") == 0) return PARAM_XL_OFFSET_Z;
    if (strcmp(param, "XL_OFFSET_ENABLE") == 0) return PARAM_XL_OFFSET_ENABLE;

    return PARAM_UNKNOWN;
}

/**
 * @brief  Get function type from string
 * @param  func: Function string
 * @retval function_type_t enum value
 */
function_type_t comm_protocol_get_function_type(const char *func)
{
    if (func == NULL) {
        return FUNC_UNKNOWN;
    }

    if (strcmp(func, "STEP_COUNTER") == 0) return FUNC_STEP_COUNTER;
    if (strcmp(func, "STEP") == 0) return FUNC_STEP_COUNTER;
    if (strcmp(func, "TAP") == 0) return FUNC_TAP_DETECTION;
    if (strcmp(func, "TAP_DETECTION") == 0) return FUNC_TAP_DETECTION;
    if (strcmp(func, "FREE_FALL") == 0) return FUNC_FREE_FALL;
    if (strcmp(func, "FF") == 0) return FUNC_FREE_FALL;
    if (strcmp(func, "WAKE_UP") == 0) return FUNC_WAKE_UP;
    if (strcmp(func, "WAKE") == 0) return FUNC_WAKE_UP;
    if (strcmp(func, "TILT") == 0) return FUNC_TILT;
    if (strcmp(func, "6D") == 0) return FUNC_6D_ORIENTATION;
    if (strcmp(func, "6D_ORIENTATION") == 0) return FUNC_6D_ORIENTATION;
    if (strcmp(func, "SIGNIFICANT_MOTION") == 0) return FUNC_SIGNIFICANT_MOTION;
    if (strcmp(func, "SIG_MOTION") == 0) return FUNC_SIGNIFICANT_MOTION;

    return FUNC_UNKNOWN;
}

/**
 * @brief  Get response string for response code
 * @param  code: Response code
 * @retval String description
 */
const char* comm_protocol_get_response_string(response_code_t code)
{
    switch (code) {
        case RESP_OK:
            return "OK";
        case RESP_ERROR:
            return "Error";
        case RESP_INVALID_CMD:
            return "Invalid command";
        case RESP_INVALID_PARAM:
            return "Invalid parameter";
        case RESP_SENSOR_ERROR:
            return "Sensor error";
        case RESP_BUSY:
            return "Busy";
        case RESP_NOT_INITIALIZED:
            return "Not initialized";
        case RESP_TIMEOUT:
            return "Timeout";
        default:
            return "Unknown error";
    }
}

/**
 * @brief  Check if string is valid hexadecimal
 * @param  str: String to check
 * @retval true if valid hex, false otherwise
 */
bool comm_protocol_is_valid_hex(const char *str)
{
    if (str == NULL || *str == '\0') {
        return false;
    }

    /* Skip 0x prefix if present */
    if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
        str += 2;
    }

    while (*str) {
        if (!((*str >= '0' && *str <= '9') ||
              (*str >= 'a' && *str <= 'f') ||
              (*str >= 'A' && *str <= 'F'))) {
            return false;
        }
        str++;
    }

    return true;
}

/**
 * @brief  Convert hex string to byte
 * @param  hex: Hex string (2 characters)
 * @retval Byte value
 */
uint8_t comm_protocol_hex_to_byte(const char *hex)
{
    if (hex == NULL) {
        return 0;
    }

    return (uint8_t)strtol(hex, NULL, 16);
}

/* Private helper functions --------------------------------------------------*/

/**
 * @brief  Parse command type from string
 * @param  cmd_str: Command string
 * @retval command_type_t enum value
 */
static command_type_t parse_command_type(const char *cmd_str)
{
    if (cmd_str == NULL) {
        return CMD_UNKNOWN;
    }

    /* System commands */
    if (strcmp(cmd_str, "PING") == 0) return CMD_PING;
    if (strcmp(cmd_str, "RESET") == 0) return CMD_RESET;
    if (strcmp(cmd_str, "STATUS") == 0) return CMD_STATUS;
    if (strcmp(cmd_str, "VERSION") == 0) return CMD_VERSION;

    /* Configuration commands */
    if (strcmp(cmd_str, "SET") == 0) return CMD_SET;
    if (strcmp(cmd_str, "GET") == 0) return CMD_GET;
    if (strcmp(cmd_str, "CONFIG_SAVE") == 0) return CMD_CONFIG_SAVE;
    if (strcmp(cmd_str, "CONFIG_LOAD") == 0) return CMD_CONFIG_LOAD;

    /* Data streaming */
    if (strcmp(cmd_str, "STREAM_START") == 0) return CMD_STREAM_START;
    if (strcmp(cmd_str, "STREAM_STOP") == 0) return CMD_STREAM_STOP;
    if (strcmp(cmd_str, "STREAM_CONFIG") == 0) return CMD_STREAM_CONFIG;

    /* Register access */
    if (strcmp(cmd_str, "REG_READ") == 0) return CMD_REG_READ;
    if (strcmp(cmd_str, "REG_WRITE") == 0) return CMD_REG_WRITE;
    if (strcmp(cmd_str, "REG_DUMP") == 0) return CMD_REG_DUMP;

    /* Calibration */
    if (strcmp(cmd_str, "CALIBRATE") == 0) return CMD_CALIBRATE;
    if (strcmp(cmd_str, "SELF_TEST") == 0) return CMD_SELF_TEST;

    /* Embedded functions */
    if (strcmp(cmd_str, "ENABLE") == 0) return CMD_ENABLE_FUNC;
    if (strcmp(cmd_str, "DISABLE") == 0) return CMD_DISABLE_FUNC;
    if (strcmp(cmd_str, "GET_STEP_COUNT") == 0) return CMD_GET_STEP_COUNT;
    if (strcmp(cmd_str, "RESET_STEP_COUNT") == 0) return CMD_RESET_STEP_COUNT;

    /* FIFO */
    if (strcmp(cmd_str, "FIFO_READ") == 0) return CMD_FIFO_READ;
    if (strcmp(cmd_str, "FIFO_STATUS") == 0) return CMD_FIFO_STATUS;
    if (strcmp(cmd_str, "FIFO_CONFIG") == 0) return CMD_FIFO_CONFIG;

    /* SFLP - special handling for Python GUI compatibility */
    if (strcmp(cmd_str, "ENABLE") == 0) return CMD_SFLP_ENABLE;  /* Will be handled by param */
    if (strcmp(cmd_str, "DISABLE") == 0) return CMD_SFLP_DISABLE;  /* Will be handled by param */
    if (strcmp(cmd_str, "SFLP_ENABLE") == 0) return CMD_SFLP_ENABLE;
    if (strcmp(cmd_str, "SFLP_DISABLE") == 0) return CMD_SFLP_DISABLE;
    if (strcmp(cmd_str, "SFLP_CONFIG") == 0) return CMD_SFLP_CONFIG;

    return CMD_UNKNOWN;
}

/**
 * @brief  Execute PING command
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success
 */
static int32_t execute_ping(comm_protocol_t *comm)
{
    comm_protocol_send_response(comm, RESP_OK, NULL);
    return 0;
}

/**
 * @brief  Execute STATUS command
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success, negative: Error
 */
static int32_t execute_status(comm_protocol_t *comm)
{
    char status[256];

    sprintf(status, "STATUS:initialized=%d,streaming=%d,errors=%lu,cmd_rx=%lu,cmd_proc=%lu\r\n",
            comm->sensor_mgr->initialized ? 1 : 0,
            comm->streaming_enabled ? 1 : 0,
            comm->errors,
            comm->commands_received,
            comm->commands_processed);

    comm_protocol_send_string(comm, status);
    return 0;
}

/**
 * @brief  Execute VERSION command
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success
 */
static int32_t execute_version(comm_protocol_t *comm)
{
    char version[64];
    sprintf(version, "VERSION:%s\r\n", VERSION_STRING);
    comm_protocol_send_string(comm, version);
    return 0;
}

/**
 * @brief  Execute RESET command
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success, negative: Error
 */
static int32_t execute_reset(comm_protocol_t *comm)
{
    int32_t result = sensor_manager_reset(comm->sensor_mgr);

    if (result == 0) {
        comm_protocol_send_response(comm, RESP_OK, NULL);
    } else {
        comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Reset failed");
    }

    return result;
}

/**
 * @brief  Execute ENABLE command
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  param: Feature to enable
 * @retval 0: Success, negative: Error
 */
static int32_t execute_enable(comm_protocol_t *comm, const char *param)
{
    if (strcmp(param, "SFLP") == 0) {
        int32_t result = sensor_manager_enable_sflp(comm->sensor_mgr, true);
        if (result == 0) {
            comm->include_sflp = true;
            comm_protocol_send_response(comm, RESP_OK, NULL);
        } else {
            comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to enable SFLP");
        }
        return result;
    }

    /* Handle accelerometer enable */
    if (strcmp(param, "ACCELEROMETER") == 0) {
        /* Restore accelerometer to configured ODR (or 120Hz if currently OFF) */
        lsm6dsv_data_rate_t odr = comm->sensor_mgr->config.xl_odr;
        if (odr == LSM6DSV_ODR_OFF) {
            odr = LSM6DSV_ODR_AT_120Hz;  /* Default to 120Hz */
        }
        int32_t result = sensor_manager_set_xl_odr(comm->sensor_mgr, odr);
        if (result == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
        } else {
            comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to enable accelerometer");
        }
        return result;
    }

    /* Handle gyroscope enable */
    if (strcmp(param, "GYROSCOPE") == 0) {
        /* Restore gyroscope to configured ODR (or 120Hz if currently OFF) */
        lsm6dsv_data_rate_t odr = comm->sensor_mgr->config.gy_odr;
        if (odr == LSM6DSV_ODR_OFF) {
            odr = LSM6DSV_ODR_AT_120Hz;  /* Default to 120Hz */
        }
        int32_t result = sensor_manager_set_gy_odr(comm->sensor_mgr, odr);
        if (result == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
        } else {
            comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to enable gyroscope");
        }
        return result;
    }

    /* Handle other embedded functions */
    function_type_t func = comm_protocol_get_function_type(param);
    int32_t result = -1;

    switch (func) {
        case FUNC_STEP_COUNTER:
            result = sensor_manager_enable_step_counter(comm->sensor_mgr, true);
            break;
        case FUNC_TAP_DETECTION:
            result = sensor_manager_enable_tap_detection(comm->sensor_mgr, true);
            break;
        case FUNC_FREE_FALL:
            result = sensor_manager_enable_free_fall(comm->sensor_mgr, true);
            break;
        case FUNC_WAKE_UP:
            result = sensor_manager_enable_wake_up(comm->sensor_mgr, true);
            break;
        case FUNC_TILT:
            result = sensor_manager_enable_tilt(comm->sensor_mgr, true);
            break;
        case FUNC_6D_ORIENTATION:
            result = sensor_manager_enable_6d_orientation(comm->sensor_mgr, true);
            break;
        case FUNC_SIGNIFICANT_MOTION:
            result = sensor_manager_enable_significant_motion(comm->sensor_mgr, true);
            break;
        default:
            comm_protocol_send_response(comm, RESP_INVALID_PARAM, "Unknown function");
            return -1;
    }

    if (result == 0) {
        comm_protocol_send_response(comm, RESP_OK, NULL);
    } else {
        comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to enable function");
    }

    return result;
}

/**
 * @brief  Execute DISABLE command
 * @param  comm: Pointer to comm_protocol_t structure
 * @param  param: Feature to disable
 * @retval 0: Success, negative: Error
 */
static int32_t execute_disable(comm_protocol_t *comm, const char *param)
{
    if (strcmp(param, "SFLP") == 0) {
        int32_t result = sensor_manager_enable_sflp(comm->sensor_mgr, false);
        if (result == 0) {
            comm->include_sflp = false;
            comm_protocol_send_response(comm, RESP_OK, NULL);
        } else {
            comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to disable SFLP");
        }
        return result;
    }

    /* Handle accelerometer disable */
    if (strcmp(param, "ACCELEROMETER") == 0) {
        /* Power down accelerometer by setting ODR to OFF */
        int32_t result = sensor_manager_set_xl_odr(comm->sensor_mgr, LSM6DSV_ODR_OFF);
        if (result == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
        } else {
            comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to disable accelerometer");
        }
        return result;
    }

    /* Handle gyroscope disable */
    if (strcmp(param, "GYROSCOPE") == 0) {
        /* Power down gyroscope by setting ODR to OFF */
        int32_t result = sensor_manager_set_gy_odr(comm->sensor_mgr, LSM6DSV_ODR_OFF);
        if (result == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
        } else {
            comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to disable gyroscope");
        }
        return result;
    }

    /* Handle other embedded functions */
    function_type_t func = comm_protocol_get_function_type(param);
    int32_t result = -1;

    switch (func) {
        case FUNC_STEP_COUNTER:
            result = sensor_manager_enable_step_counter(comm->sensor_mgr, false);
            break;
        case FUNC_TAP_DETECTION:
            result = sensor_manager_enable_tap_detection(comm->sensor_mgr, false);
            break;
        case FUNC_FREE_FALL:
            result = sensor_manager_enable_free_fall(comm->sensor_mgr, false);
            break;
        case FUNC_WAKE_UP:
            result = sensor_manager_enable_wake_up(comm->sensor_mgr, false);
            break;
        case FUNC_TILT:
            result = sensor_manager_enable_tilt(comm->sensor_mgr, false);
            break;
        case FUNC_6D_ORIENTATION:
            result = sensor_manager_enable_6d_orientation(comm->sensor_mgr, false);
            break;
        case FUNC_SIGNIFICANT_MOTION:
            result = sensor_manager_enable_significant_motion(comm->sensor_mgr, false);
            break;
        default:
            comm_protocol_send_response(comm, RESP_INVALID_PARAM, "Unknown function");
            return -1;
    }

    if (result == 0) {
        comm_protocol_send_response(comm, RESP_OK, NULL);
    } else {
        comm_protocol_send_response(comm, RESP_SENSOR_ERROR, "Failed to disable function");
    }

    return result;
}

/**
 * @brief  Execute GET:CONFIG command
 * @param  comm: Pointer to comm_protocol_t structure
 * @retval 0: Success, negative: Error
 */
static int32_t execute_get_config(comm_protocol_t *comm)
{
    char config[512];

    /* Format configuration as key=value pairs (all Configuration tab parameters) */
    sprintf(config, "CONFIG:xl_odr=%d,xl_fs=%d,xl_mode=%d,gy_odr=%d,gy_fs=%d,gy_mode=%d,"
                    "xl_lpf2=%d,xl_lpf2_bw=%d,xl_hpf=%d,xl_fast=%d,"
                    "gy_lpf1=%d,gy_lpf1_bw=%d,sflp_en=%d,sflp_odr=%d,"
                    "xl_offset_x=%.3f,xl_offset_y=%.3f,xl_offset_z=%.3f,xl_offset_en=%d\r\n",
            (int)comm->sensor_mgr->config.xl_odr,
            (int)comm->sensor_mgr->config.xl_fs,
            (int)comm->sensor_mgr->config.xl_mode,
            (int)comm->sensor_mgr->config.gy_odr,
            (int)comm->sensor_mgr->config.gy_fs,
            (int)comm->sensor_mgr->config.gy_mode,
            comm->sensor_mgr->config.xl_lpf2_en ? 1 : 0,
            (int)comm->sensor_mgr->config.xl_lpf2_bw,
            comm->sensor_mgr->config.xl_hpf_en ? 1 : 0,
            comm->sensor_mgr->config.xl_fast_settling_en ? 1 : 0,
            comm->sensor_mgr->config.gy_lpf1_en ? 1 : 0,
            (int)comm->sensor_mgr->config.gy_lpf1_bw,
            comm->sensor_mgr->config.sflp_game_en ? 1 : 0,
            (int)comm->sensor_mgr->config.sflp_odr,
            comm->sensor_mgr->config.xl_offset_x,
            comm->sensor_mgr->config.xl_offset_y,
            comm->sensor_mgr->config.xl_offset_z,
            comm->sensor_mgr->config.xl_offset_en ? 1 : 0);

    comm_protocol_send_string(comm, config);
    return 0;
}

/**
 * @brief  Parse ODR value (Hz) to LSM6DSV enum
 * @param  odr_hz: ODR in Hz (includes standard, HA1, and HA2 modes)
 * @retval lsm6dsv_data_rate_t enum value
 */
static lsm6dsv_data_rate_t parse_odr_value(float odr_hz)
{
    /* Map standard ODR values to enums first (covers common use cases) */
    if (odr_hz <= 0.5f) return LSM6DSV_ODR_OFF;  /* Only explicit zero/very low values power down */
    if (odr_hz < 5.0f) return LSM6DSV_ODR_AT_1Hz875;  /* 1.875 Hz - now works correctly! */
    if (odr_hz < 11.0f) return LSM6DSV_ODR_AT_7Hz5;   /* 7.5 Hz */
    if (odr_hz < 12.0f) return LSM6DSV_ODR_AT_15Hz;   /* 15 Hz - before HA2 12.5 Hz check */

    /* Check for High Accuracy Mode 2 (HA2) ODRs with exact matching */
    if (odr_hz >= 12.4f && odr_hz <= 12.6f) return LSM6DSV_ODR_HA02_AT_12Hz5;    /* 12.5 Hz */
    if (odr_hz >= 24.9f && odr_hz <= 25.1f) return LSM6DSV_ODR_HA02_AT_25Hz;     /* 25 Hz */
    if (odr_hz >= 49.9f && odr_hz <= 50.1f) return LSM6DSV_ODR_HA02_AT_50Hz;     /* 50 Hz */
    if (odr_hz >= 99.9f && odr_hz <= 100.1f) return LSM6DSV_ODR_HA02_AT_100Hz;   /* 100 Hz */
    if (odr_hz >= 199.9f && odr_hz <= 200.1f) return LSM6DSV_ODR_HA02_AT_200Hz;  /* 200 Hz */
    if (odr_hz >= 399.9f && odr_hz <= 400.1f) return LSM6DSV_ODR_HA02_AT_400Hz;  /* 400 Hz */
    if (odr_hz >= 799.9f && odr_hz <= 800.1f) return LSM6DSV_ODR_HA02_AT_800Hz;  /* 800 Hz */
    if (odr_hz >= 1599.9f && odr_hz <= 1600.1f) return LSM6DSV_ODR_HA02_AT_1600Hz; /* 1600 Hz */
    if (odr_hz >= 3199.9f && odr_hz <= 3200.1f) return LSM6DSV_ODR_HA02_AT_3200Hz; /* 3200 Hz */
    if (odr_hz >= 6399.9f && odr_hz <= 6400.1f) return LSM6DSV_ODR_HA02_AT_6400Hz; /* 6400 Hz */

    /* Check for High Accuracy Mode 1 (HA1) ODRs with exact matching */
    if (odr_hz >= 15.5f && odr_hz <= 15.7f) return LSM6DSV_ODR_HA01_AT_15Hz625;  /* 15.625 Hz */
    if (odr_hz >= 31.2f && odr_hz <= 31.3f) return LSM6DSV_ODR_HA01_AT_31Hz25;   /* 31.25 Hz */
    if (odr_hz >= 62.4f && odr_hz <= 62.6f) return LSM6DSV_ODR_HA01_AT_62Hz5;    /* 62.5 Hz */
    if (odr_hz >= 124.9f && odr_hz <= 125.1f) return LSM6DSV_ODR_HA01_AT_125Hz;  /* 125 Hz */
    if (odr_hz >= 249.9f && odr_hz <= 250.1f) return LSM6DSV_ODR_HA01_AT_250Hz;  /* 250 Hz */
    if (odr_hz >= 499.9f && odr_hz <= 500.1f) return LSM6DSV_ODR_HA01_AT_500Hz;  /* 500 Hz */
    if (odr_hz >= 999.9f && odr_hz <= 1000.1f) return LSM6DSV_ODR_HA01_AT_1000Hz; /* 1000 Hz */
    if (odr_hz >= 1999.9f && odr_hz <= 2000.1f) return LSM6DSV_ODR_HA01_AT_2000Hz; /* 2000 Hz */
    if (odr_hz >= 3999.9f && odr_hz <= 4000.1f) return LSM6DSV_ODR_HA01_AT_4000Hz; /* 4000 Hz */
    if (odr_hz >= 7999.9f && odr_hz <= 8000.1f) return LSM6DSV_ODR_HA01_AT_8000Hz; /* 8000 Hz */

    /* Continue with remaining standard ODR values */
    if (odr_hz < 22.0f) return LSM6DSV_ODR_AT_15Hz;   /* Catch-all for 15 Hz region */
    if (odr_hz < 45.0f) return LSM6DSV_ODR_AT_30Hz;
    if (odr_hz < 90.0f) return LSM6DSV_ODR_AT_60Hz;
    if (odr_hz < 180.0f) return LSM6DSV_ODR_AT_120Hz;
    if (odr_hz < 360.0f) return LSM6DSV_ODR_AT_240Hz;
    if (odr_hz < 720.0f) return LSM6DSV_ODR_AT_480Hz;
    if (odr_hz < 1440.0f) return LSM6DSV_ODR_AT_960Hz;
    if (odr_hz < 2880.0f) return LSM6DSV_ODR_AT_1920Hz;
    if (odr_hz < 5760.0f) return LSM6DSV_ODR_AT_3840Hz;

    return LSM6DSV_ODR_AT_7680Hz;  /* Maximum */
}

/**
 * @brief  Parse accelerometer full-scale value to LSM6DSV enum
 * @param  fs_g: Full scale in g (2, 4, 8, 16)
 * @retval lsm6dsv_xl_full_scale_t enum value
 */
static lsm6dsv_xl_full_scale_t parse_xl_fs_value(int fs_g)
{
    switch (fs_g) {
        case 2:  return LSM6DSV_2g;
        case 4:  return LSM6DSV_4g;
        case 8:  return LSM6DSV_8g;
        case 16: return LSM6DSV_16g;
        default: return LSM6DSV_4g;  /* Default */
    }
}

/**
 * @brief  Parse gyroscope full-scale value to LSM6DSV enum
 * @param  fs_dps: Full scale in dps (125, 250, 500, 1000, 2000, 4000)
 * @retval lsm6dsv_gy_full_scale_t enum value
 */
static lsm6dsv_gy_full_scale_t parse_gy_fs_value(int fs_dps)
{
    switch (fs_dps) {
        case 125:  return LSM6DSV_125dps;
        case 250:  return LSM6DSV_250dps;
        case 500:  return LSM6DSV_500dps;
        case 1000: return LSM6DSV_1000dps;
        case 2000: return LSM6DSV_2000dps;
        case 4000: return LSM6DSV_4000dps;
        default:   return LSM6DSV_2000dps;  /* Default */
    }
}

/**
 * @brief  Parse SFLP ODR value (returns raw value for bit field)
 * @param  odr_hz: ODR in Hz (15, 30, 60, 120, 240, 480)
 * @retval SFLP ODR bit value (0-5), or -1 for invalid
 */
static int32_t parse_sflp_odr_value(float odr_hz)
{
    /* SFLP ODR mapping (from datasheet):
     * 0 = 15Hz, 1 = 30Hz, 2 = 60Hz, 3 = 120Hz, 4 = 240Hz, 5 = 480Hz
     */
    if (odr_hz < 22.0f) return 0;      /* 15Hz */
    if (odr_hz < 45.0f) return 1;      /* 30Hz */
    if (odr_hz < 90.0f) return 2;      /* 60Hz */
    if (odr_hz < 180.0f) return 3;     /* 120Hz */
    if (odr_hz < 360.0f) return 4;     /* 240Hz */
    if (odr_hz <= 480.0f) return 5;    /* 480Hz */

    return -1;  /* Invalid */
}
