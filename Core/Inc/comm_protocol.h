/**
  ******************************************************************************
  * @file    comm_protocol.h
  * @brief   UART communication protocol for sensor configuration and data
  ******************************************************************************
  */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sensor_manager.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define COMM_RX_BUFFER_SIZE     256
#define COMM_TX_BUFFER_SIZE     1024
#define COMM_MAX_CMD_LENGTH     128
#define COMM_MAX_PARAMS         10

/* Command types */
typedef enum {
    CMD_NONE = 0,
    /* System commands */
    CMD_PING,
    CMD_RESET,
    CMD_STATUS,
    CMD_VERSION,

    /* Configuration commands */
    CMD_SET,
    CMD_GET,
    CMD_CONFIG_SAVE,
    CMD_CONFIG_LOAD,

    /* Data streaming */
    CMD_STREAM_START,
    CMD_STREAM_STOP,
    CMD_STREAM_CONFIG,

    /* Register access */
    CMD_REG_READ,
    CMD_REG_WRITE,
    CMD_REG_DUMP,

    /* Calibration */
    CMD_CALIBRATE,
    CMD_SELF_TEST,

    /* Embedded functions */
    CMD_ENABLE_FUNC,
    CMD_DISABLE_FUNC,
    CMD_GET_STEP_COUNT,
    CMD_RESET_STEP_COUNT,

    /* FIFO */
    CMD_FIFO_READ,
    CMD_FIFO_STATUS,
    CMD_FIFO_CONFIG,

    /* SFLP */
    CMD_SFLP_ENABLE,
    CMD_SFLP_DISABLE,
    CMD_SFLP_CONFIG,

    CMD_UNKNOWN
} command_type_t;

/* Stream formats */
typedef enum {
    STREAM_FORMAT_CSV = 0,
    STREAM_FORMAT_JSON,
    STREAM_FORMAT_BINARY
} stream_format_t;

/* Response codes */
typedef enum {
    RESP_OK = 0,
    RESP_ERROR,
    RESP_INVALID_CMD,
    RESP_INVALID_PARAM,
    RESP_SENSOR_ERROR,
    RESP_BUSY,
    RESP_NOT_INITIALIZED,
    RESP_TIMEOUT
} response_code_t;

/* Parameter types for SET/GET commands */
typedef enum {
    PARAM_ACC_ODR,
    PARAM_ACC_FS,
    PARAM_ACC_MODE,
    PARAM_GYRO_ODR,
    PARAM_GYRO_FS,
    PARAM_GYRO_MODE,
    PARAM_FIFO_MODE,
    PARAM_FIFO_WATERMARK,
    PARAM_INT1_CONFIG,
    PARAM_INT2_CONFIG,
    PARAM_WAKE_THRESHOLD,
    PARAM_WAKE_DURATION,
    PARAM_FF_THRESHOLD,
    PARAM_FF_DURATION,
    PARAM_TAP_THRESHOLD_X,
    PARAM_TAP_THRESHOLD_Y,
    PARAM_TAP_THRESHOLD_Z,
    PARAM_6D_THRESHOLD,
    PARAM_SFLP_ODR,
    PARAM_UNKNOWN
} parameter_type_t;

/* Embedded function types */
typedef enum {
    FUNC_STEP_COUNTER,
    FUNC_TAP_DETECTION,
    FUNC_FREE_FALL,
    FUNC_WAKE_UP,
    FUNC_TILT,
    FUNC_6D_ORIENTATION,
    FUNC_SIGNIFICANT_MOTION,
    FUNC_UNKNOWN
} function_type_t;

/* Command structure */
typedef struct {
    command_type_t type;
    char params[COMM_MAX_PARAMS][32];
    uint8_t param_count;
} command_t;

/* Communication protocol context */
typedef struct {
    /* UART handle */
    UART_HandleTypeDef *huart;

    /* Sensor manager reference */
    sensor_manager_t *sensor_mgr;

    /* Buffers */
    uint8_t rx_buffer[COMM_RX_BUFFER_SIZE];
    uint8_t tx_buffer[COMM_TX_BUFFER_SIZE];
    uint16_t rx_index;
    uint16_t tx_index;

    /* Command processing */
    char cmd_buffer[COMM_MAX_CMD_LENGTH];
    uint16_t cmd_index;
    bool cmd_ready;

    /* Streaming state */
    bool streaming_enabled;
    stream_format_t stream_format;
    uint32_t stream_interval_ms;
    uint32_t last_stream_time;
    bool include_timestamp;
    bool include_temperature;
    bool include_sflp;

    /* Statistics */
    uint32_t commands_received;
    uint32_t commands_processed;
    uint32_t stream_packets_sent;
    uint32_t errors;
} comm_protocol_t;

/* Function prototypes -------------------------------------------------------*/

/* Initialization */
int32_t comm_protocol_init(comm_protocol_t *comm, UART_HandleTypeDef *huart, sensor_manager_t *sensor_mgr);
int32_t comm_protocol_deinit(comm_protocol_t *comm);

/* UART interrupt handlers */
void comm_protocol_rx_callback(comm_protocol_t *comm, uint8_t data);
void comm_protocol_tx_complete_callback(comm_protocol_t *comm);

/* Command processing */
void comm_protocol_process(comm_protocol_t *comm);
int32_t comm_protocol_parse_command(comm_protocol_t *comm, const char *cmd_str, command_t *cmd);
int32_t comm_protocol_execute_command(comm_protocol_t *comm, const command_t *cmd);

/* Response functions */
int32_t comm_protocol_send_response(comm_protocol_t *comm, response_code_t code, const char *message);
int32_t comm_protocol_send_data(comm_protocol_t *comm, const uint8_t *data, uint16_t length);
int32_t comm_protocol_send_string(comm_protocol_t *comm, const char *str);

/* Streaming functions */
int32_t comm_protocol_start_streaming(comm_protocol_t *comm, stream_format_t format, uint32_t interval_ms);
int32_t comm_protocol_stop_streaming(comm_protocol_t *comm);
void comm_protocol_stream_update(comm_protocol_t *comm);
int32_t comm_protocol_send_stream_data(comm_protocol_t *comm, const sensor_data_t *data, const sflp_data_t *sflp);

/* Configuration handlers */
int32_t comm_protocol_handle_set(comm_protocol_t *comm, const char *param, const char *value);
int32_t comm_protocol_handle_get(comm_protocol_t *comm, const char *param);

/* Register access */
int32_t comm_protocol_read_register(comm_protocol_t *comm, uint8_t reg);
int32_t comm_protocol_write_register(comm_protocol_t *comm, uint8_t reg, uint8_t value);
int32_t comm_protocol_dump_registers(comm_protocol_t *comm);

/* Utility functions */
parameter_type_t comm_protocol_get_param_type(const char *param);
function_type_t comm_protocol_get_function_type(const char *func);
const char* comm_protocol_get_response_string(response_code_t code);
bool comm_protocol_is_valid_hex(const char *str);
uint8_t comm_protocol_hex_to_byte(const char *hex);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PROTOCOL_H */
