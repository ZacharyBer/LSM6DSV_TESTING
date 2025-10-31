/**
  ******************************************************************************
  * @file    data_formatter.h
  * @brief   Data formatting utilities for sensor output
  ******************************************************************************
  */

#ifndef DATA_FORMATTER_H
#define DATA_FORMATTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensor_manager.h"
#include "comm_protocol.h"
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define FORMAT_BUFFER_SIZE      512
#define CSV_HEADER_MAX_LEN      256
#define JSON_BUFFER_SIZE        512

/* Data format configuration */
typedef struct {
    bool include_timestamp;
    bool include_acc;
    bool include_gyro;
    bool include_temperature;
    bool include_sflp;
    bool include_quaternion;
    bool include_gravity;
    bool include_units;
    uint8_t decimal_places;
} format_config_t;

/* Function prototypes -------------------------------------------------------*/

/* CSV formatting */
int32_t data_formatter_csv_header(char *buffer, size_t size, const format_config_t *config);
int32_t data_formatter_csv_data(char *buffer, size_t size,
                                const sensor_data_t *sensor_data,
                                const sflp_data_t *sflp_data,
                                const format_config_t *config);
int32_t data_formatter_csv_sflp(char *buffer, uint16_t size, const sflp_data_t *sflp, uint32_t timestamp);

/* JSON formatting */
int32_t data_formatter_json_data(char *buffer, size_t size,
                                 const sensor_data_t *sensor_data,
                                 const sflp_data_t *sflp_data,
                                 const format_config_t *config);

/* Binary formatting */
int32_t data_formatter_binary_pack(uint8_t *buffer, size_t size,
                                   const sensor_data_t *sensor_data,
                                   const sflp_data_t *sflp_data,
                                   const format_config_t *config);

/* Stream formatting */
int32_t data_formatter_stream(comm_protocol_t *comm,
                              const sensor_data_t *sensor_data,
                              const sflp_data_t *sflp_data);

/* Configuration formatting */
int32_t data_formatter_config_json(char *buffer, size_t size,
                                   const sensor_config_t *config);

/* Status formatting */
int32_t data_formatter_status_json(char *buffer, size_t size,
                                   const sensor_manager_t *mgr);

/* Register dump formatting */
int32_t data_formatter_register_dump(char *buffer, size_t size,
                                     uint8_t start_reg, uint8_t *values, uint8_t count);

/* Utility functions */
void data_formatter_load_default_config(format_config_t *config);
float data_formatter_truncate(float value, uint8_t decimal_places);

#ifdef __cplusplus
}
#endif

#endif /* DATA_FORMATTER_H */
