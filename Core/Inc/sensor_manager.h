/**
  ******************************************************************************
  * @file    sensor_manager.h
  * @brief   High-level sensor management interface for LSM6DSV
  ******************************************************************************
  */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lsm6dsv_reg.h"
#include "platform_i2c.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define SENSOR_DATA_BUFFER_SIZE     512
#define FIFO_BUFFER_SIZE           512

/* Sensor Data Structures ----------------------------------------------------*/
typedef struct {
    float acc_x;    /* Acceleration X in mg */
    float acc_y;    /* Acceleration Y in mg */
    float acc_z;    /* Acceleration Z in mg */
    float gyro_x;   /* Angular rate X in mdps */
    float gyro_y;   /* Angular rate Y in mdps */
    float gyro_z;   /* Angular rate Z in mdps */
    float temp;     /* Temperature in Celsius */
    uint32_t timestamp; /* Timestamp in microseconds */
} sensor_data_t;

typedef struct {
    float quat_w;   /* Quaternion W component */
    float quat_x;   /* Quaternion X component */
    float quat_y;   /* Quaternion Y component */
    float quat_z;   /* Quaternion Z component */
    float gravity_x; /* Gravity vector X */
    float gravity_y; /* Gravity vector Y */
    float gravity_z; /* Gravity vector Z */
} sflp_data_t;

typedef struct {
    /* Basic configuration */
    lsm6dsv_data_rate_t xl_odr;
    lsm6dsv_data_rate_t gy_odr;
    lsm6dsv_xl_full_scale_t xl_fs;
    lsm6dsv_gy_full_scale_t gy_fs;
    lsm6dsv_xl_mode_t xl_mode;
    lsm6dsv_gy_mode_t gy_mode;

    /* FIFO configuration */
    lsm6dsv_fifo_mode_t fifo_mode;
    uint8_t fifo_watermark;
    lsm6dsv_fifo_xl_batch_t fifo_xl_batch;
    lsm6dsv_fifo_xl_batch_t fifo_gy_batch;

    /* Interrupts */
    bool int1_drdy_xl;
    bool int1_drdy_gy;
    bool int2_drdy_xl;
    bool int2_drdy_gy;

    /* Embedded functions */
    bool step_counter_en;
    bool tap_detection_en;
    bool free_fall_en;
    bool wake_up_en;
    bool tilt_en;
    bool d6d_en;
    bool significant_motion_en;

    /* SFLP */
    bool sflp_game_en;
    lsm6dsv_sflp_odr_t sflp_odr;

    /* Thresholds */
    uint8_t wake_up_threshold;     /* LSBs - 1 LSB = FS_XL/256 */
    uint8_t wake_up_duration;      /* 1 LSB = 1/ODR_XL */
    uint8_t free_fall_threshold;   /* LSBs */
    uint8_t free_fall_duration;    /* 1 LSB = 1/ODR_XL */
    uint8_t tap_threshold_x;
    uint8_t tap_threshold_y;
    uint8_t tap_threshold_z;
    uint8_t d6d_threshold;         /* 50, 60, 70, 80 degrees */
} sensor_config_t;

typedef struct {
    /* Device context */
    stmdev_ctx_t ctx;
    platform_ctx_t pctx;

    /* Current configuration */
    sensor_config_t config;

    /* Current data */
    sensor_data_t current_data;
    sflp_data_t sflp_data;

    /* Status flags */
    bool initialized;
    bool streaming;
    bool error;
    uint32_t error_code;

    /* FIFO buffer */
    uint8_t fifo_buffer[FIFO_BUFFER_SIZE];

    /* Statistics */
    uint32_t samples_read;
    uint32_t errors_count;

    /* Calibration offsets */
    float acc_offset[3];
    float gyro_offset[3];
} sensor_manager_t;

/* Interrupt event types */
typedef enum {
    INT_EVENT_NONE = 0,
    INT_EVENT_DATA_READY,
    INT_EVENT_FIFO_WATERMARK,
    INT_EVENT_FIFO_FULL,
    INT_EVENT_FIFO_OVERRUN,
    INT_EVENT_STEP_DETECTED,
    INT_EVENT_SINGLE_TAP,
    INT_EVENT_DOUBLE_TAP,
    INT_EVENT_FREE_FALL,
    INT_EVENT_WAKE_UP,
    INT_EVENT_TILT,
    INT_EVENT_6D_ORIENTATION,
    INT_EVENT_SIGNIFICANT_MOTION
} interrupt_event_t;

/* Function Prototypes -------------------------------------------------------*/

/* Initialization and configuration */
int32_t sensor_manager_init(sensor_manager_t *mgr, I2C_HandleTypeDef *hi2c, uint8_t i2c_address);
int32_t sensor_manager_deinit(sensor_manager_t *mgr);
int32_t sensor_manager_reset(sensor_manager_t *mgr);
int32_t sensor_manager_apply_config(sensor_manager_t *mgr, const sensor_config_t *config);
int32_t sensor_manager_get_config(sensor_manager_t *mgr, sensor_config_t *config);
int32_t sensor_manager_load_default_config(sensor_config_t *config);

/* Basic sensor operations */
int32_t sensor_manager_start(sensor_manager_t *mgr);
int32_t sensor_manager_stop(sensor_manager_t *mgr);
int32_t sensor_manager_read_data(sensor_manager_t *mgr, sensor_data_t *data);
int32_t sensor_manager_read_sflp(sensor_manager_t *mgr, sflp_data_t *data);

/* Accelerometer specific */
int32_t sensor_manager_set_xl_odr(sensor_manager_t *mgr, lsm6dsv_data_rate_t odr);
int32_t sensor_manager_set_xl_fs(sensor_manager_t *mgr, lsm6dsv_xl_full_scale_t fs);
int32_t sensor_manager_set_xl_mode(sensor_manager_t *mgr, lsm6dsv_xl_mode_t mode);

/* Gyroscope specific */
int32_t sensor_manager_set_gy_odr(sensor_manager_t *mgr, lsm6dsv_data_rate_t odr);
int32_t sensor_manager_set_gy_fs(sensor_manager_t *mgr, lsm6dsv_gy_full_scale_t fs);
int32_t sensor_manager_set_gy_mode(sensor_manager_t *mgr, lsm6dsv_gy_mode_t mode);

/* FIFO operations */
int32_t sensor_manager_fifo_enable(sensor_manager_t *mgr, lsm6dsv_fifo_mode_t mode);
int32_t sensor_manager_fifo_disable(sensor_manager_t *mgr);
int32_t sensor_manager_fifo_set_watermark(sensor_manager_t *mgr, uint8_t watermark);
int32_t sensor_manager_fifo_read(sensor_manager_t *mgr, sensor_data_t *data, uint16_t *count);
int32_t sensor_manager_fifo_get_status(sensor_manager_t *mgr, uint16_t *level, bool *full, bool *ovr);

/* Embedded functions */
int32_t sensor_manager_enable_step_counter(sensor_manager_t *mgr, bool enable);
int32_t sensor_manager_get_step_count(sensor_manager_t *mgr, uint16_t *steps);
int32_t sensor_manager_reset_step_counter(sensor_manager_t *mgr);

int32_t sensor_manager_enable_tap_detection(sensor_manager_t *mgr, bool enable);
int32_t sensor_manager_set_tap_threshold(sensor_manager_t *mgr, uint8_t x, uint8_t y, uint8_t z);

int32_t sensor_manager_enable_free_fall(sensor_manager_t *mgr, bool enable);
int32_t sensor_manager_set_free_fall_threshold(sensor_manager_t *mgr, uint8_t threshold, uint8_t duration);

int32_t sensor_manager_enable_wake_up(sensor_manager_t *mgr, bool enable);
int32_t sensor_manager_set_wake_up_threshold(sensor_manager_t *mgr, uint8_t threshold, uint8_t duration);

int32_t sensor_manager_enable_6d_orientation(sensor_manager_t *mgr, bool enable);
int32_t sensor_manager_set_6d_threshold(sensor_manager_t *mgr, uint8_t threshold);

int32_t sensor_manager_enable_tilt(sensor_manager_t *mgr, bool enable);

/* SFLP (Sensor Fusion) */
int32_t sensor_manager_enable_sflp(sensor_manager_t *mgr, bool enable);
int32_t sensor_manager_set_sflp_odr(sensor_manager_t *mgr, lsm6dsv_sflp_odr_t odr);

/* Interrupt configuration */
int32_t sensor_manager_config_int1(sensor_manager_t *mgr, bool drdy_xl, bool drdy_gy);
int32_t sensor_manager_config_int2(sensor_manager_t *mgr, bool drdy_xl, bool drdy_gy);
int32_t sensor_manager_get_interrupt_source(sensor_manager_t *mgr, interrupt_event_t *event);

/* Self-test and calibration */
int32_t sensor_manager_run_self_test(sensor_manager_t *mgr, bool *xl_pass, bool *gy_pass);
int32_t sensor_manager_calibrate_offsets(sensor_manager_t *mgr);
int32_t sensor_manager_apply_offsets(sensor_manager_t *mgr, bool enable);

/* Direct register access */
int32_t sensor_manager_read_register(sensor_manager_t *mgr, uint8_t reg, uint8_t *value);
int32_t sensor_manager_write_register(sensor_manager_t *mgr, uint8_t reg, uint8_t value);
int32_t sensor_manager_read_registers(sensor_manager_t *mgr, uint8_t start_reg, uint8_t *buffer, uint16_t len);

/* Status and diagnostics */
int32_t sensor_manager_get_device_id(sensor_manager_t *mgr, uint8_t *id);
bool sensor_manager_is_data_ready(sensor_manager_t *mgr);
const char* sensor_manager_get_error_string(uint32_t error_code);

/* Utility functions */
float sensor_manager_acc_lsb_to_mg(int16_t lsb, lsm6dsv_xl_full_scale_t fs);
float sensor_manager_gy_lsb_to_mdps(int16_t lsb, lsm6dsv_gy_full_scale_t fs);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
