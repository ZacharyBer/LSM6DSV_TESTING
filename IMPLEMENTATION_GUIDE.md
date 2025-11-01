# LSM6DSV Full Feature Implementation Guide

## STATUS: Firmware 95% Complete, Protocol & GUI Need Integration

This guide provides the remaining implementation details for exposing all firmware features through protocol commands and the Python GUI.

---

## ✅ COMPLETED FIRMWARE FEATURES

### Filtering (100%)
- `sensor_manager_set_xl_lpf2()` - Accel LPF2 with 8 bandwidth levels
- `sensor_manager_set_xl_hpf()` - Accel high-pass filter
- `sensor_manager_set_xl_fast_settling()` - Fast settling mode
- `sensor_manager_set_gy_lpf1()` - Gyro LPF1 with 8 bandwidth levels

### Embedded Functions (100%)
- **Tap**: `sensor_manager_enable_tap_detection()`, `sensor_manager_set_tap_threshold()`, `sensor_manager_set_tap_timing()`, `sensor_manager_set_tap_axes()`, `sensor_manager_set_tap_priority()`, `sensor_manager_set_tap_mode()`
- **Wake-up**: `sensor_manager_enable_wake_up()`, `sensor_manager_set_wake_up_threshold()`, `sensor_manager_set_wake_up_axes()`
- **Free-fall**: `sensor_manager_enable_free_fall()`, `sensor_manager_set_free_fall_threshold()`
- **6D**: `sensor_manager_enable_6d_orientation()`, `sensor_manager_set_6d_threshold()`
- **Tilt**: `sensor_manager_enable_tilt()`
- **Step Counter**: `sensor_manager_enable_step_counter()`, `sensor_manager_get_step_count()`, `sensor_manager_reset_step_counter()`
- **Significant Motion**: `sensor_manager_enable_significant_motion()`

### Infrastructure (100%)
- **Interrupts**: `sensor_manager_config_int1()`, `sensor_manager_config_int2()` - Route all events to INT1/INT2
- **Self-Test**: `sensor_manager_run_self_test()` - Accel & gyro
- **Calibration**: `sensor_manager_calibrate_offsets()` - 100-sample averaging
- **GPIO Callback**: `HAL_GPIO_EXTI_Callback()` - Reads interrupt sources and sends INT:EVENT messages via UART

---

## 📋 REMAINING WORK

### 1. Protocol Commands (Core/Src/comm_protocol.c)

Add these two handler functions before the `parse_odr_value()` function (around line 1100):

```c
/**
 * @brief  Handle SET commands
 * @param  comm: Communication protocol context
 * @param  param1: Parameter name (e.g., "XL_LPF2", "TAP_TIMING")
 * @param  param2: Parameter value
 * @retval 0 on success, negative on error
 */
int32_t comm_protocol_handle_set(comm_protocol_t *comm, const char *param1, const char *param2)
{
    sensor_manager_t *mgr = comm->sensor_mgr;
    char response[128];

    /* Filtering commands */
    if (strcmp(param1, "XL_LPF2") == 0) {
        // Format: "1:3" = enable:bandwidth (0-7)
        int enable, bw;
        if (sscanf(param2, "%d:%d", &enable, &bw) == 2) {
            lsm6dsv_filt_xl_lp2_bandwidth_t bw_val = (lsm6dsv_filt_xl_lp2_bandwidth_t)bw;
            if (sensor_manager_set_xl_lpf2(mgr, enable, bw_val) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid XL_LPF2 params");
        return -1;
    }

    if (strcmp(param1, "XL_HPF") == 0) {
        int enable = atoi(param2);
        if (sensor_manager_set_xl_hpf(mgr, enable) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "XL_HPF failed");
        return -1;
    }

    if (strcmp(param1, "XL_FAST_SETTLING") == 0) {
        int enable = atoi(param2);
        if (sensor_manager_set_xl_fast_settling(mgr, enable) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Fast settling failed");
        return -1;
    }

    if (strcmp(param1, "GY_LPF1") == 0) {
        int enable, bw;
        if (sscanf(param2, "%d:%d", &enable, &bw) == 2) {
            lsm6dsv_filt_gy_lp1_bandwidth_t bw_val = (lsm6dsv_filt_gy_lp1_bandwidth_t)bw;
            if (sensor_manager_set_gy_lpf1(mgr, enable, bw_val) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid GY_LPF1 params");
        return -1;
    }

    /* Power mode commands */
    if (strcmp(param1, "XL_MODE") == 0) {
        // 0=High Perf, 1=LP 2-avg, 2=LP 4-avg, 3=LP 8-avg
        int mode_val = atoi(param2);
        lsm6dsv_xl_mode_t mode = (lsm6dsv_xl_mode_t)mode_val;
        if (sensor_manager_set_xl_mode(mgr, mode) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "XL_MODE failed");
        return -1;
    }

    if (strcmp(param1, "GY_MODE") == 0) {
        // 0=High Perf, 1=Low Power
        int mode_val = atoi(param2);
        lsm6dsv_gy_mode_t mode = (lsm6dsv_gy_mode_t)mode_val;
        if (sensor_manager_set_gy_mode(mgr, mode) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "GY_MODE failed");
        return -1;
    }

    /* Tap detection commands */
    if (strcmp(param1, "TAP_THRESHOLD") == 0) {
        // Format: "X:Y:Z" (0-31 each)
        int x, y, z;
        if (sscanf(param2, "%d:%d:%d", &x, &y, &z) == 3) {
            if (sensor_manager_set_tap_threshold(mgr, x, y, z) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid TAP_THRESHOLD");
        return -1;
    }

    if (strcmp(param1, "TAP_TIMING") == 0) {
        // Format: "SHOCK:QUIET:LATENCY"
        int shock, quiet, latency;
        if (sscanf(param2, "%d:%d:%d", &shock, &quiet, &latency) == 3) {
            if (sensor_manager_set_tap_timing(mgr, shock, quiet, latency) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid TAP_TIMING");
        return -1;
    }

    if (strcmp(param1, "TAP_AXES") == 0) {
        // Format: "X_EN:Y_EN:Z_EN" (0 or 1 each)
        int x, y, z;
        if (sscanf(param2, "%d:%d:%d", &x, &y, &z) == 3) {
            if (sensor_manager_set_tap_axes(mgr, x, y, z) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid TAP_AXES");
        return -1;
    }

    if (strcmp(param1, "TAP_PRIORITY") == 0) {
        // 0=XYZ, 1=YXZ, 2=XZY, 3=ZYX, 5=YZX, 6=ZXY
        int priority = atoi(param2);
        lsm6dsv_tap_axis_priority_t prio = (lsm6dsv_tap_axis_priority_t)priority;
        if (sensor_manager_set_tap_priority(mgr, prio) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "TAP_PRIORITY failed");
        return -1;
    }

    if (strcmp(param1, "TAP_MODE") == 0) {
        // 0=Single only, 1=Both single and double
        int mode = atoi(param2);
        lsm6dsv_tap_mode_t tap_mode = (lsm6dsv_tap_mode_t)mode;
        if (sensor_manager_set_tap_mode(mgr, tap_mode) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "TAP_MODE failed");
        return -1;
    }

    /* Wake-up detection commands */
    if (strcmp(param1, "WAKE_THRESHOLD") == 0) {
        // Format: "THRESHOLD:DURATION"
        int threshold, duration;
        if (sscanf(param2, "%d:%d", &threshold, &duration) == 2) {
            if (sensor_manager_set_wake_up_threshold(mgr, threshold, duration) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid WAKE_THRESHOLD");
        return -1;
    }

    if (strcmp(param1, "WAKE_AXES") == 0) {
        // Format: "X:Y:Z" (0 or 1 each)
        int x, y, z;
        if (sscanf(param2, "%d:%d:%d", &x, &y, &z) == 3) {
            if (sensor_manager_set_wake_up_axes(mgr, x, y, z) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid WAKE_AXES");
        return -1;
    }

    /* Free-fall detection commands */
    if (strcmp(param1, "FF_THRESHOLD") == 0) {
        // Format: "THRESHOLD:DURATION" (threshold 0-7, duration 0-31)
        int threshold, duration;
        if (sscanf(param2, "%d:%d", &threshold, &duration) == 2) {
            if (sensor_manager_set_free_fall_threshold(mgr, threshold, duration) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
                return 0;
            }
        }
        comm_protocol_send_response(comm, RESP_ERROR, "Invalid FF_THRESHOLD");
        return -1;
    }

    /* 6D orientation command */
    if (strcmp(param1, "6D_THRESHOLD") == 0) {
        // Valid values: 50, 60, 70, 80 (degrees)
        int threshold = atoi(param2);
        if (sensor_manager_set_6d_threshold(mgr, threshold) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "6D_THRESHOLD failed");
        return -1;
    }

    /* Unknown command */
    snprintf(response, sizeof(response), "Unknown SET param: %s", param1);
    comm_protocol_send_response(comm, RESP_ERROR, response);
    return -1;
}

/**
 * @brief  Handle GET commands
 * @param  comm: Communication protocol context
 * @param  param: Parameter name to get
 * @retval 0 on success, negative on error
 */
int32_t comm_protocol_handle_get(comm_protocol_t *comm, const char *param)
{
    sensor_manager_t *mgr = comm->sensor_mgr;
    char response[128];

    if (strcmp(param, "STEP_COUNT") == 0) {
        uint16_t steps = 0;
        if (sensor_manager_get_step_count(mgr, &steps) == 0) {
            snprintf(response, sizeof(response), "STEP_COUNT:%u\r\n", steps);
            HAL_UART_Transmit(comm->huart, (uint8_t*)response, strlen(response), 1000);
            return 0;
        }
        comm_protocol_send_response(comm, RESP_ERROR, "STEP_COUNT read failed");
        return -1;
    }

    if (strcmp(param, "CONFIG") == 0) {
        return execute_get_config(comm);
    }

    /* Unknown GET parameter */
    snprintf(response, sizeof(response), "Unknown GET param: %s", param);
    comm_protocol_send_response(comm, RESP_ERROR, response);
    return -1;
}
```

Also update the `execute_enable()` and `execute_disable()` functions to handle the new embedded functions. Search for these functions and add:

```c
/* In execute_enable(), add after existing SFLP handling: */
    if (strcmp(param, "TAP") == 0) {
        if (sensor_manager_enable_tap_detection(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }
    if (strcmp(param, "WAKE_UP") == 0) {
        if (sensor_manager_enable_wake_up(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }
    if (strcmp(param, "FREE_FALL") == 0) {
        if (sensor_manager_enable_free_fall(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }
    if (strcmp(param, "6D") == 0) {
        if (sensor_manager_enable_6d_orientation(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }
    if (strcmp(param, "TILT") == 0) {
        if (sensor_manager_enable_tilt(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }
    if (strcmp(param, "STEP_COUNTER") == 0) {
        if (sensor_manager_enable_step_counter(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }
    if (strcmp(param, "SIG_MOT") == 0) {
        if (sensor_manager_enable_significant_motion(mgr, true) == 0) {
            comm_protocol_send_response(comm, RESP_OK, NULL);
            return 0;
        }
    }

/* Similarly in execute_disable(), change true to false for all */
```

Add command handlers for self-test and calibration in the switch statement (around line 290):

```c
        case CMD_RUN_SELF_TEST:
            if (cmd->param_count >= 1) {
                bool xl_pass, gy_pass;
                if (sensor_manager_run_self_test(mgr, &xl_pass, &gy_pass) == 0) {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "SELF_TEST:XL=%s,GY=%s",
                             xl_pass ? "PASS" : "FAIL", gy_pass ? "PASS" : "FAIL");
                    comm_protocol_send_response(comm, RESP_OK, msg);
                } else {
                    comm_protocol_send_response(comm, RESP_ERROR, "Self-test failed");
                }
            }
            break;

        case CMD_RUN_CALIBRATE:
            if (sensor_manager_calibrate_offsets(mgr) == 0) {
                comm_protocol_send_response(comm, RESP_OK, "Calibration complete");
            } else {
                comm_protocol_send_response(comm, RESP_ERROR, "Calibration failed");
            }
            break;

        case CMD_GET_STEP_COUNT:
            result = comm_protocol_handle_get(comm, "STEP_COUNT");
            break;

        case CMD_RESET_STEP_COUNT:
            if (sensor_manager_reset_step_counter(mgr) == 0) {
                comm_protocol_send_response(comm, RESP_OK, NULL);
            } else {
                comm_protocol_send_response(comm, RESP_ERROR, "Reset failed");
            }
            break;
```

---

### 2. Python GUI Complete Restructure (python-gui/lsm6dsv_gui.py)

**Current Structure**: 2 tabs (Configuration, Interrupt Events)
**New Structure**: 6 tabs with comprehensive controls

Due to length, here's the approach with key widget additions for each tab:

#### Tab 1: Basic Config
Keep existing: I2C Address, Accel ODR/FS, Gyro ODR/FS, SFLP

#### Tab 2: Power & Filtering
```python
# Add after creating notebook
filter_tab = ttk.Frame(notebook)
notebook.add(filter_tab, text="Power & Filtering")

# Power Modes
ttk.Label(filter_tab, text="Accelerometer Mode:").grid(row=0, column=0)
xl_mode_var = tk.StringVar(value="0")  # 0=High Perf
xl_mode_combo = ttk.Combobox(filter_tab, textvariable=xl_mode_var,
    values=["0 - High Performance", "1 - Low Power 2-avg", "2 - Low Power 4-avg", "3 - Low Power 8-avg"])
xl_mode_combo.grid(row=0, column=1)

ttk.Label(filter_tab, text="Gyroscope Mode:").grid(row=1, column=0)
gy_mode_var = tk.StringVar(value="0")
gy_mode_combo = ttk.Combobox(filter_tab, textvariable=gy_mode_var,
    values=["0 - High Performance", "1 - Low Power"])
gy_mode_combo.grid(row=1, column=1)

# Accel Filters
ttk.Label(filter_tab, text="Accel LPF2:").grid(row=2, column=0)
xl_lpf2_en_var = tk.BooleanVar()
ttk.Checkbutton(filter_tab, text="Enable", variable=xl_lpf2_en_var).grid(row=2, column=1)
xl_lpf2_bw_var = tk.StringVar(value="3")  # Medium
ttk.Combobox(filter_tab, textvariable=xl_lpf2_bw_var,
    values=["0 - Ultra Light", "1 - Very Light", "2 - Light", "3 - Medium",
            "4 - Strong", "5 - Very Strong", "6 - Aggressive", "7 - Xtreme"]).grid(row=2, column=2)

xl_hpf_en_var = tk.BooleanVar()
ttk.Checkbutton(filter_tab, text="Accel HPF", variable=xl_hpf_en_var).grid(row=3, column=1)

xl_fast_settling_var = tk.BooleanVar()
ttk.Checkbutton(filter_tab, text="Fast Settling", variable=xl_fast_settling_var).grid(row=4, column=1)

# Gyro Filter
ttk.Label(filter_tab, text="Gyro LPF1:").grid(row=5, column=0)
gy_lpf1_en_var = tk.BooleanVar()
ttk.Checkbutton(filter_tab, text="Enable", variable=gy_lpf1_en_var).grid(row=5, column=1)
gy_lpf1_bw_var = tk.StringVar(value="3")
ttk.Combobox(filter_tab, textvariable=gy_lpf1_bw_var,
    values=["0 - Ultra Light", "1 - Very Light", "2 - Light", "3 - Medium",
            "4 - Strong", "5 - Very Strong", "6 - Aggressive", "7 - Xtreme"]).grid(row=5, column=2)

# Apply button
def apply_power_filter():
    send_command(f"SET:XL_MODE:{xl_mode_var.get().split('-')[0].strip()}")
    send_command(f"SET:GY_MODE:{gy_mode_var.get().split('-')[0].strip()}")
    en = 1 if xl_lpf2_en_var.get() else 0
    bw = xl_lpf2_bw_var.get().split('-')[0].strip()
    send_command(f"SET:XL_LPF2:{en}:{bw}")
    send_command(f"SET:XL_HPF:{1 if xl_hpf_en_var.get() else 0}")
    send_command(f"SET:XL_FAST_SETTLING:{1 if xl_fast_settling_var.get() else 0}")
    en = 1 if gy_lpf1_en_var.get() else 0
    bw = gy_lpf1_bw_var.get().split('-')[0].strip()
    send_command(f"SET:GY_LPF1:{en}:{bw}")

ttk.Button(filter_tab, text="Apply Power & Filtering", command=apply_power_filter).grid(row=6, column=1)
```

#### Tab 3: Interrupts (Enhanced)
Add to existing interrupt tab:
```python
# Tap timing parameters
ttk.Label(int_tab, text="Tap Timing - Shock:").grid(row=10, column=0)
tap_shock_var = tk.StringVar(value="2")
ttk.Spinbox(int_tab, from_=0, to=3, textvariable=tap_shock_var, width=5).grid(row=10, column=1)

ttk.Label(int_tab, text="Quiet:").grid(row=10, column=2)
tap_quiet_var = tk.StringVar(value="2")
ttk.Spinbox(int_tab, from_=0, to=3, textvariable=tap_quiet_var, width=5).grid(row=10, column=3)

ttk.Label(int_tab, text="Latency:").grid(row=10, column=4)
tap_latency_var = tk.StringVar(value="4")
ttk.Spinbox(int_tab, from_=0, to=15, textvariable=tap_latency_var, width=5).grid(row=10, column=5)

# Tap axes
ttk.Label(int_tab, text="Tap Axes:").grid(row=11, column=0)
tap_x_en_var = tk.BooleanVar(value=True)
tap_y_en_var = tk.BooleanVar(value=True)
tap_z_en_var = tk.BooleanVar(value=True)
ttk.Checkbutton(int_tab, text="X", variable=tap_x_en_var).grid(row=11, column=1)
ttk.Checkbutton(int_tab, text="Y", variable=tap_y_en_var).grid(row=11, column=2)
ttk.Checkbutton(int_tab, text="Z", variable=tap_z_en_var).grid(row=11, column=3)

# Tap mode
ttk.Label(int_tab, text="Tap Mode:").grid(row=12, column=0)
tap_mode_var = tk.StringVar(value="1")
ttk.Combobox(int_tab, textvariable=tap_mode_var,
    values=["0 - Single Only", "1 - Both Single & Double"]).grid(row=12, column=1)

# Wake-up duration
ttk.Label(int_tab, text="Wake Duration:").grid(row=13, column=2)
wake_duration_var = tk.StringVar(value="0")
ttk.Spinbox(int_tab, from_=0, to=3, textvariable=wake_duration_var, width=5).grid(row=13, column=3)

# Wake axes
wake_x_en_var = tk.BooleanVar(value=True)
wake_y_en_var = tk.BooleanVar(value=True)
wake_z_en_var = tk.BooleanVar(value=True)
ttk.Label(int_tab, text="Wake Axes:").grid(row=14, column=0)
ttk.Checkbutton(int_tab, text="X", variable=wake_x_en_var).grid(row=14, column=1)
ttk.Checkbutton(int_tab, text="Y", variable=wake_y_en_var).grid(row=14, column=2)
ttk.Checkbutton(int_tab, text="Z", variable=wake_z_en_var).grid(row=14, column=3)

# Free-fall duration
ttk.Label(int_tab, text="FF Duration:").grid(row=15, column=2)
ff_duration_var = tk.StringVar(value="6")
ttk.Spinbox(int_tab, from_=0, to=31, textvariable=ff_duration_var, width=5).grid(row=15, column=3)

# 6D threshold (ADD THIS - currently missing!)
ttk.Label(int_tab, text="6D Threshold:").grid(row=16, column=0)
sixd_threshold_var = tk.StringVar(value="60")
ttk.Combobox(int_tab, textvariable=sixd_threshold_var,
    values=["50", "60", "70", "80"]).grid(row=16, column=1)

# Apply button update to send all new params
def apply_interrupt_config():
    # ... existing code ...

    # Add new commands:
    send_command(f"SET:TAP_TIMING:{tap_shock_var.get()}:{tap_quiet_var.get()}:{tap_latency_var.get()}")
    send_command(f"SET:TAP_AXES:{1 if tap_x_en_var.get() else 0}:{1 if tap_y_en_var.get() else 0}:{1 if tap_z_en_var.get() else 0}")
    send_command(f"SET:TAP_MODE:{tap_mode_var.get().split('-')[0].strip()}")
    send_command(f"SET:WAKE_THRESHOLD:{wake_threshold_var.get()}:{wake_duration_var.get()}")
    send_command(f"SET:WAKE_AXES:{1 if wake_x_en_var.get() else 0}:{1 if wake_y_en_var.get() else 0}:{1 if wake_z_en_var.get() else 0}")
    send_command(f"SET:FF_THRESHOLD:{ff_threshold_var.get()}:{ff_duration_var.get()}")
    send_command(f"SET:6D_THRESHOLD:{sixd_threshold_var.get()}")
```

#### Tab 4: Step Counter & Motion
```python
step_tab = ttk.Frame(notebook)
notebook.add(step_tab, text="Step & Motion")

# Step counter
step_en_var = tk.BooleanVar()
ttk.Checkbutton(step_tab, text="Enable Step Counter", variable=step_en_var).grid(row=0, column=0)

step_count_label = ttk.Label(step_tab, text="Steps: 0")
step_count_label.grid(row=1, column=0)

def update_step_count():
    send_command("GET:STEP_COUNT")
    # Response will be parsed in serial handler

def reset_step_count():
    send_command("RESET:STEP_COUNT")

ttk.Button(step_tab, text="Reset Steps", command=reset_step_count).grid(row=2, column=0)

# Significant motion
sig_mot_en_var = tk.BooleanVar()
ttk.Checkbutton(step_tab, text="Enable Significant Motion", variable=sig_mot_en_var).grid(row=3, column=0)

def apply_step_motion():
    if step_en_var.get():
        send_command("ENABLE:STEP_COUNTER")
    else:
        send_command("DISABLE:STEP_COUNTER")
    if sig_mot_en_var.get():
        send_command("ENABLE:SIG_MOT")
    else:
        send_command("DISABLE:SIG_MOT")

ttk.Button(step_tab, text="Apply", command=apply_step_motion).grid(row=4, column=0)

# Periodically update step count
def periodic_step_update():
    if step_en_var.get():
        update_step_count()
    window.after(2000, periodic_step_update)  # Every 2 seconds

periodic_step_update()
```

#### Tab 5: Self-Test & Calibration
```python
test_tab = ttk.Frame(notebook)
notebook.add(test_tab, text="Self-Test & Cal")

# Self-test results
test_result_text = tk.Text(test_tab, height=10, width=50)
test_result_text.grid(row=0, column=0, columnspan=2)

def run_self_test():
    send_command("RUN:SELF_TEST")
    # Response will show in test_result_text via serial handler

ttk.Button(test_tab, text="Run Accel Self-Test", command=run_self_test).grid(row=1, column=0)
ttk.Button(test_tab, text="Run Gyro Self-Test", command=run_self_test).grid(row=1, column=1)

# Calibration
cal_status_label = ttk.Label(test_tab, text="Calibration Status: Not calibrated")
cal_status_label.grid(row=2, column=0)

def run_calibration():
    cal_status_label.config(text="Calibrating... (keep sensor still)")
    send_command("RUN:CALIBRATE")
    # After 2 seconds (100 samples @ 10ms), update status
    window.after(2000, lambda: cal_status_label.config(text="Calibration complete!"))

ttk.Button(test_tab, text="Calibrate", command=run_calibration).grid(row=3, column=0)
```

#### Tab 6: Event Log
Move existing event log and counters from interrupt tab to this tab.

---

### 3. Serial Response Handlers

In the `process_serial_line()` function, add handlers for new responses:

```python
def process_serial_line(line):
    # ... existing code ...

    # Handle step count response
    if line.startswith("STEP_COUNT:"):
        count = line.split(':')[1]
        step_count_label.config(text=f"Steps: {count}")
        return

    # Handle self-test response
    if line.startswith("SELF_TEST:"):
        parts = line.split(':')[1].split(',')
        xl_result = parts[0].split('=')[1]
        gy_result = parts[1].split('=')[1]
        result_text = f"Accelerometer: {xl_result}\nGyroscope: {gy_result}\n"
        test_result_text.insert(tk.END, result_text)
        return

    # ... rest of existing parsing ...
```

---

## SUMMARY

**Firmware Status**: 95% complete
- ✅ All sensor functions implemented
- ✅ GPIO interrupt callback complete
- ⏳ Protocol commands need integration (~200 lines)

**GUI Status**: Needs restructure
- ⏳ Restructure to 6 tabs
- ⏳ Add ~50 new widgets
- ⏳ Wire up all command sending

**Estimated Completion Time**: 2-3 hours for protocol + GUI integration

All the heavy lifting (firmware algorithms, interrupt handling, sensor APIs) is done. The remaining work is primarily "wiring" - connecting the GUI to the protocol commands to the firmware functions.
