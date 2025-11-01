# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32 embedded firmware project with Python GUI for interfacing with the LSM6DSV 6-axis IMU sensor. It streams real-time accelerometer/gyroscope data over UART to a Python visualization tool.

**Target Hardware:** NUCLEO-U5A5ZJ-Q (STM32U5A5ZJTXQ)
**Sensor:** LSM6DSV 6-axis IMU (I2C)
**Key Feature:** Real-time CSV data streaming at 921600 baud with microsecond timestamps

## Build Commands

### Firmware (STM32)

**Build with CMake:**
```bash
mkdir -p build
cd build
cmake ..
make
```

**Flash to board:**
```bash
# Using st-flash
st-flash write LSM6DSV_TESTING.bin 0x08000000

# Or use STM32CubeProgrammer GUI
```

**Regenerate from .ioc (after hardware changes):**
```bash
# Open LSM6DSV_TESTING.ioc in STM32CubeMX
# Click "GENERATE CODE"
# Rebuild project
```

### Python GUI

**Install dependencies:**
```bash
cd python-gui
pip install -r requirements.txt
```

**Run application:**
```bash
python lsm6dsv_gui.py
```

## Architecture Overview

### Firmware Architecture

The firmware follows a **hybrid polling + interrupt-ready architecture** inspired by IMU-TESTER but designed for extensibility.

#### Core Data Flow
```
LSM6DSV (I2C) → main.c polling loop → CSV formatting → UART TX → Python GUI
                     ↓
                TIM2 timestamps (microseconds)
```

#### Key Architectural Decisions

**1. Simple Direct Implementation in main.c**
- All LSM6DSV logic lives in `Core/Src/main.c` (~450 lines)
- Uses STMicroelectronics driver (`Drivers/LSM6DSV/lsm6dsv_reg.c`) directly
- No complex abstraction layers for the core data path
- **Why:** Proven approach from IMU-TESTER, easy to understand and modify

**2. Platform Abstraction Only for I2C**
- `platform_i2c.c`: Thin wrapper around HAL I2C functions
- Provides retry logic and timestamp generation
- **Why:** Separates hardware-specific code but keeps main logic simple

**3. Fully Active 3-Layer Architecture (v3.0)**
- `sensor_manager.c/h`: Complete API for all LSM6DSV features (~1,800 lines)
- `comm_protocol.c/h`: Full UART command parser with 30+ commands (~1,400 lines)
- `data_formatter.c/h`: CSV and response formatting
- **Why:** Runtime configuration, embedded functions, self-test, calibration all accessible via UART

**4. Timestamp Strategy**
- TIM2 runs at 1MHz (prescaler=16-1 for 16MHz clock)
- `__HAL_TIM_GET_COUNTER(&htim2)` gives direct microsecond timestamps
- 32-bit counter wraps after ~71 minutes (acceptable for sessions)

**5. CSV Streaming Design**
```
Format: LSM6DSV,<timestamp_us>,<ax_mg>,<ay_mg>,<az_mg>,<gx_mdps>,<gy_mdps>,<gz_mdps>
```
- Simple comma-separated, easy to parse in Python
- Timestamp first allows rate calculations
- Units embedded in Python parser (mg = milligravity, mdps = millidegrees/sec)

#### STM32CubeMX Integration

**Critical files that MUST NOT be edited directly:**
- `Core/Src/main.c` - USE USER CODE sections only (between `/* USER CODE BEGIN */` and `/* USER CODE END */`)
- `Core/Inc/main.h` - Auto-generated
- All `*_it.c` files - Use USER CODE sections

**Safe to edit:**
- `Core/Src/platform_i2c.c`
- `Core/Src/sensor_manager.c`
- `Core/Src/comm_protocol.c`
- Files in `python-gui/`

**When modifying .ioc:**
1. Edit `LSM6DSV_TESTING.ioc` in STM32CubeMX
2. Click "GENERATE CODE"
3. Verify USER CODE sections preserved
4. Rebuild project

### Python GUI Architecture

**Threading Model:**
```
Main Thread (tkinter GUI)
    ↓
Serial Thread (daemon) → Queue → Main Thread update_data()
    ↓                       ↑
Serial Port             Data parsing
```

**Key Design Decisions:**

**1. Queue-Based Communication**
- Serial thread reads UART, pushes lines to queue
- Main thread processes queue in periodic `update_data()` (50ms)
- **Why:** Prevents tkinter GUI freezing during I/O

**2. Deque Buffers (maxlen=1000)**
- Ring buffers automatically discard old data
- Constant memory usage
- **Why:** Prevents memory growth during long sessions

**3. Plot Update Strategy**
- Data collected continuously
- Plots redrawn at 50ms intervals (20Hz)
- matplotlib's `draw_idle()` prevents redundant redraws
- **Why:** Balance responsiveness vs CPU usage

**4. CSV Parser**
- Splits on commas, identifies by first field
- `LSM6DSV` → accel/gyro data
- `LSM6DSV_SFLP` → quaternion data
- Non-matching lines → console output
- **Why:** Simple, extensible, handles mixed messages

## Critical Configuration Constants

### Firmware (main.c)

```c
#define LSM6DSV_I2C_ADDR_DEFAULT  LSM6DSV_I2C_ADDR_HIGH  // 0x6B (SDO/SA0 = VDD)
#define STREAMING_INTERVAL_MS 10  // 100Hz streaming rate
#define UART_TX_BUFFER_SIZE 256   // UART transmit buffer
```

**Current sensor config (LSM6DSV_Init function):**
- Accelerometer: ±4g @ 120Hz ODR, high performance mode
- Gyroscope: ±2000dps @ 120Hz ODR, high performance mode
- Block Data Update enabled
- Auto-increment enabled

### Hardware Addresses

**I2C (LSM6DSV):**
- 0x6A: SDO/SA0 pin = GND
- 0x6B: SDO/SA0 pin = VDD (current default)

**Change I2C address:** Edit `current_i2c_address` in main.c

**UART Configuration:**
- Baud: 921600
- Format: 8N1 (8 data, no parity, 1 stop)
- Location: USART1 on PA9(TX)/PA10(RX)

**Timer:**
- TIM2: 1MHz tick (16MHz clock ÷ 16 prescaler)
- Mode: Free-running up counter
- Period: 0xFFFFFFFF (wraps at 4,294 seconds)

## Common Modifications

### Change Sensor ODR/Full-Scale

**Location:** `Core/Src/main.c` → `LSM6DSV_Init()` function

```c
// Change accelerometer range
lsm6dsv_xl_full_scale_set(&lsm6dsv_ctx, LSM6DSV_8g);  // ±8g instead of ±4g

// Change ODR
lsm6dsv_xl_data_rate_set(&lsm6dsv_ctx, LSM6DSV_ODR_AT_240Hz);  // 240Hz

// Change gyro range
lsm6dsv_gy_full_scale_set(&lsm6dsv_ctx, LSM6DSV_1000dps);  // ±1000dps
```

**Available options documented in:** `Drivers/LSM6DSV/lsm6dsv_reg.h`

### Enable SFLP (Sensor Fusion)

**Add to `LSM6DSV_Init()` after gyro configuration:**

```c
/* Enable SFLP game rotation vector */
lsm6dsv_sflp_game_rotation_set(&lsm6dsv_ctx, 1);
lsm6dsv_sflp_data_rate_set(&lsm6dsv_ctx, LSM6DSV_SFLP_ODR_15Hz);
sflp_enabled = 1;
```

**Note:** SFLP reading code exists in `LSM6DSV_ReadSFLP()` but is not enabled by default.

### Change Streaming Rate

```c
#define STREAMING_INTERVAL_MS 20  // 50Hz instead of 100Hz
```

### Change UART Baud Rate

1. Edit `LSM6DSV_TESTING.ioc` → USART1 → Baud Rate
2. Regenerate code in STM32CubeMX
3. Update `baud_var` default in `python-gui/lsm6dsv_gui.py`
4. Rebuild firmware

## File Organization

**Core firmware (all in Core/Src/main.c):**
- `LSM6DSV_Init()`: Sensor initialization and configuration
- `LSM6DSV_ReadData()`: Poll accel/gyro, convert to physical units
- `LSM6DSV_ReadSFLP()`: Read sensor fusion quaternions
- `LSM6DSV_PrintData()`: Format CSV and transmit via UART
- `main()`: Initialization + infinite polling loop

**Supporting layers:**
- `platform_i2c.c`: I2C wrappers + `platform_get_timestamp()` using TIM2
- `sensor_manager.c/h`: Complete API with all LSM6DSV features (~1,800 lines)
- `comm_protocol.c/h`: Full command parser and executor (~1,400 lines)
- `data_formatter.c/h`: CSV and response formatting (~370 lines)
- `stm32u5xx_it.c`: GPIO interrupt handlers for embedded functions

**Python GUI (single file):**
- `python-gui/lsm6dsv_gui.py`: Complete GUI application (~1,100 lines)
  - Serial communication thread
  - CSV parser with interrupt event handling
  - Real-time matplotlib plots (6 IMU axes + 8 fusion channels)
  - Multi-tab configuration UI with 8 sub-tabs
  - Complete control of all sensor features

## Implementation Status

### ✅ Fully Implemented (v3.0 - Complete Embedded Functions)
- **Real-time data streaming** at sensor ODR (configurable 7.5-960Hz)
- **Accelerometer & gyroscope reading** with automatic unit conversion
- **Microsecond timestamps** via TIM2
- **CSV format output** via data_formatter layer
- **Python GUI** with multi-tab interface and 30+ configuration controls
- **I2C auto-detection** (both 0x6A and 0x6B addresses)
- **LED status indicators** (Green=sensor OK, Blue=data streaming, Red=error)
- **sensor_manager.c** (~1,800 lines): Complete API for all LSM6DSV features
- **comm_protocol.c** (~1,400 lines): Full UART command parsing with 30+ commands
- **data_formatter.c** (~370 lines): CSV formatting, status/config reporting
- **UART RX interrupt handler**: Command reception in background
- **Runtime configuration**: ODR, full-scale, power modes, filtering via UART
- **3-layer architecture**: Clean separation (sensor/protocol/formatter)
- **All embedded functions implemented:**
  - **Tap detection**: Single/double tap with X/Y/Z thresholds, timing parameters
  - **Wake-up detection**: Activity/inactivity with axis-specific enables
  - **Free-fall detection**: 8 threshold levels (156-500mg)
  - **6D orientation**: 4 angle thresholds (50°/60°/70°/80°)
  - **Tilt detection**: Automatic tilt event detection
  - **Step counter**: With get/reset commands
  - **Significant motion**: Motion detection algorithm
- **Hardware filtering**: LPF2, HPF, fast settling for accel; LPF1 for gyro
- **Power modes**: Low power, high performance, high accuracy modes
- **GPIO interrupt event reporting**: Real-time INT:EVENT_TYPE messages via UART
- **Self-test**: Accelerometer and gyroscope validation
- **Calibration**: Offset calculation and storage
- **SFLP (Sensor Fusion)**: Enable/disable with ODR control

### 🔧 Partially Implemented
- **SFLP quaternion reading**: Can enable/disable SFLP but ST driver doesn't expose quaternion registers yet
- **FIFO operations**: API framework exists (functions stubbed)

### ❌ Not Implemented (Future GUI Features)
- **Data logging**: GUI doesn't save CSV files to disk yet
- **FFT analysis**: No frequency domain analysis
- **3D visualization**: No quaternion-based 3D orientation display (pending SFLP driver update)

## Architecture Overview (v2.0)

### Implemented 3-Layer Architecture

```
Python GUI (lsm6dsv_gui.py)
    ↕ UART 921600 baud (Commands + Data)
┌─────────────────────────────────────────┐
│         STM32U5A5 Firmware              │
│                                         │
│  main.c (orchestration layer)          │
│    ├─> comm_protocol.c                 │
│    │     └─> Parse UART commands       │
│    │         Execute via sensor_mgr    │
│    │                                    │
│    ├─> sensor_manager.c                │
│    │     └─> Runtime configuration     │
│    │         SFLP enable/disable        │
│    │         Data reading               │
│    │                                    │
│    └─> data_formatter.c                │
│          └─> CSV formatting             │
│              Response messages          │
│                                         │
│  platform_i2c.c (HAL abstraction)      │
│  lsm6dsv_reg.c (ST driver)             │
└─────────────────────────────────────────┘
    ↕ I2C 400kHz
  LSM6DSV Sensor

```

### Supported Commands (Python GUI → Firmware)

**Basic Configuration:**
- `SET:ACC_ODR:<Hz>` - Change accelerometer ODR (7.5-960 Hz)
- `SET:ACC_FS:<g>` - Change full scale (2, 4, 8, 16 g)
- `SET:ACC_MODE:<mode>` - Set power mode (0=Low Power, 1=High Performance, 2=High Accuracy)
- `SET:GYRO_ODR:<Hz>` - Change gyroscope ODR (7.5-960 Hz)
- `SET:GYRO_FS:<dps>` - Change full scale (125, 250, 500, 1000, 2000, 4000 dps)
- `SET:GYRO_MODE:<mode>` - Set power mode (0=Low Power, 1=High Performance, 2=Sleep)

**Hardware Filtering:**
- `SET:XL_LPF2:<0|1>` - Enable/disable accelerometer LPF2
- `SET:XL_LPF2_BW:<0-7>` - Set LPF2 bandwidth (0-7)
- `SET:XL_HPF:<0|1>` - Enable/disable accelerometer high-pass filter
- `SET:XL_FAST_SETTLING:<0|1>` - Enable/disable fast settling mode
- `SET:GY_LPF1:<0|1>` - Enable/disable gyroscope LPF1
- `SET:GY_LPF1_BW:<0-7>` - Set LPF1 bandwidth (0-7)

**Tap Detection:**
- `ENABLE:TAP` / `DISABLE:TAP` - Enable/disable tap detection
- `SET:TAP_THRESHOLD_X:<0-31>` - Set X-axis tap threshold
- `SET:TAP_THRESHOLD_Y:<0-31>` - Set Y-axis tap threshold
- `SET:TAP_THRESHOLD_Z:<0-31>` - Set Z-axis tap threshold
- `SET:TAP_SHOCK:<0-3>` - Set shock time window
- `SET:TAP_QUIET:<0-3>` - Set quiet time window
- `SET:TAP_LATENCY:<0-15>` - Set tap gap/latency
- `SET:TAP_AXES:<111>` - Enable axes (format: XYZ as binary string, e.g., "111")
- `SET:TAP_PRIORITY:<0-5>` - Set axis priority (0=XYZ, 1=YXZ, 2=XZY, 3=ZYX, 4=YZX, 5=ZXY)
- `SET:TAP_MODE:<0|1>` - Set mode (0=Single only, 1=Single+Double)

**Wake-Up Detection:**
- `ENABLE:WAKE_UP` / `DISABLE:WAKE_UP` - Enable/disable wake-up detection
- `SET:WAKE_THRESHOLD:<0-63>` - Set wake-up threshold
- `SET:WAKE_DURATION:<0-3>` - Set wake-up duration
- `SET:WAKE_AXES:<111>` - Enable axes (format: XYZ as binary string)

**Free-Fall Detection:**
- `ENABLE:FREE_FALL` / `DISABLE:FREE_FALL` - Enable/disable free-fall detection
- `SET:FF_THRESHOLD:<0-7>` - Set threshold (0=156mg, 1=219mg, ..., 7=500mg)
- `SET:FF_DURATION:<0-31>` - Set free-fall duration

**6D Orientation:**
- `ENABLE:6D` / `DISABLE:6D` - Enable/disable 6D orientation detection
- `SET:6D_THRESHOLD:<50|60|70|80>` - Set angle threshold (degrees)

**Other Embedded Functions:**
- `ENABLE:TILT` / `DISABLE:TILT` - Enable/disable tilt detection
- `ENABLE:SIG_MOTION` / `DISABLE:SIG_MOTION` - Enable/disable significant motion
- `ENABLE:STEP_COUNTER` / `DISABLE:STEP_COUNTER` - Enable/disable step counter
- `GET_STEP_COUNT` - Get current step count (responds: `STEP_COUNT:<value>`)
- `RESET_STEP_COUNT` - Reset step counter to zero

**SFLP (Sensor Fusion):**
- `ENABLE:SFLP` / `DISABLE:SFLP` - Enable/disable game rotation vector
- `SET:SFLP_ODR:<Hz>` - Set fusion rate (15, 30, 60, 120, 240, 480 Hz)

**Self-Test & Calibration:**
- `SELF_TEST` - Run self-test (responds: `SELF_TEST:XL=PASS/FAIL,GY=PASS/FAIL`)
- `CALIBRATE` - Calibrate offsets (place sensor on flat surface first)

**System:**
- `PING` - Test connectivity (responds: `OK`)
- `STATUS` - Get system status
- `GET:CONFIG` - Get current configuration
- `VERSION` - Get firmware version
- `RESET` - Reset sensor
- `STREAM_START` / `STREAM_STOP` - Control data streaming
- `REG_READ:<addr>` - Read register (hex address)
- `REG_WRITE:<addr>:<value>` - Write register (hex)

### Interrupt Event Messages (Firmware → GUI)

The firmware sends real-time interrupt event messages via UART when embedded functions trigger:

- `INT:WAKE_UP` - Wake-up/activity detected
- `INT:SINGLE_TAP` - Single tap detected
- `INT:DOUBLE_TAP` - Double tap detected
- `INT:FREE_FALL` - Free-fall condition detected
- `INT:6D_ORIENT` - 6D orientation change detected
- `INT:TILT` - Tilt event detected
- `INT:STEP_DET` - Step detected by step counter
- `INT:SIG_MOT` - Significant motion detected
- `INT:SLEEP_CHANGE` - Sleep state change

These messages are parsed by the Python GUI and displayed in the Event Log tab with timestamps and counters.

### Python GUI Tab Structure

**Tab 1: IMU Data**
- 6 separate plots (Accel X/Y/Z, Gyro X/Y/Z)
- Real-time visualization at 20 Hz
- Zoom, pan, export capabilities

**Tab 2: Sensor Fusion**
- 8 plots: Quaternions (W, X, Y, Z) + Euler angles (Roll, Pitch, Yaw)
- Side-by-side comparison
- Only active when SFLP enabled

**Tab 3: Configuration** (3 sub-tabs)
- **Basic Config**: ODR, Full Scale with individual Apply buttons
- **Power & Filtering**: Power modes, LPF2/HPF/LPF1 controls with bandwidth settings
- **SFLP**: Sensor fusion enable/disable and ODR configuration

**Tab 4: Embedded Functions** (5 sub-tabs)
- **Tap Detection**: X/Y/Z thresholds, shock/quiet/latency timing, axes, priority, mode
- **Wake-Up & Free Fall**: Thresholds, durations, axis enables for both features
- **6D, Tilt & Motion**: 6D threshold, tilt enable, significant motion enable
- **Step & Cal**: Step counter with display, self-test with results, calibration
- **Event Log**: Real-time event messages with timestamps and counters

**Tab 5: Console**
- Raw serial data monitoring
- Command echo and response display
- Error message logging

### Future Enhancements

1. **Complete SFLP Quaternion Reading**: Waiting for ST driver update to expose quaternion registers
2. **Enable FIFO Mode**: High-speed data buffering (framework exists, needs testing)
3. **Python GUI Data Logging**: Save CSV data to files
4. **3D Visualization**: Quaternion-based 3D orientation display (pending SFLP completion)
5. **FFT Analysis Tab**: Frequency domain analysis of IMU data

## Debugging Tips

**Sensor not found:**
- Check I2C address matches SDO/SA0 hardware pin
- Verify I2C wiring (PB10=SCL, PB11=SDA)
- Use I2C scanner to detect devices

**No data in GUI:**
- Verify Green LED is on (sensor OK)
- Check baud rate is 921600 (not 115200)
- Look at console tab for error messages
- Test with serial terminal (should see CSV lines)

**Slow data rate:**
- Check `STREAMING_INTERVAL_MS` (smaller = faster)
- Verify USB cable supports high baud rates
- Some USB-serial adapters can't do 921600 baud

**Timestamp issues:**
- TIM2 must be started: `HAL_TIM_Base_Start(&htim2)`
- Counter wraps at 2^32 microseconds (~71 minutes)
- Check prescaler is 16-1 for 16MHz clock

**Modifying code:**
- Only edit USER CODE sections in .ioc-generated files
- Regenerating code will erase non-USER CODE edits
- Test after every .ioc regeneration

## Data Format Reference

### LSM6DSV Data Packet
```
LSM6DSV,<timestamp>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\r\n
        uint32     float float float float float float
        (us)       (mg)  (mg)  (mg)  (mdps)(mdps)(mdps)
```

### SFLP Data Packet (when enabled)
```
LSM6DSV_SFLP,<timestamp>,<qw>,<qx>,<qy>,<qz>\r\n
             uint32      float float float float
             (us)        (normalized quaternion)
```

### Conversion Functions
```c
// In lsm6dsv_reg.h
float lsm6dsv_from_fs4_to_mg(int16_t lsb);     // ±4g accelerometer
float lsm6dsv_from_fs2000_to_mdps(int16_t lsb); // ±2000dps gyroscope
// Other ranges have equivalent functions
```

## References

- **LSM6DSV Driver:** `Drivers/LSM6DSV/lsm6dsv_reg.h` - Full API documentation
- **STM32 HAL:** Standard STM32U5 HAL library
- **Inspired by:** https://github.com/ZacharyBer/IMU-TESTER.git
- **Datasheet:** ST LSM6DSV datasheet for register details

## Important Constraints

1. **USER CODE sections only** in CubeMX-generated files
2. **TIM2 must run** for timestamps to work
3. **UART baud 921600** is hardcoded in multiple places (sync if changing)
4. **I2C address** must match hardware SDO/SA0 pin state
5. **CSV format** is fixed (Python parser expects exact column count)
6. **Main loop is blocking** - long operations will delay streaming
7. **No multithreading** on STM32 side - keep ISRs short if adding interrupts
