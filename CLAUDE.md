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

**3. Unused Layers (Designed but Not Active)**
- `sensor_manager.c/h`: High-level API for advanced features (not currently used)
- `comm_protocol.c/h`: Command parser framework (not implemented)
- **Why:** Architecture ready for future commands, but current code works without them

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
- `sensor_manager.c/h`: Advanced API (partially implemented, not used in main)
- `comm_protocol.c/h`: Command framework (headers only, not implemented)

**Python GUI (single file):**
- `python-gui/lsm6dsv_gui.py`: Complete GUI application (~680 lines)
  - Serial communication thread
  - CSV parser
  - Real-time matplotlib plots
  - Configuration UI (commands not implemented yet)

## Implementation Status

### ✅ Fully Implemented (v2.0 - Full Integration)
- **Real-time data streaming** at sensor ODR (configurable 7.5-960Hz)
- **Accelerometer & gyroscope reading** with automatic unit conversion
- **Microsecond timestamps** via TIM2
- **CSV format output** via data_formatter layer
- **Python GUI** with 3-axis plots and full command interface
- **I2C auto-detection** (both 0x6A and 0x6B addresses)
- **LED status indicators** (Green=sensor OK, Blue=data streaming, Red=error)
- **sensor_manager.c** (~1,400 lines): Runtime configuration, SFLP support, calibration framework
- **comm_protocol.c** (~1,050 lines): Full UART command parsing and execution
- **data_formatter.c** (~370 lines): CSV formatting, status/config reporting
- **UART RX interrupt handler**: Command reception in background
- **Runtime configuration**: Change ODR, full-scale, enable SFLP via commands
- **3-layer architecture**: Clean separation (sensor/protocol/formatter)
- **Main.c refactored**: Uses new layers, ~30% code reduction

### 🔧 Partially Implemented (Stub Functions)
- **SFLP quaternion reading**: Can enable/disable SFLP but driver doesn't expose quaternion registers
- **Embedded functions**: API exists with stubs (step counter, tap, free fall, wake-up, tilt, 6D)
- **FIFO operations**: API exists with stubs
- **Interrupt event reporting**: GPIO callbacks exist but event decoding not implemented
- **Calibration**: Offset framework exists, self-test stubbed

### ❌ Not Implemented (Python GUI Features)
- **Data logging**: GUI doesn't save CSV files to disk yet
- **FFT analysis**: GUI has no frequency domain analysis
- **Calibration wizard**: No GUI-guided calibration process
- **3D visualization**: No quaternion-based 3D orientation display

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

**Configuration:**
- `SET:ACC_ODR:<Hz>` - Change accelerometer ODR (7.5-960 Hz)
- `SET:ACC_FS:<g>` - Change full scale (2, 4, 8, 16 g)
- `SET:GYRO_ODR:<Hz>` - Change gyroscope ODR
- `SET:GYRO_FS:<dps>` - Change full scale (125-4000 dps)

**SFLP (Sensor Fusion):**
- `ENABLE:SFLP` - Enable game rotation vector
- `DISABLE:SFLP` - Disable sensor fusion
- `SET:SFLP_ODR:<Hz>` - Set fusion rate (15-480 Hz)

**System:**
- `PING` - Test connectivity (responds "OK")
- `STATUS` - Get system status
- `GET:CONFIG` - Get current configuration

### Future Enhancements

1. **Complete SFLP Integration**: Add quaternion register reading when ST updates driver
2. **Implement Embedded Functions**: Enable step counter, tap, free fall via GUI
3. **Add Interrupt Event Reporting**: Send `INT:event_name` messages to GUI
4. **Enable FIFO Mode**: High-speed data buffering
5. **Python GUI Logging**: Save CSV data to files
6. **Calibration Wizard**: GUI-guided offset calibration

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
