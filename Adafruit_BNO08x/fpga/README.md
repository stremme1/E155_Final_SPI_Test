# BNO08X FPGA Implementation

This directory contains a SystemVerilog implementation of the BNO08X sensor interface for FPGA, ported from the C implementation in `Code_for_C_imp` and using the Adafruit BNO08x library structure.

## Overview

The implementation provides:
- SPI master interface for BNO08X communication (SPI Mode 3)
- SHTP (Sensor Hub Transport Protocol) handler
- BNO08X controller with initialization and sensor reading
- Sensor data processing (quaternion to Euler conversion)
- Drum trigger logic based on orientation and motion

## Clock Configuration

The system is designed for a **3MHz clock** derived from a 48MHz HSOSC divided by 16:

```systemverilog
// HARDWARE CLOCK - HSOSC (ACTIVE FOR HARDWARE)
// CLKHF_DIV(2'b11) = divide by 16 to get 3MHz from 48MHz
HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
    .CLKHFPU(1'b1),   // Power up
    .CLKHFEN(1'b1),   // Enable
    .CLKHF(clk)       // Output clock (3MHz from 48MHz / 16)
);
```

## Module Structure

### 1. `bno08x_spi_master.sv`
SPI master interface implementing SPI Mode 3 (CPOL=1, CPHA=1):
- Clock idles high
- Data captured on rising edge
- Maximum SPI clock: 3MHz (within BNO08X specifications)
- Handles CS setup/hold timing requirements

**Key Features:**
- Configurable data length (up to 65535 bytes)
- Separate read/write modes
- Proper timing for CS, SCK, MOSI, MISO signals

### 2. `shtp_protocol.sv`
SHTP protocol handler for packet assembly and parsing:
- 4-byte SHTP header format
- Channel and sequence number management
- Packet length handling with continuation bit support

**SHTP Header Format:**
- Byte 0: Length LSB
- Byte 1: Length MSB (bit 15 = continuation bit)
- Byte 2: Channel
- Byte 3: Sequence Number

### 3. `bno08x_controller.sv`
Main controller for BNO08X initialization and operation:
- Hardware reset and wake signal handling
- Product ID request/response
- Sensor enable/configuration
- Interrupt-driven data reading
- SHTP channel management

**State Machine:**
- `INIT_RESET`: Hardware reset sequence
- `INIT_WAIT_INT`: Wait for interrupt after reset
- `INIT_GET_PRODUCT_ID`: Request product information
- `INIT_WAIT_PRODUCT_ID`: Wait for product ID response
- `IDLE`: Ready for sensor operations
- `ENABLE_SENSOR`: Configure sensor reports
- `READ_SENSOR_DATA`: Read sensor data via SPI
- `PROCESS_SENSOR_DATA`: Parse and process sensor reports

### 4. `sensor_processor.sv`
Sensor data processing and drum trigger logic:
- Parses quaternion and gyroscope reports
- Converts quaternion to Euler angles (roll, pitch, yaw)
- Implements drum trigger logic based on orientation zones
- Supports dual-sensor system (left/right hand)

**Drum Trigger Mapping:**
- 0: Snare drum
- 1: Hi-hat
- 2: Kick drum (button trigger)
- 3: High tom
- 4: Mid tom
- 5: Crash cymbal
- 6: Ride cymbal
- 7: Floor tom
- 8: No trigger

### 5. `quaternion_to_euler.sv`
Quaternion to Euler angle conversion:
- Converts quaternion (Q30 format) to Euler angles (degrees, Q16 format)
- Uses ZYX (yaw-pitch-roll) convention
- Implements atan2 and asin functions (simplified version)

**Note:** The current implementation uses simplified approximations. For production use, implement proper CORDIC algorithms or lookup tables for accurate angle conversion.

### 6. `bno08x_drum_system.sv`
Top-level module integrating all components:
- Connects SPI controller, sensor processor, and drum logic
- Manages yaw offset configuration
- Provides unified interface for the drum system

## SPI Interface Connections

```
FPGA                    BNO08X
----                    ------
spi_cs_n    ------>     H_CSN (Pin 18)
spi_sck     ------>     H_SCL/SCK (Pin 19)
spi_mosi    ------>     H_MOSI (Pin 17)
spi_miso    <------     H_SDA/H_MISO (Pin 20)
h_intn      <------     H_INTN (Pin 14)
wake        ------>     PS0/WAKE (Pin 6)
reset_bno   ------>     NRST (Pin 11)
```

## Usage Example

```systemverilog
// Instantiate the drum system
bno08x_drum_system drum_sys (
    .clk(clk_3mhz),
    .rst_n(rst_n),
    .spi_cs_n(spi_cs_n),
    .spi_sck(spi_sck),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .h_intn(h_intn),
    .wake(wake),
    .reset_bno(reset_bno),
    .sensor_select(1'b0),  // 0 = right hand, 1 = left hand
    .enable_system(1'b1),
    .sensor_id(8'h08),     // SH2_GAME_ROTATION_VECTOR
    .report_interval(32'd10000),  // 10ms = 100Hz
    .set_yaw_offset(1'b0),
    .yaw_offset1(32'd0),
    .yaw_offset2(32'd0),
    .drum_trigger(drum_trigger),
    .trigger_valid(trigger_valid),
    .initialized(initialized),
    .error(error),
    // ... other outputs
);
```

## Sensor Configuration

### Supported Sensor IDs (from sh2.h):
- `0x01`: Accelerometer
- `0x02`: Gyroscope (Calibrated)
- `0x05`: Rotation Vector
- `0x08`: Game Rotation Vector (recommended for drum system)
- `0x09`: Geomagnetic Rotation Vector

### Report Intervals:
- Minimum: 1000 microseconds (1000 Hz)
- Typical: 10000 microseconds (100 Hz)
- Maximum: Depends on sensor type (see BNO08X datasheet)

## Drum Logic

The drum trigger logic is based on the original C implementation in `Code_for_C_imp/main.c`:

### Right Hand (sensor_select = 0):
- **Yaw 20-120°**: Snare drum (gyro_y < -2500)
- **Yaw 340-20°**: High tom or Crash cymbal (pitch > 50°)
- **Yaw 305-340°**: Mid tom or Ride cymbal (pitch > 50°)
- **Yaw 200-305°**: Floor tom or Ride cymbal (pitch > 30°)

### Left Hand (sensor_select = 1):
- **Yaw 350-100°**: Snare or Hi-hat (pitch > 30° and gyro_z > -2000)
- **Yaw 325-350°**: High tom or Crash cymbal (pitch > 50°)
- **Yaw 300-325°**: Mid tom or Ride cymbal (pitch > 50°)
- **Yaw 200-300°**: Floor tom or Ride cymbal (pitch > 30°)

## Timing Considerations

### SPI Timing (from BNO08X datasheet):
- Maximum SPI clock: 3MHz ✓ (our clock is 3MHz)
- CS setup to CLK: 0.1μs min
- CS hold: 16.83ns min
- CLK to MISO valid: 35ns max
- MOSI setup: 25ns min
- MOSI hold: 5.4ns min

### Interrupt Timing:
- H_INTN assertion indicates data ready
- Host should respond within 10ms to avoid timeout
- Recommended: respond within 1/10 of fastest sensor period

## Implementation Notes

1. **Fixed-Point Arithmetic**: The implementation uses fixed-point arithmetic (Q30 for quaternions, Q16 for angles). Ensure proper scaling in calculations.

2. **Quaternion to Euler Conversion**: The current implementation uses simplified approximations. For production, implement:
   - CORDIC algorithm for atan2 and asin
   - Lookup tables for faster computation
   - Proper quadrant handling

3. **Dual Sensor Support**: The system supports two sensors (left/right hand). Use `sensor_select` to switch between them.

4. **Yaw Offset**: The system supports yaw offset calibration. Set `set_yaw_offset` to capture current yaw as the zero reference.

5. **Error Handling**: The `error` signal indicates communication or initialization failures. Monitor this signal and implement retry logic if needed.

## Testing and Verification

1. **Initialization**: Verify `initialized` signal goes high after power-on
2. **SPI Communication**: Monitor SPI signals with logic analyzer
3. **Sensor Data**: Check `data_ready` and sensor data outputs
4. **Drum Triggers**: Verify trigger outputs match expected orientation zones

## References

- BNO08X Datasheet (Revision 1.17)
- Adafruit BNO08x Library
- SH-2 Reference Manual
- iCE40 UltraPlus Family Data Sheet

## License

This implementation is based on:
- Adafruit BNO08x library (BSD license)
- SH-2 library (Apache License 2.0)
- Original C implementation in `Code_for_C_imp`

