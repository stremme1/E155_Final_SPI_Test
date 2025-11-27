# Simplified FPGA Design Summary

## Problem Solved
- **Original Design**: Used 18 DSP blocks (2x quaternion_to_euler_dsp with ~9 DSP each)
- **FPGA Limitation**: Only 8 DSP blocks available
- **Solution**: Move all processing to MCU, FPGA only reads sensors and sends raw data

## New Architecture

### FPGA (Simplified - 0 DSP blocks)
1. **bno085_controller** (2x) - Read quaternion and gyro data from sensors
2. **spi_master** (2x) - BNO085 SPI communication
3. **sensor_data_packer** - Pack raw sensor data into 32-byte packets
4. **sensor_data_spi_slave** - Send data packets to MCU via SPI
5. **button_debouncer** (2x) - Debounce button inputs

### MCU (Enhanced)
1. **SPI Master** - Receive 32-byte sensor data packets from FPGA
2. **Drum Processing** - Use existing C code from `Code_for_C_imp/main.c`:
   - Quaternion to Euler conversion (software)
   - Yaw normalization
   - Zone detection
   - Strike detection
   - Drum selection
3. **DAC Playback** - Play drum sounds (already implemented)

## Resource Savings

### Removed from FPGA (saves 18 DSP blocks):
- ❌ `quaternion_to_euler_dsp` (18 DSP blocks)
- ❌ `yaw_normalizer` (pipeline stages)
- ❌ `drum_zone_detector` (comparison logic)
- ❌ `strike_detector` (simple)
- ❌ `drum_selector` (simple)
- ❌ `drum_trigger_processor` (orchestration)

### Kept in FPGA:
- ✅ `bno085_controller` (2x) - Required to read sensors
- ✅ `spi_master` (2x) - Required for BNO085 communication
- ✅ `sensor_data_packer` - Simple combinational logic (no DSP)
- ✅ `sensor_data_spi_slave` - SPI communication (no DSP)
- ✅ `button_debouncer` (2x) - Simple FSM (no DSP)

## Data Format

### 32-Byte Sensor Data Packet (FPGA → MCU)
```
Byte 0-1:   Sensor 1 Quaternion W (16-bit signed, MSB first)
Byte 2-3:   Sensor 1 Quaternion X (16-bit signed, MSB first)
Byte 4-5:   Sensor 1 Quaternion Y (16-bit signed, MSB first)
Byte 6-7:   Sensor 1 Quaternion Z (16-bit signed, MSB first)
Byte 8-9:   Sensor 1 Gyroscope X (16-bit signed, MSB first)
Byte 10-11: Sensor 1 Gyroscope Y (16-bit signed, MSB first)
Byte 12-13: Sensor 1 Gyroscope Z (16-bit signed, MSB first)
Byte 14:    Sensor 1 Flags (bit 0=quat_valid, bit 1=gyro_valid)
Byte 15-22: Sensor 2 Quaternion (same format as Sensor 1)
Byte 23-28: Sensor 2 Gyroscope (same format as Sensor 1)
Byte 29:    Sensor 2 Flags (bit 0=quat_valid, bit 1=gyro_valid)
Byte 30:    Buttons (bit 0=kick_btn)
Byte 31:    Buttons (bit 0=calibrate_btn)
```

## SPI Protocol

### Handshaking (same as Lab07):
1. MCU pulls LOAD low (request data)
2. FPGA asserts DONE when data is ready
3. MCU reads 32 bytes via SPI (256 clock cycles)
4. MCU pulls LOAD high to acknowledge
5. FPGA deasserts DONE

### SPI Mode:
- CPOL=0, CPHA=0 (SPI Mode 0)
- Data sampled on rising edge
- MSB first

## Files Created

### FPGA:
1. `drum_trigger_top_simplified.sv` - Simplified top-level module
2. `sensor_data_packer.sv` - Packs sensor data into bytes
3. `sensor_data_spi_slave.sv` - Sends multi-byte packets to MCU
4. `tb_drum_trigger_top_simplified.sv` - Testbench

### MCU:
1. `main_sensor_data.c` - Receives sensor data and processes it

## Testing

### Testbench:
- Tests 32-byte packet structure
- Tests button detection
- Verifies SPI communication

### Compilation:
- All modules compile individually ✅
- HSOSC is Lattice primitive (expected in synthesis) ✅
- No multiple driver errors ✅
- Ready for FPGA synthesis ✅

## Next Steps

1. **Synthesize** `drum_trigger_top_simplified.sv` in Lattice Radiant
2. **Flash MCU** with `main_sensor_data.c`
3. **Test** end-to-end system
4. **Verify** drum sounds play correctly

## Benefits

1. **Resource Efficient**: 0 DSP blocks (vs 18 before)
2. **Flexible**: Processing logic in software (easier to modify)
3. **Testable**: Can test FPGA and MCU independently
4. **Maintainable**: Reuses existing C code logic

