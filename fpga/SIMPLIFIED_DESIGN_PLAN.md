# Simplified FPGA Design Plan

## Problem
- Current design uses 18 DSP blocks (2x quaternion_to_euler_dsp with ~9 DSP each)
- FPGA only has 8 DSP blocks available
- Design synthesizes but doesn't map to device

## Solution: Move Processing to MCU
- **FPGA**: Just read BNO085 sensors and send raw data to MCU via SPI
- **MCU**: Do all processing (quaternion→euler, zone detection, drum selection) using existing C code

## New Architecture

### FPGA (Simplified)
1. **BNO085 Controllers** (2x) - Read quaternion and gyro data
2. **SPI Master** (2x) - For BNO085 communication
3. **Data Packer** - Pack sensor data into SPI packets
4. **SPI Slave** - Send data packets to MCU

### MCU (Enhanced)
1. **SPI Master** - Receive sensor data from FPGA
2. **Drum Processing** - Use existing C code from `Code_for_C_imp`
3. **DAC Playback** - Play drum sounds (already implemented)

## Data Format

### Sensor Data Packet (to MCU)
- **Sensor 1 (Right Hand)**:
  - Quaternion: w, x, y, z (4x 16-bit signed)
  - Gyroscope: x, y, z (3x 16-bit signed)
  - Valid flags: quat_valid, gyro_valid
- **Sensor 2 (Left Hand)**:
  - Quaternion: w, x, y, z (4x 16-bit signed)
  - Gyroscope: x, y, z (3x 16-bit signed)
  - Valid flags: quat_valid, gyro_valid
- **Buttons**: calibrate_btn, kick_btn

### SPI Protocol
- MCU requests data by pulling LOAD low
- FPGA sends data packet (multiple bytes)
- Use existing SPI slave pattern from Lab07

## Resource Savings
- **Remove**: quaternion_to_euler_dsp (18 DSP blocks)
- **Remove**: yaw_normalizer (pipeline stages)
- **Remove**: drum_zone_detector (comparison logic)
- **Remove**: strike_detector (simple)
- **Remove**: drum_selector (simple)
- **Keep**: bno085_controller (2x) - needed to read sensors
- **Keep**: spi_master (2x) - needed for BNO085
- **Keep**: spi_slave - needed for MCU communication
- **Add**: data_packer - simple combinational logic

## Implementation Steps
1. Create simplified top module that reads sensors and packs data
2. Update SPI slave to send multi-byte data packets
3. Update MCU code to receive and process sensor data
4. Create testbench for simplified design

