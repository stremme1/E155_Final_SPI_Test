# Testbench Documentation

## Overview

The comprehensive testbench `tb_dual_sensor_mcu.sv` tests the complete dual BNO085 sensor system with MCU SPI integration.

## Test Coverage

### 1. Dual Sensor Initialization
- Tests initialization of both BNO085 sensors
- Verifies Product ID requests
- Verifies Rotation Vector and Gyroscope feature enables
- Checks that both sensors complete initialization before proceeding

### 2. Sensor Data Reading
- **Sensor 1**: Tests quaternion and gyroscope data reception
- **Sensor 2**: Tests quaternion and gyroscope data reception
- Verifies data validity signals
- Checks data values match expected test vectors

### 3. Data Ready Signal
- Verifies `data_ready` signal is asserted when both sensors have new data
- Tests synchronization between sensors

### 4. MCU SPI Communication
- Tests DONE/LOAD handshake protocol (Lab07 pattern)
- Simulates MCU SPI master (Mode 0: CPOL=0, CPHA=0)
- Verifies 29-byte packet transmission
- Tests byte-by-byte data transfer

### 5. Data Packet Verification
- Verifies packet header (0xAA)
- Checks Sensor 1 data format (14 bytes: quat + gyro)
- Checks Sensor 2 data format (14 bytes: quat + gyro)
- Verifies little-endian byte ordering
- Compares received data with expected values

### 6. Multiple Packet Transmission
- Tests sending multiple data packets sequentially
- Verifies system can handle continuous data flow

### 7. Status LEDs
- Verifies initialization LED
- Verifies error LED (should be OFF)
- Verifies heartbeat LED

## Running the Testbench

### Using Icarus Verilog (iverilog)

```bash
# Compile
iverilog -o dual_sensor_test \
    -s tb_dual_sensor_mcu \
    tb_dual_sensor_mcu.sv \
    drum_trigger_top.sv \
    dual_bno085_controller.sv \
    bno085_controller.sv \
    sensor_data_formatter.sv \
    spi_slave_mcu.sv \
    spi_master.sv \
    mock_bno085.sv

# Run
vvp dual_sensor_test

# View waveform (if GTKWave is installed)
gtkwave dual_sensor_mcu_test.vcd
```

### Using Verilator

```bash
verilator --cc --exe --build \
    tb_dual_sensor_mcu.sv \
    drum_trigger_top.sv \
    dual_bno085_controller.sv \
    bno085_controller.sv \
    sensor_data_formatter.sv \
    spi_slave_mcu.sv \
    spi_master.sv \
    mock_bno085.sv \
    --top-module tb_dual_sensor_mcu
```

### Using ModelSim/QuestaSim

```tcl
# In ModelSim/QuestaSim
vlog -work work *.sv
vsim -voptargs=+acc work.tb_dual_sensor_mcu
add wave -radix hex /tb_dual_sensor_mcu/*
run -all
```

## Test Vectors

### Sensor 1 Test Data
- **Quaternion**: W=0x4000, X=100, Y=200, Z=300
- **Gyroscope**: X=1000, Y=2000, Z=3000

### Sensor 2 Test Data
- **Quaternion**: W=0x5000, X=400, Y=500, Z=600
- **Gyroscope**: X=4000, Y=5000, Z=6000

### Second Packet Test Data
- **Sensor 1**: Quat W=0x6000, X=1000, Y=2000, Z=3000; Gyro X=5000, Y=6000, Z=7000
- **Sensor 2**: Quat W=0x7000, X=4000, Y=5000, Z=6000; Gyro X=8000, Y=9000, Z=10000

## Expected Output

The testbench will display:
- Test step progress
- Pass/fail status for each test
- Sensor data values
- MCU packet contents
- Final test summary

Example output:
```
========================================
Dual BNO085 + MCU SPI Integration Test
========================================

[TEST] Step 1: System Reset
[TEST] Reset released at 1000

[TEST] Step 2: Waiting for Dual Sensor Initialization
[PASS] Both Sensors Initialized!
  - LED status: 1
  - Sensor 1 commands: 3
  - Sensor 2 commands: 3
[PASS] All initialization commands completed

[TEST] Step 3: Testing Sensor 1 Data Reading
[PASS] Sensor 1 Quaternion Received!
[PASS] Sensor 1 Gyroscope Received!

[TEST] Step 4: Testing Sensor 2 Data Reading
[PASS] Sensor 2 Quaternion Received!
[PASS] Sensor 2 Gyroscope Received!

[TEST] Step 5: Testing Data Ready Signal
[PASS] Data ready signal asserted

[TEST] Step 6: Testing MCU SPI Communication
[MCU] Waiting for DONE signal...
[MCU] DONE signal received, starting SPI transaction
[MCU] Received byte 0: 0xaa
[MCU] Received byte 1: 0x00
...
[MCU] Packet read complete

[TEST] Step 7: Verifying Packet Contents
[PASS] Packet header correct: 0xaa
[PASS] Sensor 1 data matches expected values
[PASS] Sensor 2 data matches expected values

[TEST] Step 8: Testing Multiple Packet Transmission
[PASS] Second packet received successfully

[TEST] Step 9: Final Status Check
[PASS] Error LED is OFF
[PASS] Initialized LED is ON

========================================
All Tests Completed Successfully!
========================================
```

## Debugging

### Common Issues

1. **Initialization Timeout**
   - Check that mock sensors are responding correctly
   - Verify PS0/WAKE signals are working
   - Check INT signal assertions

2. **Data Mismatch**
   - Verify byte ordering (little-endian)
   - Check that sensor data is being captured correctly
   - Verify formatter is packaging data correctly

3. **SPI Communication Issues**
   - Verify SPI Mode 0 timing (CPOL=0, CPHA=0)
   - Check DONE/LOAD handshake timing
   - Verify CS signal timing

### Signal Monitoring

Key signals to monitor:
- `done` - FPGA asserts when data ready
- `load` - MCU toggles to acknowledge
- `cs_n_mcu` - MCU chip select
- `miso_mcu` - Data from FPGA to MCU
- `sclk_mcu` - SPI clock from MCU
- `int_n1`, `int_n2` - Sensor interrupt signals
- `data_ready` - Both sensors have data

## Waveform Analysis

The testbench generates a VCD file (`dual_sensor_mcu_test.vcd`) that can be viewed in GTKWave or other waveform viewers.

Key signals to examine:
1. **Initialization Phase**: Watch both sensors initialize in parallel
2. **Data Reading Phase**: Monitor sensor data capture
3. **MCU Communication Phase**: Watch DONE/LOAD handshake and SPI transaction
4. **Packet Format**: Verify 29-byte packet structure

## Notes

- The testbench uses delay acceleration to speed up simulation
- Mock sensors simulate realistic BNO085 behavior
- MCU SPI master simulates Mode 0 (CPOL=0, CPHA=0)
- All timing follows Lab07 SPI protocol patterns

