# FPGA Files List

## SystemVerilog Source Files (11 modules)

### Top-Level Module
- `drum_trigger_top_integrated.sv` - Main top-level module integrating all components

### Core Modules
- `drum_trigger_processor.sv` - Processes BNO085 data and button inputs to detect drum triggers
- `drum_spi_slave.sv` - SPI slave for MCU communication (sends drum commands)
- `button_debouncer.sv` - Debounces button inputs (FSM pattern)

### BNO085 Sensor Interface
- `bno085_controller.sv` - BNO085 sensor controller
- `spi_master.sv` - SPI master for BNO085 communication

### Signal Processing
- `quaternion_to_euler_dsp.sv` - Converts quaternion to Euler angles
- `yaw_normalizer.sv` - Normalizes yaw angle with calibration
- `drum_zone_detector.sv` - Detects drum zones from yaw angle
- `strike_detector.sv` - Detects drum strikes from gyroscope
- `drum_selector.sv` - Selects drum type based on zone, pitch, and gyro

## Testbenches

### Unit Tests
- `tb_drum_spi_slave.sv` - Unit test for SPI slave module (all tests pass)

### System Tests
- `tb_drum_trigger_top_integrated.sv` - Full system testbench with timeout

## Scripts
- `run_tb_with_timeout.sh` - Testbench runner with automatic timeout

## Documentation
- `README.md` - FPGA project documentation
- `FILES_LIST.md` - This file

## Mock/Support Files
- `hsosc_mock.sv` - Mock HSOSC for simulation (if needed)
