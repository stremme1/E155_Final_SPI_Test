# FPGA Files List

## Top-Level Module (Flash This)
- **`drum_trigger_top_integrated.sv`** - Main top-level module

## Core Modules
- **`drum_spi_slave.sv`** - SPI slave for MCU communication
- **`drum_trigger_processor.sv`** - Dual sensor drum trigger processing

## BNO085 Interface
- **`bno085_controller.sv`** - BNO085 sensor controller (used x2)
- **`spi_master.sv`** - SPI master for BNO085 (used x2)

## Signal Processing
- **`quaternion_to_euler_dsp.sv`** - Quaternion to Euler conversion (used x2)
- **`yaw_normalizer.sv`** - Yaw normalization with calibration (used x2)
- **`drum_zone_detector.sv`** - Zone detection based on yaw (used x2)
- **`strike_detector.sv`** - Strike detection from gyroscope (used x2)
- **`drum_selector.sv`** - Final drum code selection (used x2)

## Utilities
- **`button_debouncer.sv`** - Button debouncing (used x2)

## Total: 11 SystemVerilog files

All files are required for synthesis. Add all `.sv` files to your FPGA project.

