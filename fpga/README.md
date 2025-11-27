# FPGA Code for Drum Trigger System

This folder contains all SystemVerilog code needed to flash the FPGA.

## Top-Level Module

**`drum_trigger_top_integrated.sv`** - Main top-level module to flash to FPGA

## Required Files

All files in this folder are required for synthesis:

1. **`drum_trigger_top_integrated.sv`** - Top-level module (flash this)
2. **`drum_spi_slave.sv`** - SPI slave for MCU communication
3. **`drum_trigger_processor.sv`** - Dual sensor drum trigger processing
4. **`bno085_controller.sv`** - BNO085 sensor controller (needed x2 for dual sensors)
5. **`spi_master.sv`** - SPI master for BNO085 communication (needed x2)
6. **`quaternion_to_euler_dsp.sv`** - Quaternion to Euler conversion (needed x2)
7. **`yaw_normalizer.sv`** - Yaw normalization with calibration (needed x2)
8. **`drum_zone_detector.sv`** - Zone detection based on yaw (needed x2)
9. **`strike_detector.sv`** - Strike detection from gyroscope (needed x2)
10. **`drum_selector.sv`** - Final drum code selection (needed x2)
11. **`button_debouncer.sv`** - Button debouncing (needed x2)

## Pin Assignments

### MCU SPI Interface
- `mcu_sck` - SPI clock from MCU (PB3)
- `mcu_sdi` - SPI data in from MCU (PB5 MOSI)
- `mcu_sdo` - SPI data out to MCU (PB4 MISO)
- `mcu_load` - Load signal from MCU (PA5)
- `mcu_done` - Done signal to MCU (PA6)

### BNO085 Sensor 1 (Right Hand)
- `sclk` - Shared SPI clock
- `mosi` - Shared SPI MOSI
- `miso1` - Sensor 1 MISO
- `cs_n1` - Sensor 1 chip select
- `int1` - Sensor 1 interrupt

### BNO085 Sensor 2 (Left Hand)
- `sclk` - Shared SPI clock (same as sensor 1)
- `mosi` - Shared SPI MOSI (same as sensor 1)
- `miso2` - Sensor 2 MISO
- `cs_n2` - Sensor 2 chip select
- `int2` - Sensor 2 interrupt

### Shared BNO085 Control
- `bno085_rst_n` - Shared reset (both sensors)

### Buttons
- `calibrate_btn_n` - Calibration button (active low)
- `kick_btn_n` - Kick drum button (active low)

### Status LEDs
- `led_initialized` - Both sensors initialized
- `led_error` - Error from either sensor
- `led_heartbeat` - System heartbeat

## Clock

- System clock: 3MHz (from HSOSC)
- SPI clock: Driven by MCU (5MHz)

## Dependencies

All modules are self-contained. No external IP cores required except:
- HSOSC primitive (built into Lattice iCE40)

## Synthesis Notes

1. Add all `.sv` files to your synthesis project
2. Set `drum_trigger_top_integrated` as top-level module
3. Assign pins according to pin assignments above
4. Clock constraint: 3MHz system clock

