# BNO085 FPGA Implementation

## Overview

This directory contains a professional FPGA implementation of the Adafruit BNO08x library for the iCE40 UltraPlus FPGA family. The implementation translates the C++ Arduino library logic into SystemVerilog, maintaining protocol compatibility with the original Adafruit library.

## Architecture

### Clock System

The design uses the iCE40 UltraPlus built-in **HSOSC (High-Speed Oscillator)** primitive:

- **HSOSC Frequency**: 48MHz (nominal)
- **Division Factor**: 16 (CLKHF_DIV = 2'b11)
- **System Clock**: 3MHz
- **SPI Clock**: ~750kHz (derived from 3MHz system clock)

**Clock Configuration:**
```systemverilog
HSOSC #(.CLKHF_DIV(2'b11)) hf_osc (
    .CLKHFPU(1'b1),   // Power up
    .CLKHFEN(1'b1),   // Enable
    .CLKHF(clk)       // Output: 3MHz (48MHz / 16)
);
```

**Reference**: [iCE40 UltraPlus Family Data Sheet](https://hmc-e155.github.io/assets/doc/FPGA-DS-02008-2-0-iCE40-UltraPlus-Family-Data-Sheet.pdf), Section 3.1.10

### SPI Configuration

Per the Adafruit BNO08x library (`Adafruit_BNO08x.cpp` line 167-169):

- **SPI Mode**: Mode 3 (CPOL=1, CPHA=1)
- **Bit Order**: MSB first
- **Frequency**: 1MHz maximum (BNO085 specification)
- **Current Implementation**: ~750kHz (within spec)

### Protocol Implementation

The implementation follows the **SHTP (Sensor Hub Transport Protocol)** as used in the Adafruit library:

#### SPI HAL Functions (from Adafruit library)

1. **`spihal_wait_for_int()`** (lines 549-560)
   - Waits for INT pin to go low (up to 500ms timeout)
   - Indicates sensor has data ready

2. **`spihal_read()`** (lines 566-606)
   - Waits for INT
   - Reads 4-byte SHTP header
   - Extracts packet length (masking continuation bit)
   - Reads full packet payload

3. **`spihal_write()`** (lines 608-619)
   - Waits for INT
   - Writes SHTP packet

#### SHTP Packet Format

```
[Length LSB] [Length MSB] [Channel] [Sequence] [Payload...]
     Byte 0      Byte 1     Byte 2     Byte 3
```

- **Length**: 16-bit little-endian (bit 15 = continuation bit, must be masked)
- **Channel**: SHTP channel number (0x02 = Control, 0x03 = Reports, 0x05 = Gyro RV)
- **Sequence**: Packet sequence number
- **Payload**: Variable length based on report type

## Module Hierarchy

```
bno085_fpga_top
├── HSOSC (clock generation)
├── spi_master (SPI Mode 3 implementation)
└── bno085_controller (SHTP protocol handler)
    ├── Initialization state machine
    ├── Command ROM (Product ID, Set Feature)
    └── Report parser (Rotation Vector, Gyroscope)
```

## Files

- **`bno085_fpga_top.sv`**: Top-level module with HSOSC clock configuration
- **`README.md`**: This file

## Dependencies

This implementation requires the following modules (from parent directory):

- `spi_master.sv`: SPI Mode 3 master implementation
- `bno085_controller.sv`: SHTP protocol controller

## Initialization Sequence

1. **Reset Wait**: 100ms delay after system reset
2. **Wake Sensor**: Drive PS0/WAKE pin low
3. **Wait for INT**: Sensor asserts INT when ready (max 150µs per datasheet)
4. **Product ID Request**: Query sensor identity
5. **Enable Rotation Vector**: Configure 50Hz rotation vector reports
6. **Enable Gyroscope**: Configure 50Hz gyroscope reports
7. **Normal Operation**: Poll INT pin and read sensor reports

## Sensor Reports

### Rotation Vector (Quaternion)
- **Report ID**: 0x05
- **Channel**: 0x05 (Gyro Rotation Vector) or 0x03 (Standard Reports)
- **Data Format**: 16-bit signed integers (Q14 fixed-point)
- **Components**: W, X, Y, Z

### Gyroscope
- **Report ID**: 0x02 (Calibrated Gyroscope)
- **Channel**: 0x03 (Standard Reports)
- **Data Format**: 16-bit signed integers
- **Components**: X, Y, Z (angular velocity)

## Pin Requirements

### Required Pull-ups

1. **FPGA Reset (`rst_n`)**: 10kΩ external pull-up to VCC
2. **BNO085 INT (`int_n`)**: FPGA internal pull-up or 4.7kΩ-10kΩ external
3. **BNO085 RST**: 10kΩ pull-up to VCC (hardware, not FPGA pin)

### Pin Assignments

Refer to your project's `PIN_CONFIGURATION.md` for specific pin mappings.

## Timing Considerations

### SPI Timing (per BNO085 datasheet)

- **tCSSU** (CS setup): 0.1µs minimum
- **tCSH** (CS hold): 0.1µs minimum
- **tWK** (Wake pulse): 150µs maximum
- **SPI Clock**: 1MHz maximum

### System Timing

- **System Clock**: 3MHz (333ns period)
- **SPI Clock**: ~750kHz (1.33µs period)
- **INT Wait Timeout**: 200µs (600 cycles @ 3MHz)

## Verification

This implementation has been verified against:

1. **Adafruit BNO08x Library**: Protocol compatibility
2. **BNO085 Datasheet**: Timing and electrical specifications
3. **iCE40 UltraPlus Data Sheet**: Clock and I/O specifications

## References

1. [Adafruit BNO08x Library](https://github.com/adafruit/Adafruit_BNO08x)
2. [iCE40 UltraPlus Family Data Sheet](https://hmc-e155.github.io/assets/doc/FPGA-DS-02008-2-0-iCE40-UltraPlus-Family-Data-Sheet.pdf)
3. BNO085 Datasheet (Hillcrest Laboratories)

## License

This implementation is based on the Adafruit BNO08x library, which uses the BSD license. Please refer to the original library's license file for details.

