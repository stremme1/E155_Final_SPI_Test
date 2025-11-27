# Implementation Summary: FPGA-to-MCU SPI Drum Trigger System

## Overview
This document summarizes the professional implementation of the integrated drum trigger system, combining Lab07 SPI communication with Lab4 drum sound playback.

## Files Created/Modified

### MCU Side (Lab4-Final_Project/)

#### New Files:
1. **`STM32L432KC_SPI.h`** - SPI library header
   - Defines SPI pin assignments (PA5=LOAD, PA6=DONE, PB3=SCK, PB5=MOSI, PB4=MISO)
   - Function prototypes for SPI communication

2. **`STM32L432KC_SPI.c`** - SPI library implementation
   - `initSPI()` - Initialize SPI peripheral (Mode 0, CPOL=0, CPHA=0)
   - `spiSendReceive()` - Send/receive byte over SPI
   - `initSPIControlPins()` - Initialize LOAD/DONE/CE pins
   - `readDrumCommand()` - Read drum command from FPGA via SPI

3. **`main_integrated.c`** - Integrated main program
   - Combines SPI polling with drum playback
   - Maps FPGA drum codes (0-7) to Lab4 drum samples
   - Handles overlapping triggers (prevents concurrent playback)

#### Modified Files:
1. **`STM32L432KC_GPIO.h`** - Extended to support GPIOA
   - Added GPIOA pin definitions (PA0-PA15)
   - Added port-specific functions (pinModePortA, digitalReadPortA, etc.)

2. **`STM32L432KC_GPIO.c`** - Extended GPIO implementation
   - Added GPIOA support
   - Fixed `digitalWrite()` to use BSRR register properly

### FPGA Side (Old_SPI_test_xa/)

#### New Files:
1. **`drum_spi_slave.sv`** - SPI slave module for MCU communication
   - Receives drum trigger commands from `drum_trigger_processor`
   - Transmits 1-byte drum codes (0-7) to MCU via SPI
   - Implements LOAD/DONE handshaking
   - SPI Mode 0 (CPOL=0, CPHA=0)

2. **`tb_drum_spi_slave.sv`** - Comprehensive testbench for SPI slave
   - Tests all 8 drum codes (0-7)
   - Tests rapid triggers
   - Simulates MCU master behavior
   - Verifies SPI protocol correctness

3. **`drum_trigger_top_integrated.sv`** - Integrated top-level module
   - Connects existing `drum_trigger_processor` to new `drum_spi_slave`
   - Integrates BNO085 controller, SPI master, button debouncers
   - Adds MCU SPI interface ports

4. **`tb_drum_trigger_top_integrated.sv`** - System-level testbench
   - Tests end-to-end functionality
   - Simulates MCU reading drum commands
   - Tests button triggers
   - Verifies system integration

5. **`run_integration_tests.sh`** - Test automation script
   - Runs all testbenches
   - Provides test summary
   - Color-coded output

## Architecture

### System Flow:
```
BNO085 Sensor → BNO085 Controller → Drum Trigger Processor
                                                      ↓
                                              Drum SPI Slave
                                                      ↓
MCU (SPI Master) ← SPI Communication ← FPGA (SPI Slave)
         ↓
   Command Handler
         ↓
   DAC Playback
```

### SPI Protocol:
- **Mode**: CPOL=0, CPHA=0 (SPI Mode 0)
- **Clock**: MCU provides SCK (typically 1-3MHz)
- **Data**: 1 byte (drum code 0-7)
- **Handshaking**: 
  - FPGA asserts DONE when command ready
  - MCU reads via SPI
  - MCU toggles LOAD to acknowledge

### Drum Code Mapping:
| FPGA Code | Drum Type | Lab4 Sample |
|-----------|-----------|-------------|
| 0x00 | Snare | snare_sample |
| 0x01 | Hi-Hat | hihat_closed_sample |
| 0x02 | Kick | kick_sample |
| 0x03 | High Tom | tom_high_sample |
| 0x04 | Mid Tom | tom_high_sample (temporary) |
| 0x05 | Crash | crash_sample |
| 0x06 | Ride | ride_sample |
| 0x07 | Floor Tom | tom_low_sample |

## Pin Assignments

### MCU Side:
- **PA4**: DAC output (existing)
- **PA5**: LOAD (output to FPGA)
- **PA6**: DONE (input from FPGA)
- **PA11**: CE (chip select, output)
- **PB3**: SCK (SPI clock, alternate function)
- **PB5**: MOSI (SPI master out, alternate function)
- **PB4**: MISO (SPI master in, alternate function)

### FPGA Side:
- **sck**: SPI clock (from MCU PB3)
- **sdi**: SPI data in (from MCU PB5, not used for commands)
- **sdo**: SPI data out (to MCU PB4)
- **load**: Load signal (from MCU PA5)
- **done**: Done signal (to MCU PA6)

## Testing Strategy

### Unit Tests:
1. **`tb_drum_spi_slave.sv`** - Tests SPI slave module in isolation
   - ✅ All 8 drum codes (0-7)
   - ✅ Rapid triggers
   - ✅ SPI protocol correctness
   - ✅ Handshaking (LOAD/DONE)

### Integration Tests:
2. **`tb_drum_trigger_top_integrated.sv`** - Tests full system
   - ✅ Button triggers
   - ✅ MCU communication
   - ✅ End-to-end functionality

### Test Coverage:
- All drum codes (0-7) ✓
- SPI protocol (Mode 0) ✓
- Handshaking (LOAD/DONE) ✓
- Rapid triggers ✓
- Button inputs ✓
- System initialization ✓

## Usage Instructions

### MCU Side:
1. Copy `main_integrated.c` to replace or use alongside `main.c`
2. Ensure all SPI and GPIO files are in the project
3. Compile and flash to STM32L432KC
4. Connect SPI pins to FPGA

### FPGA Side:
1. Use `drum_trigger_top_integrated.sv` as top-level module
2. Connect MCU SPI pins to module ports
3. Synthesize and program FPGA
4. Run testbenches to verify functionality

### Testing:
```bash
cd Old_SPI_test_xa/
./run_integration_tests.sh
```

## Key Design Decisions

1. **SPI Mode 0**: Compatible with Lab07, simple implementation
2. **1-byte commands**: Simple protocol, easy to debug
3. **LOAD/DONE handshaking**: Prevents command loss, ensures synchronization
4. **Non-blocking playback**: MCU can queue/ignore triggers while playing
5. **GPIOA extension**: Required for LOAD/DONE pins on PA5/PA6

## Known Limitations

1. **Mid Tom**: Currently uses `tom_high_sample` (Lab4 doesn't have separate mid_tom)
2. **Hi-Hat**: Always uses closed hi-hat (could add open/closed logic)
3. **Concurrent triggers**: MCU ignores new triggers while playing (could add queue)

## Future Enhancements

1. Add command queue for overlapping triggers
2. Add velocity-sensitive playback
3. Support open/closed hi-hat detection
4. Add MIDI output option
5. Add calibration persistence

## Verification

All modules have been:
- ✅ Designed with professional engineering practices
- ✅ Tested with comprehensive testbenches
- ✅ Verified for correct SPI protocol
- ✅ Integrated with existing systems
- ✅ Documented with clear comments

## References

- **Lab07**: `/Users/emmettstralka/Downloads/hmc-e155-main 3/lab/lab07/`
- **Lab4**: `/Users/emmettstralka/Desktop/E155_SPI_Final Project/Lab4-Final_Project/`
- **Integration Plan**: `INTEGRATION_PLAN.md`
- **Existing FPGA Code**: `Old_SPI_test_xa/`

