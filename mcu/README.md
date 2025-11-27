# MCU Code for Drum Trigger System

This folder contains all C code and dependencies needed to flash the STM32L432KC MCU.

## Main Application

**`main_integrated.c`** - Main application file (flash this)

## Required Files

### Core Application
- **`main_integrated.c`** - Main application (SPI communication + drum playback)

### STM32L432KC Libraries
- **`STM32L432KC_SPI.c/.h`** - SPI communication with FPGA
- **`STM32L432KC_GPIO.c/.h`** - GPIO control (LOAD, DONE pins)
- **`STM32L432KC_DAC.c/.h`** - DAC audio output
- **`STM32L432KC_TIMER.c/.h`** - Timer functions (ms_delay)
- **`STM32L432KC_RCC.c/.h`** - Clock configuration
- **`STM32L432KC_FLASH.c/.h`** - Flash latency configuration

### Drum Samples
- **`wav_arrays/`** - Folder containing all drum sample arrays
  - `drum_samples.h` - Header with all sample declarations
  - `*.c` files - Individual sample data arrays

### STM32 Device Support
- **`STM32L4xx/`** - STM32L4 device headers and startup code
- **`CMSIS_5/`** - ARM CMSIS headers
- **`SEGGER_THUMB_Startup.s`** - Startup assembly code
- **`STM32L4xx_Flash.icf`** - Linker script

## Pin Assignments

### SPI Interface to FPGA
- **PB3** - SCK (SPI clock)
- **PB4** - MISO (SPI data in from FPGA)
- **PB5** - MOSI (SPI data out to FPGA)
- **PA11** - CE (Chip select, for debugging)

### Handshaking Signals
- **PA5** - LOAD (Output to FPGA - acknowledge)
- **PA6** - DONE (Input from FPGA - data ready)

### Audio Output
- **PA4** - DAC Channel 1 (Audio output)

## Configuration

### SPI Settings
- Mode: CPOL=0, CPHA=0 (SPI Mode 0)
- Baud Rate: 5MHz (br=3, PCLK/16)
- Blocking wait for DONE signal (Lab07 pattern)

### Drum Sample Rate
- All samples: 22.05 kHz
- Defined in `main_integrated.c`: `DRUM_SAMPLE_RATE 22050`

## Build Instructions

1. Create new STM32 project in your IDE (e.g., SEGGER Embedded Studio)
2. Add all `.c` files to project
3. Add all `.h` files to include paths
4. Add `STM32L4xx/`, `CMSIS_5/` to include paths
5. Add `wav_arrays/` to include paths
6. Set `main_integrated.c` as main file
7. Use linker script: `STM32L4xx_Flash.icf`
8. Build and flash to STM32L432KC

## Drum Code Mapping

| Code | Drum Type | Sample File |
|------|-----------|-------------|
| 0x00 | Snare | `snare_sample.c` |
| 0x01 | Hi-hat | `hihat_closed_sample.c` |
| 0x02 | Kick | `kick_sample.c` |
| 0x03 | High tom | `tom_high_sample.c` |
| 0x04 | Mid tom | `tom_high_sample.c` |
| 0x05 | Crash | `crash_sample.c` |
| 0x06 | Ride | `ride_sample.c` |
| 0x07 | Floor tom | `tom_low_sample.c` |

## System Behavior

1. MCU initializes SPI and DAC
2. MCU enters blocking loop: `readDrumCommand()`
3. When FPGA asserts DONE, MCU reads drum code via SPI
4. MCU acknowledges with LOAD signal
5. MCU plays corresponding drum sample via DAC
6. Loop repeats

## Dependencies

- STM32L432KC device headers
- ARM CMSIS headers
- All drum sample arrays must be compiled

