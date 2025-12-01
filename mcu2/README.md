# MCU2 - BNO085 Drum Trigger System

This directory contains all files needed to flash and run the MCU-only BNO085 drum trigger system.

## Project Structure

### Core Application Files
- `main.c` - Main application entry point
- `bno085_driver.c/h` - BNO085 sensor driver (SHTP protocol implementation)
- `bno085_decoder.c/h` - Quaternion to Euler angle conversion
- `drum_detector.c/h` - Drum trigger detection logic (matches original code exactly)
- `audio_player.c/h` - Audio playback handler
- `debug_print.c/h` - Debug UART printing utilities

### STM32 Hardware Abstraction Layer
- `STM32L432KC_RCC.c/h` - Clock configuration
- `STM32L432KC_GPIO.c/h` - GPIO control
- `STM32L432KC_FLASH.c/h` - Flash configuration
- `STM32L432KC_SPI.c/h` - SPI communication
- `STM32L432KC_DAC.c/h` - DAC audio output
- `STM32L432KC_TIMER.c/h` - Timer utilities
- `STM32L432KC_USART.c/h` - UART communication

### Audio Samples
- `wav_arrays/drum_samples.h` - Drum sample declarations
- `wav_arrays/*_sample.c` - Individual drum sample data arrays
  - `kick_sample.c`
  - `snare_sample.c`
  - `hihat_closed_sample.c`
  - `hihat_open_sample.c`
  - `crash_sample.c`
  - `ride_sample.c`
  - `tom_high_sample.c`
  - `tom_low_sample.c`

### System Files
- `STM32L4xx/` - STM32L4 system headers and startup code
  - `Device/Include/` - STM32L4 device headers
  - `Device/Source/` - System initialization code
  - `Source/` - Startup assembly files

### Documentation
- `PIN_CONNECTIONS.md` - Hardware pin connection guide

## Features

- **Direct BNO085 Communication**: MCU communicates directly with BNO085 sensor via SPI
- **SHTP Protocol**: Full implementation of Sensor Hub Transport Protocol
- **Robust Error Handling**: Packet validation, data bounds checking, error recovery
- **Exact Detection Logic**: Uses the same drum trigger detection logic as the original FPGA-based system
- **Single Sensor Support**: Currently configured for one sensor (right hand), but detection logic supports dual sensors

## Building

All necessary source files are included. Configure your build system to:
1. Include all `.c` files in `mcu2/` and `mcu2/wav_arrays/`
2. Add `mcu2/` and `mcu2/STM32L4xx/Device/Include` to include paths
3. Link with `STM32L4xx/Device/Source/system_stm32l4xx.c` and `STM32L4xx/Source/*.s` startup files

## Hardware Setup

See `PIN_CONNECTIONS.md` for detailed pin connection information.

## Detection Logic

The drum trigger detection logic in `drum_detector.c` **exactly matches** the original code:
- Same yaw angle zones (20-120, 340-20, 305-340, 200-305, etc.)
- Same gyroscope thresholds (-2500 for trigger detection)
- Same pitch-based cymbal detection (50° for crash/ride, 30° for hi-hat)
- Same state machine for preventing retriggering

## Notes

- The system is currently configured for a **single sensor** (Sensor 1 only)
- Sensor 2 parameters are passed as zeros - the detection logic will only trigger on Sensor 1 zones
- To add a second sensor, connect another BNO085 and modify `main.c` to read from both sensors
