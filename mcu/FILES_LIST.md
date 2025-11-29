# MCU Files List

## Main Application
- **`main.c`** - Main application file (SPI + sensor decoding + drum detection + audio playback)

## STM32L432KC Libraries
- **`STM32L432KC_SPI.c/.h`** - SPI communication
- **`STM32L432KC_GPIO.c/.h`** - GPIO control
- **`STM32L432KC_DAC.c/.h`** - DAC audio output
- **`STM32L432KC_TIMER.c/.h`** - Timer functions
- **`STM32L432KC_RCC.c/.h`** - Clock configuration
- **`STM32L432KC_FLASH.c/.h`** - Flash latency

## Drum Samples
- **`wav_arrays/drum_samples.h`** - Header with all sample declarations
- **`wav_arrays/*.c`** - Individual sample data files:
  - `snare_sample.c`
  - `hihat_closed_sample.c`
  - `hihat_open_sample.c`
  - `kick_sample.c`
  - `crash_sample.c`
  - `ride_sample.c`
  - `tom_high_sample.c`
  - `tom_low_sample.c`

## Device Support
- **`STM32L4xx/`** - STM32L4 device headers and startup
- **`CMSIS_5/`** - ARM CMSIS headers
- **`SEGGER_THUMB_Startup.s`** - Startup assembly
- **`STM32L4xx_Flash.icf`** - Linker script

## Build Requirements

1. Add all `.c` files to project
2. Add all `.h` files to include paths
3. Add `STM32L4xx/Device/Include/` to includes
4. Add `CMSIS_5/CMSIS/Core/Include/` to includes
5. Add `wav_arrays/` to includes
6. Use `STM32L4xx_Flash.icf` as linker script
7. Set `main.c` as main file

