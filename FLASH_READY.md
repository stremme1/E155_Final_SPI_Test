# Flash-Ready Code Organization

## ✅ Code Organized into Two Folders

### 📁 `fpga/` - FPGA Code
All SystemVerilog files needed to flash the FPGA:
- **Top-level**: `drum_trigger_top_integrated.sv`
- **Dependencies**: All supporting modules (11 files total)
- **See**: `fpga/README.md` for details

### 📁 `mcu/` - MCU Code  
All C files and dependencies needed to flash the STM32L432KC:
- **Main**: `main_integrated.c`
- **Libraries**: All STM32L432KC_*.c/.h files
- **Samples**: `wav_arrays/` folder
- **Device Support**: STM32L4xx/, CMSIS_5/
- **See**: `mcu/README.md` for details

## Quick Start

### Flash FPGA
1. Open `fpga/drum_trigger_top_integrated.sv` in your FPGA toolchain
2. Add all `.sv` files in `fpga/` folder to project
3. Set `drum_trigger_top_integrated` as top-level
4. Assign pins (see `fpga/README.md`)
5. Synthesize and flash

### Flash MCU
1. Create new STM32 project in your IDE
2. Add all files from `mcu/` folder
3. Set `main_integrated.c` as main file
4. Configure include paths (STM32L4xx/, CMSIS_5/, wav_arrays/)
5. Build and flash

## System Overview

```
┌─────────────┐         SPI          ┌─────────────┐
│     MCU     │◄────────────────────►│    FPGA     │
│             │  (PB3,4,5, PA5,6,11) │             │
│  - SPI      │                       │  - Dual     │
│  - DAC      │                       │    BNO085   │
│  - Audio    │                       │  - Drum     │
│             │                       │    Trigger  │
└─────────────┘                       └─────────────┘
```

## Pin Connections

### MCU ↔ FPGA SPI
- MCU PB3 → FPGA mcu_sck
- MCU PB4 → FPGA mcu_sdo (MISO)
- MCU PB5 → FPGA mcu_sdi (MOSI)
- MCU PA5 → FPGA mcu_load
- MCU PA6 → FPGA mcu_done

### FPGA ↔ BNO085 Sensors
- FPGA sclk → Both sensors SCK
- FPGA mosi → Both sensors MOSI
- FPGA miso1 → Sensor 1 MISO
- FPGA miso2 → Sensor 2 MISO
- FPGA cs_n1 → Sensor 1 CS
- FPGA cs_n2 → Sensor 2 CS
- FPGA int1 → Sensor 1 INT
- FPGA int2 → Sensor 2 INT
- FPGA bno085_rst_n → Both sensors RST

## Status

✅ All code organized and ready to flash
✅ Dependencies included
✅ README files created
✅ Pin assignments documented

