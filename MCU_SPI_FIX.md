# MCU SPI Configuration Fix

## Issue
MCU was not generating SPI clock signal when tested.

## Root Causes Identified

1. **Baud Rate Divider**: Code was using `br=3` (clock/16) instead of `br=1` (clock/4) like Lab07
2. **Register Initialization**: SPI registers should be cleared before configuration
3. **Pin Configuration Order**: Pins must be configured before setting alternate functions

## Changes Made

### 1. Fixed `initSPI()` in `STM32L432KC_SPI.c`
- Clear SPI1->CR1 and SPI1->CR2 before configuration
- Added comments explaining each step
- Ensured proper initialization order

### 2. Fixed `main_integrated.c`
- Changed `initSPI(3, 0, 0)` to `initSPI(1, 0, 0)` to match Lab07
- This gives faster SPI clock (system_clock / 4 instead of /16)

## Verification Checklist

- [x] GPIO clocks enabled (GPIOAEN, GPIOBEN)
- [x] SPI1 clock enabled (SPI1EN)
- [x] Pins configured as alternate function (PB3, PB4, PB5)
- [x] Alternate function set to AF5 for SPI1
- [x] SPI configured as master (MSTR)
- [x] SPI Mode 0 (CPOL=0, CPHA=0)
- [x] 8-bit data size
- [x] SPI enabled (SPE) - LAST step

## Testing

The SPI clock should now appear on PB3 when `spiSendReceive()` is called.

