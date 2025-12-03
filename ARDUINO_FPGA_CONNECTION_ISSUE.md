# Arduino-FPGA Connection Issue Analysis

## Problem Summary
The FPGA is receiving garbage data from the Arduino, but the testbench passes all tests, indicating the FPGA logic is correct.

## Root Cause Analysis

### 1. Arduino Code Configuration Issue

**Current Arduino Code** (`ARDUINO_SENSOR_BRIDGE.ino`):
- Line 232: `#define MCU_SPI_CS 10` - Configured to send to MCU
- Line 442: `digitalWrite(MCU_SPI_CS, LOW);` - Uses pin 10 as CS
- Line 441: `SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));` - 100kHz, MSB first, Mode 0

**Problem**: The Arduino code is configured to send to the MCU (pin 10), but the FPGA expects to receive from the Arduino.

### 2. Expected Architecture
```
Arduino (Master) → FPGA (Slave) → MCU (Master reads from FPGA)
```

The Arduino should send to the FPGA, not directly to the MCU.

### 3. FPGA Expectations
- `arduino_spi_slave.sv` expects:
  - `arduino_cs_n` - Chip select from Arduino (active low)
  - `arduino_sck` - SPI clock from Arduino
  - `arduino_sdi` - SPI data in (MOSI from Arduino)

## Solutions

### Option 1: Fix Arduino Code (Recommended)
Change the Arduino code to send to FPGA instead of MCU:

```cpp
// Change from:
#define MCU_SPI_CS 10

// To:
#define FPGA_SPI_CS 10  // Or whatever pin is connected to FPGA CS

// And update the comment:
// FPGA SPI Chip Select pin (D10 = GPIO21 on Nano ESP32 → FPGA CS pin)
```

**Verify Wiring**:
- Arduino D10 (or chosen pin) → FPGA `arduino_cs_n` input
- Arduino D13 (SCK) → FPGA `arduino_sck` input
- Arduino D11 (MOSI) → FPGA `arduino_sdi` input
- Common GND between Arduino and FPGA

### Option 2: Check Physical Connections
If the Arduino is already wired to the FPGA, verify:
1. CS pin connection (Arduino → FPGA CS input)
2. SCK connection (Arduino → FPGA SCK input)
3. MOSI connection (Arduino → FPGA SDI input)
4. GND connection (common ground)

### Option 3: Verify SPI Settings Match
Both Arduino and FPGA must use:
- **SPI Mode 0** (CPOL=0, CPHA=0) ✓ (Both match)
- **MSB First** ✓ (Both match)
- **Clock Speed**: Arduino uses 100kHz, FPGA should handle this ✓

## Testbench Results
The testbench passes all 39 tests, confirming:
- ✅ Bit shifting logic is correct
- ✅ Packet parsing is correct
- ✅ Header validation works
- ✅ Data mapping works

This confirms the FPGA logic is correct, so the issue is in the physical connection or Arduino configuration.

## Next Steps
1. **Verify Arduino is sending to FPGA** (not MCU)
2. **Check physical wiring** between Arduino and FPGA
3. **Verify CS pin** is connected correctly
4. **Add debug output** to Arduino to confirm packets are being sent
5. **Use oscilloscope/logic analyzer** to verify SPI signals if available

## Debugging Tips
1. Add Serial.print in Arduino to log packet contents before sending
2. Check if FPGA error LED is on (indicates header mismatch)
3. Verify CS timing - CS should go low, then 16 bytes sent, then CS high
4. Check if bit_count and byte_count in FPGA are incrementing (if accessible via debug)

