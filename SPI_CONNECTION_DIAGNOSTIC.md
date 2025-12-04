# SPI Connection Diagnostic Guide

## Problem Summary

**Arduino is sending valid data:**
- Packet: `AA FF CF FF 8A 21 10 00 00 00 00 00 00 03 00 00`
- Header: 0xAA ✓
- Roll: -49 (0xFFCF) ✓
- Pitch: -118 (0xFF8A) ✓
- Yaw: 8464 (0x2110) ✓
- Flags: 0x03 (both valid) ✓

**FPGA is receiving zeros:**
- MCU receives: `0xAA 0x40 0x00...0x00 0x08`
- Header (0xAA) is hardcoded in `spi_slave_mcu.sv`
- Rest are zeros, error LED is ON

## Root Cause Analysis

The FPGA is **not receiving data from Arduino**. Possible causes:

### 1. SPI Wiring Issue
**Check physical connections:**
- **Arduino MOSI** → **FPGA arduino_sdi** (data line)
- **Arduino SCK** → **FPGA arduino_sck** (clock line)
- **Arduino CS (D10)** → **FPGA arduino_cs_n** (chip select, active low)

**Common issues:**
- Wires swapped (MOSI/SCK reversed)
- CS not connected or wrong pin
- Loose connections
- Wrong voltage levels (3.3V vs 5V)

### 2. CS Signal Not Working
**Symptoms:**
- FPGA never sees CS go low
- `packet_buffer` never gets written
- `cs_high_stable` never becomes true

**Check:**
- Verify CS pin on Arduino (D10 = GPIO21 on Nano ESP32)
- Verify CS pin on FPGA (arduino_cs_n input)
- Check if CS is actually toggling (use oscilloscope/logic analyzer)

### 3. SCK Not Reaching FPGA
**Symptoms:**
- No SCK rising edges detected
- `packet_buffer` stays at initialization values (all zeros)
- No bit shifting happening

**Check:**
- Verify SCK pin on Arduino (D13 = GPIO48 on Nano ESP32)
- Verify SCK pin on FPGA (arduino_sck input)
- Check SCK frequency (should be 100kHz = 10us period)

### 4. Timing Issue
**Symptoms:**
- Data arrives but timing is off
- CDC (Clock Domain Crossing) not working correctly

**Less likely** since testbench passes, but possible in hardware.

## Diagnostic Steps

### Step 1: Verify SPI Pins on Arduino
```cpp
// In ARDUINO_SENSOR_BRIDGE.ino, verify:
#define FPGA_SPI_CS 10  // D10 = GPIO21 on Nano ESP32
// Default SPI pins:
// D11 = COPI/MOSI (GPIO38)
// D12 = CIPO/MISO (GPIO47) - not used (FPGA doesn't send data)
// D13 = SCK (GPIO48)
```

### Step 2: Check FPGA Pin Assignments
Verify in your FPGA constraint file (`.pcf` or `.sdc`):
- `arduino_sdi` → correct FPGA pin
- `arduino_sck` → correct FPGA pin
- `arduino_cs_n` → correct FPGA pin

### Step 3: Use Oscilloscope/Logic Analyzer
**If available, check:**
1. **CS signal**: Should toggle LOW during packet transmission (~1.28ms)
2. **SCK signal**: Should show 100kHz clock (10us period) during transmission
3. **MOSI signal**: Should show data bits during SCK rising edges
4. **Timing**: CS goes LOW → SCK starts → 128 SCK edges → CS goes HIGH

### Step 4: Add LED Indicators (if possible)
Add temporary LED outputs to FPGA to indicate:
- CS detected (LED on when CS is low)
- SCK detected (LED blinks with SCK)
- Packet received (LED on when packet_valid is true)

### Step 5: Check Error LED Behavior
**Current state:** Error LED is ON
- This means `error = 1` in `arduino_spi_slave.sv`
- Error is set when `header != 0xAA` AND `packet_received == 1`
- If error LED is ON, it means:
  - Either header is not 0xAA (FPGA receiving wrong data)
  - Or `packet_received` is true but header is wrong

**If error LED is ON but MCU sees 0xAA:**
- The 0xAA is hardcoded in `spi_slave_mcu.sv` (line 208)
- This means FPGA is NOT receiving data, so `spi_slave_mcu` is sending hardcoded header

## Expected Behavior

**When working correctly:**
1. Arduino sends packet every 50ms (~20Hz)
2. FPGA receives packet in `packet_buffer`
3. FPGA captures packet in `packet_snapshot` when CS goes high
4. FPGA parses packet and sets outputs
5. `initialized` LED should be ON (header = 0xAA)
6. `error` LED should be OFF
7. MCU should receive actual sensor data (not zeros)

## Quick Checks

1. **Verify Arduino is actually sending:**
   - Check Serial Monitor - should see "[DEBUG] Packet sent via SPI to FPGA" every 50ms
   - ✅ Confirmed: Arduino is sending

2. **Verify FPGA pins are correct:**
   - Check FPGA constraint file
   - Verify pin assignments match physical connections

3. **Verify CS is toggling:**
   - CS should go LOW for ~1.28ms every 50ms
   - If CS never goes low, FPGA won't receive data

4. **Verify SCK is toggling:**
   - SCK should show 128 clock edges (16 bytes × 8 bits) during each packet
   - If SCK never toggles, FPGA won't receive data

## Next Steps

1. **Physical connection check**: Verify all SPI wires are connected correctly
2. **CS signal check**: Verify CS pin is actually toggling (use multimeter/oscilloscope)
3. **SCK signal check**: Verify SCK is toggling at 100kHz during transmission
4. **FPGA pin assignments**: Verify FPGA pins match physical connections

The code is correct (testbench passes), so this is almost certainly a **hardware/wiring issue**.

