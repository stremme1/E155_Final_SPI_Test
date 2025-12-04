# Diagnostic Summary - Following SPI_CONNECTION_DIAGNOSTIC.md

## Step 1: Verify SPI Pins on Arduino ✅

**Status: CONFIRMED**
- `FPGA_SPI_CS = 10` (D10 = GPIO21 on Nano ESP32) ✓
- Default SPI pins:
  - D11 = COPI/MOSI (GPIO38) ✓
  - D13 = SCK (GPIO48) ✓
- Arduino is sending valid packets (confirmed via Serial Monitor) ✓

## Step 2: Check FPGA Pin Assignments ❌

**Status: CRITICAL ISSUE FOUND**

The constraint file (`Old_SPI_test_xa/constraints.pcf`) is **MISSING** Arduino SPI pin assignments:

**Missing Pin Assignments:**
- `arduino_sck` - No pin assigned
- `arduino_sdi` - No pin assigned  
- `arduino_cs_n` - No pin assigned

**Action Required:**
1. Add pin assignments to constraint file (see `FPGA_PIN_ASSIGNMENTS_NEEDED.md`)
2. Re-synthesize FPGA with correct pin assignments
3. Verify physical connections match pin assignments

## Step 3: Diagnostic Outputs Added ✅

**Status: IMPLEMENTED**

Added diagnostic outputs to help debug:

1. **`led_cs_detected`** - High when CS is low (Arduino is sending)
   - Should blink every ~50ms when Arduino sends packets
   - If never lights: CS signal not reaching FPGA

2. **`led_packet_received`** - High when `packet_valid` is true
   - Should blink every ~50ms when packet is captured
   - If never lights: Either CS or SCK not working

**Note:** These LEDs need pin assignments in constraint file before they can be used.

## Step 4: Current LED Status

**Observed Behavior:**
- ❌ `led_error` is ON - Indicates `error = 1`
- ❓ `led_initialized` status unknown
- ✅ `led_heartbeat` is flashing (FPGA is running)

**Error LED ON means:**
- `header != 0xAA` OR `packet_received == 1` but header is wrong
- Most likely: FPGA not receiving data, so `packet_snapshot[0] = 0x00` (not 0xAA)

## Step 5: Root Cause Analysis

**Most Likely Cause: Missing FPGA Pin Assignments**

Without pin assignments, the FPGA synthesis tool doesn't know which physical pins to use for:
- `arduino_sck` (SCK signal)
- `arduino_sdi` (MOSI data)
- `arduino_cs_n` (CS signal)

**Result:** These signals may not be connected to any physical pins, so the FPGA never receives data.

## Next Steps (Priority Order)

### 1. Add FPGA Pin Assignments (CRITICAL)
- Add `arduino_sck`, `arduino_sdi`, `arduino_cs_n` to constraint file
- Add `led_cs_detected`, `led_packet_received` to constraint file
- See `FPGA_PIN_ASSIGNMENTS_NEEDED.md` for details

### 2. Re-synthesize FPGA
- Run synthesis with new pin assignments
- Program FPGA with new bitstream

### 3. Test Diagnostic LEDs
- **led_cs_detected**: Should blink every ~50ms
  - If blinks: CS signal is working ✓
  - If never lights: CS not connected or wrong pin ✗
  
- **led_packet_received**: Should blink every ~50ms
  - If blinks: Packet capture is working ✓
  - If never lights: SCK not working or timing issue ✗

### 4. Verify Physical Connections
- Check Arduino MOSI → FPGA `arduino_sdi` pin
- Check Arduino SCK → FPGA `arduino_sck` pin
- Check Arduino CS (D10) → FPGA `arduino_cs_n` pin
- Verify connections match pin assignments

### 5. Check Signal Levels
- Verify voltage levels match (3.3V vs 5V)
- Check for loose connections
- Use multimeter/oscilloscope if available

## Expected Behavior After Fix

Once pin assignments are correct and FPGA is re-synthesized:

1. **led_cs_detected** blinks every ~50ms ✓
2. **led_packet_received** blinks every ~50ms ✓
3. **led_initialized** turns ON (header = 0xAA received) ✓
4. **led_error** turns OFF (valid header received) ✓
5. MCU receives actual sensor data (not zeros) ✓

## Summary

- ✅ Arduino code is correct and sending valid data
- ✅ FPGA code is correct (testbench passes)
- ❌ **FPGA pin assignments are missing** (CRITICAL)
- ❓ Physical wiring needs verification

**The missing pin assignments are almost certainly why the FPGA isn't receiving data.**

