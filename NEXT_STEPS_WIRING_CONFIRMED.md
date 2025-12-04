# Next Steps - Wiring Confirmed ✅

## Status
- ✅ Arduino is sending valid data (confirmed via Serial Monitor)
- ✅ Physical wiring is correct:
  - D13 (SCK) → FPGA arduino_sck
  - D11 (COPI/MOSI) → FPGA arduino_sdi
  - D10 (CS) → FPGA arduino_cs_n
  - GND → GND
- ✅ FPGA code is correct (testbench passes)
- ❌ FPGA pin assignments missing (CRITICAL)

## Critical Next Step: Add FPGA Pin Assignments

Since wiring is correct, the FPGA **must** have pin assignments for these signals to work.

### Required Pin Assignments

Add to your FPGA constraint file (`.pcf` or `.sdc`):

```
# Arduino SPI Interface (FPGA is slave to Arduino)
set_io arduino_sck   PXX  # Replace PXX with actual FPGA pin connected to Arduino D13
set_io arduino_sdi   PXX  # Replace PXX with actual FPGA pin connected to Arduino D11
set_io arduino_cs_n  PXX  # Replace PXX with actual FPGA pin connected to Arduino D10

# Diagnostic LEDs (optional but recommended)
set_io led_cs_detected     PXX  # High when CS is low (Arduino sending)
set_io led_packet_received PXX  # High when packet_valid is true
```

### How to Find Pin Numbers

1. **Check your physical connections**: Which FPGA pins did you actually wire to?
2. **Check FPGA board schematic**: Find available GPIO pins
3. **Use FPGA vendor tools**: Most tools have pin assignment GUIs

### Example (if using specific pins):
```
# Example - replace with your actual pin numbers
set_io arduino_sck   P15
set_io arduino_sdi   P16
set_io arduino_cs_n  P17
set_io led_cs_detected     P19
set_io led_packet_received P20
```

## After Adding Pin Assignments

### 1. Re-synthesize FPGA
- Run synthesis with updated constraint file
- Verify no errors about unassigned pins
- Generate bitstream

### 2. Program FPGA
- Load new bitstream to FPGA
- Verify programming succeeded

### 3. Test Diagnostic LEDs

**Expected Behavior:**
- **led_heartbeat**: Should be flashing (FPGA is running) ✅
- **led_cs_detected**: Should blink every ~50ms (when Arduino sends)
  - If blinks: CS signal is working ✓
  - If never lights: Check pin assignment or connection
  
- **led_packet_received**: Should blink every ~50ms (when packet captured)
  - If blinks: Packet capture is working ✓
  - If never lights: Check SCK or MOSI pin assignments

- **led_initialized**: Should turn ON (when valid header received)
  - If ON: System is working! ✓
  - If OFF: Check all connections and pin assignments

- **led_error**: Should turn OFF (when valid header received)
  - If OFF: System is working! ✓
  - If ON: Invalid header or timing issue

## Troubleshooting Guide

### If led_cs_detected never lights:
1. Verify `arduino_cs_n` pin assignment matches physical connection
2. Check D10 wire connection
3. Verify CS pin assignment in constraint file

### If led_cs_detected blinks but led_packet_received never lights:
1. Verify `arduino_sck` pin assignment matches physical connection
2. Verify `arduino_sdi` pin assignment matches physical connection
3. Check D13 (SCK) and D11 (MOSI) wire connections
4. Check for signal integrity issues

### If both LEDs blink but led_error is ON:
1. Check if header byte is being received correctly
2. Verify SPI mode matches (Mode 0)
3. Check for timing issues
4. Verify bit order (MSB first)

### If all LEDs work but MCU still receives zeros:
1. Check `spi_slave_mcu` connections
2. Verify MCU SPI interface
3. Check data pipeline from `arduino_spi_slave` to `spi_slave_mcu`

## Verification Checklist

- [ ] Added `arduino_sck` pin assignment to constraint file
- [ ] Added `arduino_sdi` pin assignment to constraint file
- [ ] Added `arduino_cs_n` pin assignment to constraint file
- [ ] Added `led_cs_detected` pin assignment (optional)
- [ ] Added `led_packet_received` pin assignment (optional)
- [ ] Re-synthesized FPGA with new pin assignments
- [ ] Programmed FPGA with new bitstream
- [ ] Verified `led_heartbeat` is flashing
- [ ] Verified `led_cs_detected` blinks every ~50ms
- [ ] Verified `led_packet_received` blinks every ~50ms
- [ ] Verified `led_initialized` turns ON
- [ ] Verified `led_error` turns OFF
- [ ] Verified MCU receives actual sensor data

## Expected Timeline

Once pin assignments are added:
- **led_cs_detected**: Should work immediately (CS signal detection)
- **led_packet_received**: Should work if SCK/MOSI are correct
- **led_initialized**: Should turn ON within 1-2 seconds (first valid packet)
- **led_error**: Should turn OFF when initialized turns ON

## Summary

Since wiring is confirmed correct, the **only remaining issue** is the missing FPGA pin assignments. Once you:

1. Add pin assignments to constraint file
2. Re-synthesize FPGA
3. Program FPGA

The system should work! The diagnostic LEDs will help verify each step.

