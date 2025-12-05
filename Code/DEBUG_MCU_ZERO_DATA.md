# Debug: MCU Receiving Zero Data from Arduino

## Problem
MCU is receiving:
- Byte[0] = 0xAA ✓ (header correct)
- Byte[1] = 0x40 ✓ (quat_w MSB = 0x4000 >> 8, correct)
- Bytes[2-14] = 0x00 ✗ (should be sensor data: Roll, Pitch, Yaw, Gyro)
- Byte[15] = 0x08 (flags: error=1, initialized=0, gyro_valid=0, quat_valid=0)

## Root Cause Analysis

### Data Pipeline
1. **Arduino** → sends 16-byte packet via SPI
2. **arduino_spi_slave** → receives packet, parses, outputs to spi_slave_mcu
3. **spi_slave_mcu** → captures snapshot, sends to MCU

### Current Status
- ✅ Header (0xAA) is being received correctly
- ✅ quat_w (0x4000 = 16384) is being set correctly (hardcoded)
- ✗ All sensor data (quat_x, quat_y, quat_z, gyro_x, gyro_y, gyro_z) is zero
- ✗ Flags show error=1, valid=0

### Likely Issues

#### Issue 1: Packet Not Being Captured
**Symptom**: `packet_snapshot` is all zeros or stale data
**Check**: 
- Is `cs_high_stable` going high after CS goes high?
- Is `packet_valid_raw` going high?
- Is `packet_snapshot` being written correctly?

#### Issue 2: Parsing Not Working
**Symptom**: `packet_snapshot` has data, but `roll`, `pitch`, `yaw` are zero
**Check**:
- Is `packet_valid` going high?
- Are parsed values (`roll`, `pitch`, `yaw`, `gyro_x`, etc.) being set?
- Is the byte order correct (MSB-first)?

#### Issue 3: Outputs Not Being Updated
**Symptom**: Parsed values are correct, but outputs are zero
**Check**:
- Is `new_packet_available` going high?
- Is `header == HEADER_BYTE` true?
- Are outputs being updated in the `always_ff` block?

#### Issue 4: Timing Issue
**Symptom**: Data is captured but `spi_slave_mcu` captures before update
**Check**:
- Is `new_packet_available` high for long enough?
- Is `spi_slave_mcu` capturing snapshot at the right time?

## Debugging Steps

### Step 1: Verify Arduino is Sending Data
- Check Arduino serial output to confirm packet is being sent
- Verify packet format matches expected format

### Step 2: Verify FPGA is Receiving Data
- Add debug signals to check `packet_buffer` after CS goes high
- Verify `packet_buffer[0]` = 0xAA
- Verify `packet_buffer[1-6]` contain Roll/Pitch/Yaw data

### Step 3: Verify CDC is Working
- Check `cs_high_stable` goes high after CS goes high
- Check `packet_valid_raw` goes high
- Check `packet_snapshot` contains correct data

### Step 4: Verify Parsing is Working
- Check `packet_valid` goes high
- Check `roll`, `pitch`, `yaw` are non-zero
- Check `flags` is correct

### Step 5: Verify Outputs are Updated
- Check `new_packet_available` goes high
- Check `header == HEADER_BYTE` is true
- Check outputs (`quat1_x`, `quat1_y`, etc.) are updated

### Step 6: Verify MCU Capture
- Check `spi_slave_mcu` snapshot is updated when CS is high
- Verify snapshot contains correct data

## Most Likely Issue

Based on the symptoms (header correct, quat_w correct, everything else zero), the most likely issue is:

**Issue 2 or 3**: The packet is being received (header is correct), but either:
1. The parsing is not extracting the data correctly (bytes 1-13 are not being parsed)
2. The outputs are not being updated because `new_packet_available` is not going high or `header != HEADER_BYTE`

## Recommended Fix

1. **Add debug output** to verify `packet_snapshot` contains data
2. **Check timing** - ensure `new_packet_available` stays high long enough
3. **Verify header check** - ensure `header == HEADER_BYTE` is true
4. **Check byte order** - ensure MSB-first parsing is correct

## Quick Test

The fact that `quat_w = 0x4000` is being set correctly suggests the output update logic is working. The issue is likely that `roll`, `pitch`, `yaw`, `gyro_x`, etc. are all zero when the outputs are updated.

This suggests either:
- The packet is not being captured correctly (packet_snapshot is zeros)
- The parsing is not working (parsed values are zeros)
- The Arduino is sending zeros (unlikely, but possible)

