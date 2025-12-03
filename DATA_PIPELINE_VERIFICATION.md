# Data Pipeline Verification

## Complete Data Flow: Arduino → FPGA → MCU

### Stage 1: Arduino Sends to FPGA

**Arduino Packet Format (16 bytes):**
```
Byte 0:    Header (0xAA)
Byte 1-2:  Roll (int16_t, MSB first) - Euler angle scaled by 100
Byte 3-4:  Pitch (int16_t, MSB first) - Euler angle scaled by 100
Byte 5-6:  Yaw (int16_t, MSB first) - Euler angle scaled by 100
Byte 7-8:  Gyro X (int16_t, MSB first) - scaled by 2000
Byte 9-10: Gyro Y (int16_t, MSB first) - scaled by 2000
Byte 11-12: Gyro Z (int16_t, MSB first) - scaled by 2000
Byte 13:   Flags (bit 0 = Euler valid, bit 1 = Gyro valid)
Byte 14-15: Reserved (0x00)
```

**Arduino Code Location:** `ARDUINO_SENSOR_BRIDGE.ino` lines 181-221

**SPI Settings:**
- Mode: SPI_MODE0 (CPOL=0, CPHA=0)
- Bit Order: MSBFIRST
- Clock Rate: 100kHz
- CS Pin: D10 (FPGA_SPI_CS)

---

### Stage 2: FPGA Receives from Arduino

**FPGA Module:** `arduino_spi_slave.sv`

**Receives:** Same 16-byte packet format as Arduino sends

**Parses:**
- Header from byte 0
- Roll from bytes 1-2 → stored as `roll` (signed int16)
- Pitch from bytes 3-4 → stored as `pitch` (signed int16)
- Yaw from bytes 5-6 → stored as `yaw` (signed int16)
- Gyro X from bytes 7-8 → stored as `gyro_x` (signed int16)
- Gyro Y from bytes 9-10 → stored as `gyro_y` (signed int16)
- Gyro Z from bytes 11-12 → stored as `gyro_z` (signed int16)
- Flags from byte 13 → stored as `flags` (bit 0 = Euler valid, bit 1 = Gyro valid)

**Maps to spi_slave_mcu Interface:**
- `quat1_w` = 16384 (Q14 format = 1.0, hardcoded for Euler angles)
- `quat1_x` = `roll` (Roll → quat_x)
- `quat1_y` = `pitch` (Pitch → quat_y)
- `quat1_z` = `yaw` (Yaw → quat_z)
- `gyro1_x` = `gyro_x` (pass through)
- `gyro1_y` = `gyro_y` (pass through)
- `gyro1_z` = `gyro_z` (pass through)
- `quat1_valid` = `flags[0]` (Euler valid bit)
- `gyro1_valid` = `flags[1]` (Gyro valid bit)
- `initialized` = 1 if header == 0xAA, else 0
- `error` = 1 if header != 0xAA, else 0

**Code Location:** `arduino_spi_slave.sv` lines 154-262

---

### Stage 3: FPGA Sends to MCU

**FPGA Module:** `spi_slave_mcu.sv`

**Packet Format (16 bytes):**
```
Byte 0:    Header (0xAA)
Byte 1-2:  quat_w (MSB,LSB) - Q14 format (16384 = 1.0)
Byte 3-4:  quat_x (MSB,LSB) - Roll from Arduino
Byte 5-6:  quat_y (MSB,LSB) - Pitch from Arduino
Byte 7-8:  quat_z (MSB,LSB) - Yaw from Arduino
Byte 9-10: gyro_x (MSB,LSB) - Gyro X from Arduino
Byte 11-12: gyro_y (MSB,LSB) - Gyro Y from Arduino
Byte 13-14: gyro_z (MSB,LSB) - Gyro Z from Arduino
Byte 15:   Flags (bit 0=quat_valid, bit 1=gyro_valid, bit 2=initialized, bit 3=error)
```

**Code Location:** `spi_slave_mcu.sv` lines 195-219

**SPI Settings:**
- Mode: SPI Mode 0 (CPOL=0, CPHA=0)
- Bit Order: MSB First
- FPGA is slave, MCU is master

---

### Stage 4: MCU Receives from FPGA

**MCU Code:** `STM32L432KC_SPI.c`

**Expects:** Same 16-byte packet format as FPGA sends

**Parses:**
- Header from byte 0 (must be 0xAA)
- quat_w from bytes 1-2 (MSB,LSB)
- quat_x from bytes 3-4 (MSB,LSB)
- quat_y from bytes 5-6 (MSB,LSB)
- quat_z from bytes 7-8 (MSB,LSB)
- gyro_x from bytes 9-10 (MSB,LSB)
- gyro_y from bytes 11-12 (MSB,LSB)
- gyro_z from bytes 13-14 (MSB,LSB)
- Flags from byte 15 (bit 0=quat_valid, bit 1=gyro_valid)

**Code Location:** `STM32L432KC_SPI.c` lines 208-301

---

## Data Transformation Summary

| Stage | Roll | Pitch | Yaw | Gyro X | Gyro Y | Gyro Z | Flags |
|-------|------|-------|-----|--------|--------|--------|-------|
| **Arduino** | Bytes 1-2 | Bytes 3-4 | Bytes 5-6 | Bytes 7-8 | Bytes 9-10 | Bytes 11-12 | Byte 13 (bits 0-1) |
| **FPGA Receive** | `roll` | `pitch` | `yaw` | `gyro_x` | `gyro_y` | `gyro_z` | `flags` |
| **FPGA Map** | `quat_x` | `quat_y` | `quat_z` | `gyro_x` | `gyro_y` | `gyro_z` | `quat_valid`, `gyro_valid` |
| **FPGA Send** | Bytes 3-4 | Bytes 5-6 | Bytes 7-8 | Bytes 9-10 | Bytes 11-12 | Bytes 13-14 | Byte 15 (bits 0-1) |
| **MCU Receive** | `quat_x` | `quat_y` | `quat_z` | `gyro_x` | `gyro_y` | `gyro_z` | `quat_valid`, `gyro_valid` |

**Note:** FPGA adds `quat_w = 16384` (Q14 format = 1.0) at bytes 1-2 when sending to MCU.

---

## Verification Checklist

- [x] Arduino sends 16-byte packet with correct format
- [x] FPGA receives and parses Arduino packet correctly
- [x] FPGA maps Euler angles (Roll/Pitch/Yaw) to quaternion fields (quat_x/y/z)
- [x] FPGA adds quat_w = 16384 (Q14 format)
- [x] FPGA passes through gyroscope data unchanged
- [x] FPGA maps flags correctly (Euler valid → quat_valid, Gyro valid → gyro_valid)
- [x] FPGA sends 16-byte packet to MCU with correct format
- [x] MCU receives and parses FPGA packet correctly
- [x] All byte positions match between stages
- [x] All MSB-first byte ordering is consistent
- [x] Flags byte format matches between FPGA send and MCU receive

---

## Potential Issues to Check

1. **TEST_MODE in spi_slave_mcu.sv**: Currently set to `1'b1` (line 44) - this sends test pattern instead of real data!
2. **Flag byte differences**: Arduino sends flags at byte 13, FPGA sends flags at byte 15 (this is correct - different packet formats)
3. **Byte ordering**: All stages use MSB-first (correct)

