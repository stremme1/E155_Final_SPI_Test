# Code Review - Critical Issues Found

## Summary
After comparing the codebase with Lab07 working code, several critical issues were identified that will prevent proper MCU-FPGA communication.

---

## 🔴 CRITICAL ISSUE #1: Packet Size Mismatch

### Problem
- **FPGA sends**: 15 bytes (1 header + 14 data bytes for single sensor)
- **MCU `readDrumCommand()`**: Reads only 1 byte
- **MCU `readSensorDataPacket()`**: Expects 32 bytes (dual sensor format)

### Location
- `fpga/sensor_data_formatter.sv`: `PACKET_SIZE = 15`
- `mcu/STM32L432KC_SPI.c`: `readDrumCommand()` reads 1 byte
- `mcu/main_sensor_data.c`: `readSensorDataPacket()` expects 32 bytes

### Fix Required
Create a new function `readSensorDataPacket15()` that reads 15 bytes:

```c
void readSensorDataPacket15(uint8_t *packet) {
    uint8_t i;
    
    // Wait for DONE signal (Lab07 pattern)
    while(!digitalReadPortA(SPI_DONE));
    
    // Lower CE to start transaction
    digitalWritePortA(SPI_CE, GPIO_LOW);
    
    // Read 15 bytes
    for (i = 0; i < 15; i++) {
        packet[i] = (uint8_t)spiSendReceive(0x00);
    }
    
    // Wait for SPI transaction to complete
    while(SPI1->SR & SPI_SR_BSY);
    
    // Raise CE to end transaction
    digitalWritePortA(SPI_CE, GPIO_HIGH);
    
    // Acknowledge by toggling LOAD (Lab07 pattern)
    digitalWritePortA(SPI_LOAD, GPIO_HIGH);
    digitalWritePortA(SPI_LOAD, GPIO_LOW);
}
```

---

## 🔴 CRITICAL ISSUE #2: Byte Order (Endianness) Mismatch

### Problem
- **FPGA sends**: Little-endian (LSB first, MSB second)
  - Byte 1: LSB of quat_w
  - Byte 2: MSB of quat_w
- **MCU parses**: Big-endian (MSB first, LSB second)
  - `(packet[0] << 8) | packet[1]` - WRONG! This treats byte 0 as MSB

### Location
- `mcu/main_sensor_data.c` lines 271-274:
```c
data->quat1_w = (int16_t)((packet[0] << 8) | packet[1]);  // WRONG!
```

### Fix Required
Parse as little-endian (LSB first):
```c
// Correct: LSB first, MSB second
data->quat1_w = (int16_t)(packet[1] | (packet[2] << 8));  // Byte 1=LSB, Byte 2=MSB
data->quat1_x = (int16_t)(packet[3] | (packet[4] << 8));
data->quat1_y = (int16_t)(packet[5] | (packet[6] << 8));
data->quat1_z = (int16_t)(packet[7] | (packet[8] << 8));
data->gyro1_x = (int16_t)(packet[9] | (packet[10] << 8));
data->gyro1_y = (int16_t)(packet[11] | (packet[12] << 8));
data->gyro1_z = (int16_t)(packet[13] | (packet[14] << 8));
```

**Note**: Byte 0 is the header (0xAA), so data starts at byte 1.

---

## 🟡 ISSUE #3: SPI CE Handling Pattern

### Problem
Lab07 toggles CE for each byte, but current code keeps CE low for entire transaction.

### Location
- `mcu/STM32L432KC_SPI.c`: `readDrumCommand()` keeps CE low for 1 byte
- Lab07 `lab7.c`: Toggles CE for each byte

### Analysis
**Lab07 pattern** (line 117-119):
```c
digitalWrite(PA11, 1); // CE high
spiSendReceive(plaintext[i]);
digitalWrite(PA11, 0); // CE low
```

**Current code**:
```c
digitalWritePortA(SPI_CE, GPIO_LOW);  // CE low
spiSendReceive(0x00);                 // Read byte
digitalWritePortA(SPI_CE, GPIO_HIGH); // CE high
```

### Recommendation
The current pattern (CE low for entire transaction) should work with the FPGA SPI slave, which expects continuous transaction. However, verify that the FPGA properly handles this. The FPGA code in `spi_slave_mcu.sv` checks `if (!cs_sync2)` for continuous transaction, so it should work.

**Action**: Test both patterns, but current code should work.

---

## 🟡 ISSUE #4: Missing Main Function for Sensor Data

### Problem
- `main_integrated.c` uses `readDrumCommand()` (1 byte) - wrong for sensor data
- `main_sensor_data.c` has `readSensorDataPacket()` but expects 32 bytes - wrong for single sensor
- No main function that reads 15-byte single sensor packets

### Fix Required
Update `main_integrated.c` to read 15 bytes and parse correctly, OR create a new main function.

---

## 🟡 ISSUE #5: SPI Clock Configuration

### Problem
Need to verify SPI clock configuration matches between MCU and FPGA.

### Current Settings
- MCU: `initSPI(3, 0, 0)` - BR=3 (divide by 16), CPOL=0, CPHA=0
- FPGA: Expects Mode 0 (CPOL=0, CPHA=0) ✓

### Verification
- MCU master clock: 80 MHz (typical for STM32L432KC)
- SPI clock with BR=3: 80 MHz / 16 = 5 MHz ✓
- This should be fine for SPI communication

---

## 🟢 ISSUE #6: Pin Configuration Verification

### Current Pin Assignments
- **MCU SPI**:
  - SCK: PB3 → FPGA P10 ✓
  - MISO: PB4 → FPGA P21 ✓
  - MOSI: PB5 (unused, but configured) ✓
  - CE: PA11 → FPGA P19 ✓
  - LOAD: PA5 → FPGA P47 ✓
  - DONE: PA6 → FPGA P48 ✓

### Verification
All pins match constraints file. ✓

---

## 📋 Recommended Action Plan

### Priority 1 (Critical - Must Fix)
1. ✅ Create `readSensorDataPacket15()` function in `STM32L432KC_SPI.c`
2. ✅ Fix byte order parsing in sensor data reading
3. ✅ Update `main_integrated.c` to use 15-byte packet reading

### Priority 2 (Important - Should Fix)
4. ⚠️ Test SPI CE handling pattern (current should work, but verify)
5. ⚠️ Verify SPI clock speed is appropriate (5 MHz should be fine)

### Priority 3 (Nice to Have)
6. 📝 Add error checking for header byte (should be 0xAA)
7. 📝 Add timeout handling for DONE signal wait

---

## 🔍 Comparison with Lab07

### Lab07 Working Pattern
```c
// Lab07: Toggle CE for each byte
for(i = 0; i < 16; i++) {
    digitalWrite(PA11, 1); // CE high
    spiSendReceive(plaintext[i]);
    digitalWrite(PA11, 0); // CE low
}

// Wait for DONE
while(!digitalRead(PA6));

// Read data
for(i = 0; i < 16; i++) {
    digitalWrite(PA11, 1); // CE high
    cyphertext[i] = spiSendReceive(0);
    digitalWrite(PA11, 0); // CE low
}
```

### Current Code Pattern
```c
// Current: Keep CE low for entire transaction
digitalWritePortA(SPI_CE, GPIO_LOW);
for (i = 0; i < 15; i++) {
    packet[i] = (uint8_t)spiSendReceive(0x00);
}
digitalWritePortA(SPI_CE, GPIO_HIGH);
```

**Note**: FPGA SPI slave should handle both patterns, but continuous transaction (CE low) is more efficient.

---

## ✅ Summary

**Critical Issues**: 2
- Packet size mismatch (15 vs 1 vs 32 bytes)
- Byte order mismatch (little-endian vs big-endian parsing)

**Important Issues**: 2
- SPI CE handling pattern (should work, but verify)
- Missing main function for 15-byte packets

**Total Issues Found**: 6 (2 critical, 2 important, 2 minor)

