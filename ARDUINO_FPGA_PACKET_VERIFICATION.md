# Arduino-FPGA Packet Format Verification

## Packet Format Comparison

### Arduino Side (ARDUINO_SENSOR_BRIDGE.ino)

**Packet Array (16 bytes):**
```cpp
packet[0]  = 0xAA;                    // Header
packet[1]  = (roll >> 8) & 0xFF;     // Roll MSB
packet[2]  = roll & 0xFF;             // Roll LSB
packet[3]  = (pitch >> 8) & 0xFF;    // Pitch MSB
packet[4]  = pitch & 0xFF;            // Pitch LSB
packet[5]  = (yaw >> 8) & 0xFF;      // Yaw MSB
packet[6]  = yaw & 0xFF;              // Yaw LSB
packet[7]  = (gyro_x >> 8) & 0xFF;   // Gyro X MSB
packet[8]  = gyro_x & 0xFF;           // Gyro X LSB
packet[9]  = (gyro_y >> 8) & 0xFF;   // Gyro Y MSB
packet[10] = gyro_y & 0xFF;           // Gyro Y LSB
packet[11] = (gyro_z >> 8) & 0xFF;   // Gyro Z MSB
packet[12] = gyro_z & 0xFF;           // Gyro Z LSB
packet[13] = flags;                   // Flags (bit 0=Euler valid, bit 1=Gyro valid)
packet[14] = 0x00;                    // Reserved
packet[15] = 0x00;                    // Reserved
```

**SPI Transmission:**
- `SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0))`
- `SPI.transfer(packet[i])` for i = 0 to 15
- **MSB First**: Bits sent in order: bit 7, bit 6, ..., bit 0
- **SPI Mode 0**: Clock idle LOW, sample on rising edge

### FPGA Side (arduino_spi_slave.sv)

**Packet Reception:**
- Receives on `posedge sck` (rising edge) - matches SPI Mode 0
- **MSB First**: First bit received is bit 7 (MSB)
- Bit shifting: `rx_shift <= {rx_shift[6:0], sdi}` (left shift, new bit in LSB)
- After 8 bits: `packet_buffer[byte_count] <= {rx_shift[6:0], sdi}`

**Packet Parsing:**
```systemverilog
header <= packet_snapshot[0];
roll   <= {packet_snapshot[1], packet_snapshot[2]};   // {MSB, LSB}
pitch  <= {packet_snapshot[3], packet_snapshot[4]};  // {MSB, LSB}
yaw    <= {packet_snapshot[5], packet_snapshot[6]};  // {MSB, LSB}
gyro_x <= {packet_snapshot[7], packet_snapshot[8]};  // {MSB, LSB}
gyro_y <= {packet_snapshot[9], packet_snapshot[10]}; // {MSB, LSB}
gyro_z <= {packet_snapshot[11], packet_snapshot[12]};// {MSB, LSB}
flags  <= packet_snapshot[13];
```

## Verification

### ✅ Byte Order: CORRECT
- Arduino sends bytes 0-15 in order
- FPGA receives and stores in `packet_buffer[0]` through `packet_buffer[15]`
- **Match**: ✓

### ✅ 16-bit Value Packing: CORRECT
- **Arduino**: `packet[1] = MSB, packet[2] = LSB` (big-endian)
- **FPGA**: `roll = {packet_snapshot[1], packet_snapshot[2]}` = `{MSB, LSB}`
- **Match**: ✓

### ✅ Bit Order (MSB First): CORRECT
- **Arduino**: `SPI.transfer()` with `MSBFIRST` sends bits 7→6→5→4→3→2→1→0
- **FPGA**: Receives first bit (bit 7) in LSB position, then left-shifts:
  - Bit 7: `rx_shift = 00000001` (if bit 7 = 1)
  - Bit 6: `rx_shift = 00000010` (shifted left)
  - ...
  - Bit 0: `rx_shift = 10101010` (complete byte)
- **Match**: ✓

### ✅ SPI Mode: CORRECT
- **Arduino**: `SPI_MODE0` (CPOL=0, CPHA=0) - clock idle LOW, sample on rising edge
- **FPGA**: Samples on `posedge sck` (rising edge)
- **Match**: ✓

### ✅ Flags Byte: CORRECT
- **Arduino**: `packet[13] = flags` where bit 0 = Euler valid, bit 1 = Gyro valid
- **FPGA**: `flags <= packet_snapshot[13]`
- **Match**: ✓

## Example: Receiving 0xAA (Header)

**Arduino sends:**
- Byte 0 = 0xAA = `10101010` (binary)
- SPI.transfer() sends: bit 7=1, bit 6=0, bit 5=1, bit 4=0, bit 3=1, bit 2=0, bit 1=1, bit 0=0

**FPGA receives:**
- Bit 7 (1): `rx_shift = 00000001`
- Bit 6 (0): `rx_shift = 00000010`
- Bit 5 (1): `rx_shift = 00000101`
- Bit 4 (0): `rx_shift = 00001010`
- Bit 3 (1): `rx_shift = 00010101`
- Bit 2 (0): `rx_shift = 00101010`
- Bit 1 (1): `rx_shift = 01010101`
- Bit 0 (0): `rx_shift = 10101010` ✓

**Result**: `packet_buffer[0] = 0xAA` ✓

## Example: Receiving Roll = 1000 (0x03E8)

**Arduino sends:**
- `roll = 1000 = 0x03E8`
- `packet[1] = (0x03E8 >> 8) & 0xFF = 0x03`
- `packet[2] = 0x03E8 & 0xFF = 0xE8`
- Sends: `0x03` then `0xE8`

**FPGA receives:**
- `packet_snapshot[1] = 0x03` (MSB)
- `packet_snapshot[2] = 0xE8` (LSB)
- `roll = {0x03, 0xE8} = 0x03E8 = 1000` ✓

## Conclusion

✅ **ALL VERIFIED**: The Arduino packing and FPGA reading are **CORRECTLY MATCHED**:
- Byte order: ✓
- 16-bit value format (MSB,LSB): ✓
- Bit order (MSB first): ✓
- SPI Mode 0: ✓
- Flags byte: ✓

The packet format is consistent between Arduino and FPGA.

