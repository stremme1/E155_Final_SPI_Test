# Arduino Nano to FPGA Pin Connections

## Required Connections

### SPI Communication Pins (3 wires)

| Arduino Nano Pin | Signal Name | Direction | FPGA Pin | FPGA Signal Name |
|-----------------|-------------|-----------|----------|------------------|
| **D13** | SCK (SPI Clock) | Output | `arduino_sck` | SPI Clock Input |
| **D11** | COPI (MOSI) | Output | `arduino_sdi` | SPI Data Input |
| **D10** | CS (Chip Select) | Output | `arduino_cs_n` | Chip Select Input (active low) |

### Notes:
- **D12 (CIPO/MISO) is NOT needed** - FPGA doesn't send data back to Arduino (read-only mode)
- **GND must be connected** - Common ground between Arduino and FPGA
- **VCC/3.3V** - Verify voltage levels match (Arduino Nano typically 5V, but check your board)

## Connection Summary

```
Arduino Nano          →    FPGA
─────────────────────────────────
D13 (SCK)            →    arduino_sck
D11 (COPI/MOSI)      →    arduino_sdi
D10 (CS)             →    arduino_cs_n
GND                  →    GND
```

## Signal Descriptions

### D13 - SCK (SPI Clock)
- **Function**: SPI serial clock
- **Frequency**: 100kHz (set in Arduino code: `SPISettings(100000, MSBFIRST, SPI_MODE0)`)
- **Direction**: Arduino → FPGA
- **FPGA**: Clocked on `posedge sck` to receive data

### D11 - COPI/MOSI (SPI Data)
- **Function**: SPI Controller Out Peripheral In (Master Out Slave In)
- **Direction**: Arduino → FPGA
- **FPGA**: Data sampled on `posedge sck` (SPI Mode 0)
- **Format**: MSB first, 8 bits per byte, 16 bytes per packet

### D10 - CS (Chip Select)
- **Function**: Chip select (active LOW)
- **Direction**: Arduino → FPGA
- **Behavior**: 
  - HIGH = FPGA deselected (idle)
  - LOW = FPGA selected (Arduino sending data)
- **FPGA**: Active low (`cs_n`), transaction starts when CS goes LOW

## Arduino Code Configuration

The Arduino code is already configured correctly:
```cpp
#define FPGA_SPI_CS 10  // D10 pin

// Default SPI pins (hardware SPI on Arduino Nano):
// D11 = COPI/MOSI
// D12 = CIPO/MISO (not used)
// D13 = SCK

SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
digitalWrite(FPGA_SPI_CS, LOW);  // Select FPGA
// ... send data ...
digitalWrite(FPGA_SPI_CS, HIGH); // Deselect FPGA
```

## FPGA Pin Assignments Needed

You need to add these to your FPGA constraint file (`.pcf`):

```
# Arduino SPI Interface (FPGA is slave to Arduino)
set_io arduino_sck   PXX  # Connect to Arduino D13
set_io arduino_sdi   PXX  # Connect to Arduino D11
set_io arduino_cs_n  PXX  # Connect to Arduino D10
```

**Replace `PXX` with actual FPGA pin numbers** based on:
1. Your FPGA board schematic
2. Available GPIO pins
3. Physical wiring you've done

## Voltage Level Compatibility

**Important**: Check voltage levels!

- **Arduino Nano**: Typically 5V logic levels
- **FPGA (iCE40UP5k)**: Typically 3.3V logic levels

**If Arduino is 5V and FPGA is 3.3V:**
- You may need level shifters or voltage dividers
- Or use a 3.3V Arduino-compatible board
- Check your specific board's voltage levels

## Testing Connections

Once connected, use the diagnostic LEDs to verify:

1. **led_cs_detected**: Should blink every ~50ms when Arduino sends packets
   - If never lights: Check D10 → arduino_cs_n connection

2. **led_packet_received**: Should blink every ~50ms when packet is captured
   - If never lights: Check D13 (SCK) or D11 (MOSI) connections

3. **led_initialized**: Should turn ON when valid header (0xAA) is received
   - If stays OFF: Check all connections and pin assignments

4. **led_error**: Should turn OFF when valid header is received
   - If stays ON: Invalid header or no data received

## Troubleshooting

### No signals detected:
- Check GND connection (common ground is essential)
- Verify pin assignments in constraint file
- Check for loose connections
- Verify voltage levels match

### CS detected but no packet:
- Check SCK (D13) connection
- Check MOSI (D11) connection
- Verify SPI mode matches (Mode 0)

### Packet received but wrong data:
- Check bit order (should be MSB first)
- Verify SPI mode (should be Mode 0)
- Check for signal integrity issues

