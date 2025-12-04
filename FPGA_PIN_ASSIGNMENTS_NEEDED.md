# FPGA Pin Assignments Required

## Critical Issue: Missing Arduino SPI Pin Assignments

The constraint file (`Old_SPI_test_xa/constraints.pcf`) does **NOT** include pin assignments for the Arduino SPI interface. This means the FPGA synthesis tool doesn't know which physical pins to use for:

- `arduino_sck` - SPI clock from Arduino
- `arduino_sdi` - SPI data in (MOSI) from Arduino  
- `arduino_cs_n` - Chip select from Arduino (active low)

## Required Pin Assignments

Add these lines to your FPGA constraint file (`.pcf` or `.sdc`):

```
# Arduino SPI Interface (FPGA is slave to Arduino)
set_io arduino_sck   PXX  # Replace PXX with actual FPGA pin for SCK
set_io arduino_sdi   PXX  # Replace PXX with actual FPGA pin for MOSI
set_io arduino_cs_n  PXX  # Replace PXX with actual FPGA pin for CS
```

## Diagnostic LED Pin Assignments

Also add pin assignments for the new diagnostic LEDs:

```
# Diagnostic LEDs (for debugging SPI reception)
set_io led_cs_detected     PXX  # High when CS is low (Arduino sending)
set_io led_packet_received PXX  # High when packet_valid is true
```

## How to Find Correct Pin Numbers

1. **Check your FPGA board schematic** - Find which pins are available for GPIO
2. **Check your wiring** - See which physical pins you connected the Arduino SPI signals to
3. **Use FPGA vendor tools** - Most FPGA tools have pin assignment GUIs

## Expected Behavior After Pin Assignment

Once pins are correctly assigned:

1. **led_cs_detected** should blink every ~50ms (when Arduino sends packet)
   - If this LED never lights: CS signal not reaching FPGA
   
2. **led_packet_received** should blink every ~50ms (when packet is captured)
   - If this LED never lights: Either CS not working OR SCK not working
   
3. **led_initialized** should turn ON (when valid header received)
   - If this stays OFF: Header not being received correctly
   
4. **led_error** should turn OFF (when valid header received)
   - If this stays ON: Invalid header or no data received

## Current Status

- ✅ Arduino is sending valid data (confirmed via Serial Monitor)
- ❌ FPGA pin assignments missing (CRITICAL)
- ❌ Diagnostic LEDs added but need pin assignments
- ❓ Physical wiring unknown (needs verification)

## Next Steps

1. **Add pin assignments** to constraint file for:
   - `arduino_sck`
   - `arduino_sdi`
   - `arduino_cs_n`
   - `led_cs_detected`
   - `led_packet_received`

2. **Re-synthesize FPGA** with new pin assignments

3. **Test diagnostic LEDs**:
   - If `led_cs_detected` blinks: CS signal is working
   - If `led_packet_received` blinks: Packet capture is working
   - If both blink but `led_error` is ON: Header mismatch (check wiring)

4. **Verify physical connections** match pin assignments

