# Hardware Testing Guide - Dual BNO085 Sensor System

## Overview
This guide walks you through testing the dual BNO085 sensor system with MCU SPI communication on hardware.

## Prerequisites
- FPGA board (Lattice iCE40 or compatible)
- STM32L432KC MCU board
- Two BNO085 sensors
- USB programmer for FPGA (e.g., FTDI)
- ST-Link or compatible programmer for MCU
- Oscilloscope/logic analyzer (optional, for debugging)

---

## Part 1: FPGA Setup

### 1.1 Files to Use
- **Top-level module**: `fpga/drum_trigger_top.sv`
- **Constraints file**: `fpga/constraints_dual_sensor.pdc`
- **Dependencies**: All modules in `fpga/` folder

### 1.2 Synthesis Steps

1. **Open your FPGA tool** (Lattice Diamond, Radiant, or your preferred tool)

2. **Create a new project** or open existing project

3. **Add source files**:
   ```
   - drum_trigger_top.sv (top-level)
   - dual_bno085_controller.sv
   - bno085_controller.sv
   - sensor_data_formatter.sv
   - spi_slave_mcu.sv
   - spi_master.sv
   - hsosc_mock.sv (if needed for simulation, not for synthesis)
   ```

4. **Add constraints file**:
   - Use `constraints_dual_sensor.pdc`
   - **IMPORTANT**: Verify pin assignments match your board layout
   - Update pin numbers if your board uses different pins

5. **Set top-level module**: `drum_trigger_top`

6. **Synthesize and place & route**

7. **Generate bitstream** (.bin file)

8. **Program FPGA** using your programmer

### 1.3 Pin Connections - Verify Before Flashing

**BNO085 Sensor 1:**
- SCLK1 → Sensor 1 SCLK
- MOSI1 → Sensor 1 MOSI  
- MISO1 ← Sensor 1 MISO
- CS_N1 → Sensor 1 CS
- PS0_WAKE1 → Sensor 1 PS0
- INT_N1 ← Sensor 1 INT
- BNO085_RST_N1 → Sensor 1 RST

**BNO085 Sensor 2:**
- SCLK2 → Sensor 2 SCLK
- MOSI2 → Sensor 2 MOSI
- MISO2 ← Sensor 2 MISO
- CS_N2 → Sensor 2 CS
- PS0_WAKE2 → Sensor 2 PS0
- INT_N2 ← Sensor 2 INT
- BNO085_RST_N2 → Sensor 2 RST

**MCU SPI Interface:**
- SCLK_MCU ← MCU PB3 (SPI1_SCK)
- MOSI_MCU ← MCU PB5 (SPI1_MOSI) - not used but must be connected
- MISO_MCU → MCU PB4 (SPI1_MISO)
- CS_N_MCU ← MCU PA11 (SPI1_CE)
- DONE → MCU PA6 (GPIO input)
- LOAD ← MCU PA5 (GPIO output)

**Status LEDs:**
- LED_INITIALIZED → LED (indicates both sensors initialized)
- LED_ERROR → LED (indicates error state)
- LED_HEARTBEAT → LED (blinks at ~1Hz)

**Reset:**
- FPGA_RST_N → Reset button (active low, with pull-up)

---

## Part 2: MCU Setup

### 2.1 MCU Code Files
- Main file: `mcu/main_sensor_data.c` (or create new main file)
- SPI functions: `mcu/STM32L432KC_SPI.c` and `STM32L432KC_SPI.h`
- GPIO functions: `mcu/STM32L432KC_GPIO.c` and `STM32L432KC_GPIO.h`

### 2.2 Required MCU Code

**Note:** The existing `main_sensor_data.c` has a `readSensorDataPacket()` function, but it reads 32 bytes and uses big-endian. You need to update it to read 29 bytes with little-endian format.

Update the function to match the FPGA's packet format:

```c
// Read full sensor data packet (29 bytes) - CORRECTED VERSION
// Returns 1 if successful, 0 if timeout
uint8_t readSensorData(uint8_t *packet) {
    uint32_t timeout = 1000000;  // Timeout counter
    uint8_t i;
    
    // Wait for DONE signal (with timeout)
    while(!digitalReadPortA(SPI_DONE)) {
        timeout--;
        if(timeout == 0) {
            return 0;  // Timeout
        }
    }
    
    // Start SPI transaction
    digitalWritePortA(SPI_CE, GPIO_LOW);
    
    // Small delay to ensure CS is stable
    for(volatile int j = 0; j < 10; j++);
    
    // Read all 29 bytes (NOT 32!)
    for(i = 0; i < 29; i++) {
        packet[i] = (uint8_t)spiSendReceive(0x00);
    }
    
    // Wait for SPI to complete
    while(SPI1->SR & SPI_SR_BSY);
    
    // End SPI transaction
    digitalWritePortA(SPI_CE, GPIO_HIGH);
    
    // Acknowledge by toggling LOAD
    digitalWritePortA(SPI_LOAD, GPIO_HIGH);
    for(volatile int j = 0; j < 10; j++);  // Small delay
    digitalWritePortA(SPI_LOAD, GPIO_LOW);
    
    return 1;  // Success
}
```

**IMPORTANT:** The existing `readSensorDataPacket()` in `main_sensor_data.c` reads 32 bytes and uses big-endian. You need to either:
1. Replace it with the function above (29 bytes, little-endian), OR
2. Update the existing function to read 29 bytes and parse as little-endian

### 2.3 Main Loop Example

```c
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"

int main(void) {
    uint8_t sensor_packet[29];
    uint16_t quat1_w, quat1_x, quat1_y, quat1_z;
    uint16_t gyro1_x, gyro1_y, gyro1_z;
    uint16_t quat2_w, quat2_x, quat2_y, quat2_z;
    uint16_t gyro2_x, gyro2_y, gyro2_z;
    
    // Initialize SPI (Mode 0: CPOL=0, CPHA=0)
    initSPI(1, 0, 0);  // Prescaler=1, CPOL=0, CPHA=0
    initSPIControlPins();
    
    // Main loop
    while(1) {
        // Read sensor data packet
        if(readSensorData(sensor_packet)) {
            // Verify header
            if(sensor_packet[0] == 0xAA) {
                // Parse Sensor 1 data (little-endian)
                quat1_w = sensor_packet[1] | (sensor_packet[2] << 8);
                quat1_x = sensor_packet[3] | (sensor_packet[4] << 8);
                quat1_y = sensor_packet[5] | (sensor_packet[6] << 8);
                quat1_z = sensor_packet[7] | (sensor_packet[8] << 8);
                gyro1_x = sensor_packet[9] | (sensor_packet[10] << 8);
                gyro1_y = sensor_packet[11] | (sensor_packet[12] << 8);
                gyro1_z = sensor_packet[13] | (sensor_packet[14] << 8);
                
                // Parse Sensor 2 data
                quat2_w = sensor_packet[15] | (sensor_packet[16] << 8);
                quat2_x = sensor_packet[17] | (sensor_packet[18] << 8);
                quat2_y = sensor_packet[19] | (sensor_packet[20] << 8);
                quat2_z = sensor_packet[21] | (sensor_packet[22] << 8);
                gyro2_x = sensor_packet[23] | (sensor_packet[24] << 8);
                gyro2_y = sensor_packet[25] | (sensor_packet[26] << 8);
                gyro2_z = sensor_packet[27] | (sensor_packet[28] << 8);
                
                // Process sensor data here
                // (e.g., print via UART, use for drum triggering, etc.)
            }
        }
        
        // Small delay between reads
        delay_ms(10);
    }
}
```

### 2.4 Compile and Flash MCU

1. **Open your MCU IDE** (STM32CubeIDE, Keil, etc.)

2. **Add the function** `readSensorData()` to your code

3. **Verify SPI configuration**:
   - SPI Mode: Mode 0 (CPOL=0, CPHA=0)
   - Clock speed: Appropriate for your setup (e.g., 1-10 MHz)
   - MSB first
   - 8-bit data

4. **Verify GPIO pins**:
   - PA5: LOAD (output, push-pull)
   - PA6: DONE (input, pull-up)
   - PA11: SPI_CE (output, push-pull)
   - PB3: SPI1_SCK
   - PB4: SPI1_MISO
   - PB5: SPI1_MOSI

5. **Compile** the project

6. **Flash** to MCU using ST-Link or your programmer

---

## Part 3: Hardware Testing Procedure

### 3.1 Pre-Power Checklist

- [ ] FPGA programmed with `drum_trigger_top`
- [ ] MCU flashed with sensor data reading code
- [ ] All connections verified (see pin connections above)
- [ ] Both BNO085 sensors connected
- [ ] Power supplies connected (3.3V for sensors, appropriate voltage for FPGA/MCU)
- [ ] Ground connections verified (common ground between all devices)

### 3.2 Power-Up Sequence

1. **Power on FPGA board**
   - Check that `led_heartbeat` starts blinking (~1Hz)
   - This indicates the FPGA is running

2. **Power on MCU board**
   - MCU should start executing code

3. **Observe LEDs**:
   - `led_heartbeat`: Should blink (system running)
   - `led_initialized`: Should turn ON after ~2-3 seconds (both sensors initialized)
   - `led_error`: Should stay OFF (no errors)

### 3.3 Initialization Phase

**Expected behavior:**
- FPGA initializes both BNO085 sensors
- Initialization takes ~2-3 seconds
- `led_initialized` turns ON when both sensors are ready
- If `led_error` turns ON, check sensor connections

**Troubleshooting initialization:**
- If `led_initialized` never turns ON:
  - Check sensor power (3.3V)
  - Verify SPI connections (SCLK, MOSI, MISO, CS)
  - Check PS0_WAKE and RST connections
  - Verify INT_N connections
  - Check with oscilloscope/logic analyzer

### 3.4 Data Transmission Testing

**Expected behavior:**
- MCU polls DONE signal
- When data is ready, DONE goes high
- MCU reads 29 bytes via SPI
- MCU toggles LOAD to acknowledge
- Process repeats

**Verification methods:**

1. **LED observation**:
   - `led_initialized` should stay ON
   - `led_heartbeat` should keep blinking

2. **Oscilloscope/Logic Analyzer**:
   - Monitor DONE signal (PA6) - should pulse when data ready
   - Monitor SPI signals (SCK, MISO, CS) - should see activity
   - Monitor LOAD signal (PA5) - should toggle after each read

3. **MCU Debug Output** (if UART available):
   - Print received packet header (should be 0xAA)
   - Print sensor data values
   - Check for reasonable quaternion values (typically 0x0000-0x7FFF)
   - Check for reasonable gyroscope values

### 3.5 Data Validation

**Check received data:**
- Header byte should always be `0xAA`
- Quaternion values should be in reasonable range
- Gyroscope values should change when sensors move
- Both sensors should provide data

**Expected packet structure:**
```
Byte 0: 0xAA (header)
Bytes 1-14: Sensor 1 data (W,X,Y,Z quat + X,Y,Z gyro)
Bytes 15-28: Sensor 2 data (W,X,Y,Z quat + X,Y,Z gyro)
```

---

## Part 4: Troubleshooting

### 4.1 FPGA Issues

**Problem: No LED activity**
- Check FPGA programming
- Verify clock source (HSOSC)
- Check reset signal

**Problem: LED_ERROR is ON**
- One or both sensors failed to initialize
- Check sensor connections
- Verify sensor power supply
- Check SPI signals with scope

**Problem: LED_INITIALIZED never turns ON**
- Sensors not initializing
- Check sensor connections
- Verify sensor power (3.3V)
- Check PS0_WAKE and RST signals

### 4.2 MCU Issues

**Problem: MCU never receives data**
- Check DONE signal connection (PA6)
- Verify MCU is polling DONE correctly
- Check SPI connections
- Verify SPI mode (must be Mode 0)

**Problem: MCU receives wrong data**
- Verify header byte (should be 0xAA)
- Check SPI clock phase/polarity (Mode 0)
- Verify byte order (little-endian)
- Check timing - may need delays

**Problem: MCU times out waiting for DONE**
- FPGA may not be asserting DONE
- Check DONE signal connection
- Verify sensors are initialized (LED_INITIALIZED ON)
- Check if data_ready is being asserted in FPGA

### 4.3 Sensor Issues

**Problem: Sensor 1 not working**
- Check Sensor 1 SPI connections
- Verify Sensor 1 power
- Check PS0_WAKE1 and RST_N1
- Verify INT_N1 connection

**Problem: Sensor 2 not working**
- Check Sensor 2 SPI connections
- Verify Sensor 2 power
- Check PS0_WAKE2 and RST_N2
- Verify INT_N2 connection

**Problem: Both sensors not working**
- Check common connections (power, ground)
- Verify FPGA is running
- Check reset signals

### 4.4 SPI Communication Issues

**Problem: No SPI activity**
- Verify CS_N_MCU connection
- Check SPI clock (SCLK_MCU)
- Verify MISO_MCU connection
- Check SPI mode configuration

**Problem: SPI data corruption**
- Check signal integrity (use scope)
- Verify timing (may need delays)
- Check for noise/crosstalk
- Verify ground connections

---

## Part 5: Expected Results

### 5.1 Normal Operation

- **LED_INITIALIZED**: ON (after ~2-3 seconds)
- **LED_ERROR**: OFF
- **LED_HEARTBEAT**: Blinking at ~1Hz
- **MCU receives data**: Every ~10-50ms (depending on sensor update rate)
- **Header byte**: Always 0xAA
- **Sensor data**: Reasonable values that change with sensor movement

### 5.2 Data Rate

- BNO085 sensors typically update at 20-100 Hz
- MCU should read data packets at similar rate
- If MCU reads faster, it will wait for DONE signal
- If MCU reads slower, FPGA will buffer data

---

## Part 6: Next Steps

Once basic communication is working:

1. **Add UART debugging** to MCU to print received data
2. **Add data validation** (check header, reasonable ranges)
3. **Implement your application logic** (drum triggering, etc.)
4. **Optimize timing** if needed (add delays, adjust SPI speed)
5. **Add error handling** (timeouts, retries, etc.)

---

## Quick Reference

### Pin Summary

**MCU → FPGA:**
- PA5: LOAD (GPIO output)
- PA11: SPI_CE (SPI chip select)
- PB3: SPI1_SCK (SPI clock)
- PB5: SPI1_MOSI (SPI master out - not used)

**FPGA → MCU:**
- PA6: DONE (GPIO input)
- PB4: SPI1_MISO (SPI master in - data)

**SPI Configuration:**
- Mode: 0 (CPOL=0, CPHA=0)
- Speed: 1-10 MHz (adjust as needed)
- Format: 8-bit, MSB first

### Packet Format
- 29 bytes total
- Byte 0: 0xAA (header)
- Bytes 1-14: Sensor 1 (7 values × 2 bytes)
- Bytes 15-28: Sensor 2 (7 values × 2 bytes)
- All values little-endian

---

Good luck with your testing! 🚀

