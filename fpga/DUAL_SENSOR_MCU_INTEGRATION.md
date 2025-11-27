# Dual BNO085 Sensor Integration with MCU SPI Communication

## Overview

This implementation provides:
- **Dual BNO085 sensor support** - Two independent BNO085 sensors with separate SPI interfaces
- **MCU SPI communication** - FPGA acts as SPI slave to send sensor data to MCU
- **Data formatting** - Packages sensor data into 29-byte packets for MCU

## Module Structure

### Core Modules

1. **`dual_bno085_controller.sv`** - Manages two BNO085 sensors
   - Instantiates two `bno085_controller` modules
   - Provides unified data ready signal when both sensors have new data
   - Outputs quaternion and gyroscope data from both sensors

2. **`sensor_data_formatter.sv`** - Formats sensor data for MCU
   - Packages data into 29-byte packets:
     - Byte 0: Header (0xAA)
     - Bytes 1-14: Sensor 1 data (W,X,Y,Z quat + X,Y,Z gyro = 7 values × 2 bytes)
     - Bytes 15-28: Sensor 2 data (same format)

3. **`spi_slave_mcu.sv`** - SPI slave for MCU communication
   - SPI Mode 0 (CPOL=0, CPHA=0) - matches MCU configuration
   - Implements DONE/LOAD handshake protocol (Lab07 pattern)
   - Sends data byte-by-byte as MCU requests

4. **`drum_trigger_top.sv`** - Top-level integration module
   - Connects all components
   - Manages reset sequencing for BNO085 sensors
   - Provides status LEDs

## Data Packet Format

Each packet is 29 bytes:

```
Byte 0:    0xAA (Header)
Bytes 1-2: Sensor1 Quat W (LSB, MSB)
Bytes 3-4: Sensor1 Quat X (LSB, MSB)
Bytes 5-6: Sensor1 Quat Y (LSB, MSB)
Bytes 7-8: Sensor1 Quat Z (LSB, MSB)
Bytes 9-10: Sensor1 Gyro X (LSB, MSB)
Bytes 11-12: Sensor1 Gyro Y (LSB, MSB)
Bytes 13-14: Sensor1 Gyro Z (LSB, MSB)
Bytes 15-16: Sensor2 Quat W (LSB, MSB)
Bytes 17-18: Sensor2 Quat X (LSB, MSB)
Bytes 19-20: Sensor2 Quat Y (LSB, MSB)
Bytes 21-22: Sensor2 Quat Z (LSB, MSB)
Bytes 23-24: Sensor2 Gyro X (LSB, MSB)
Bytes 25-26: Sensor2 Gyro Y (LSB, MSB)
Bytes 27-28: Sensor2 Gyro Z (LSB, MSB)
```

## MCU Integration

### MCU Code Modification Required

The existing `readDrumCommand()` function reads a single byte. For sensor data, you need to read 29 bytes. Here's a suggested function:

```c
// Read full sensor data packet (29 bytes)
void readSensorData(uint8_t *packet) {
    uint8_t i;
    
    // Wait for DONE signal
    while(!digitalReadPortA(SPI_DONE)) {
        // Wait - FPGA will assert DONE when data is ready
    }
    
    // Read all 29 bytes
    digitalWritePortA(SPI_CE, GPIO_LOW);
    for(i = 0; i < 29; i++) {
        packet[i] = (uint8_t)spiSendReceive(0x00);
    }
    while(SPI1->SR & SPI_SR_BSY);  // Wait for SPI to complete
    digitalWritePortA(SPI_CE, GPIO_HIGH);
    
    // Acknowledge by toggling LOAD
    digitalWritePortA(SPI_LOAD, GPIO_HIGH);
    digitalWritePortA(SPI_LOAD, GPIO_LOW);
}
```

### MCU Usage Example

```c
uint8_t sensor_packet[29];
uint16_t quat1_w, quat1_x, quat1_y, quat1_z;
uint16_t gyro1_x, gyro1_y, gyro1_z;
uint16_t quat2_w, quat2_x, quat2_y, quat2_z;
uint16_t gyro2_x, gyro2_y, gyro2_z;

// Initialize SPI (Mode 0, CPOL=0, CPHA=0)
initSPI(1, 0, 0);
initSPIControlPins();

// Main loop
while(1) {
    // Read sensor data packet
    readSensorData(sensor_packet);
    
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
        
        // Process sensor data...
    }
}
```

## Pin Assignments

See `constraints_dual_sensor.pdc` for pin assignments. Key pins:

### Sensor 1
- SCLK, MOSI, MISO, CS_N, PS0_WAKE, INT_N, RST_N

### Sensor 2
- SCLK, MOSI, MISO, CS_N, PS0_WAKE, INT_N, RST_N

### MCU SPI Interface
- `sclk_mcu` - MCU SPI clock (PB3)
- `miso_mcu` - FPGA -> MCU data (PB4)
- `cs_n_mcu` - MCU chip select (PA11)
- `done` - FPGA -> MCU DONE signal (PA6)
- `load` - MCU -> FPGA LOAD signal (PA5)

## Protocol Flow

1. **FPGA Side:**
   - Both BNO085 sensors initialize independently
   - When both sensors have new data, `data_ready` is asserted
   - Data formatter packages data into 29-byte packet
   - SPI slave asserts `done` signal to MCU

2. **MCU Side:**
   - MCU polls `done` signal (blocking wait)
   - When `done` is high, MCU starts SPI transaction
   - MCU reads 29 bytes sequentially
   - MCU toggles `load` to acknowledge

3. **FPGA Response:**
   - SPI slave deasserts `done` after first byte
   - Continues sending bytes as MCU requests
   - After 29 bytes sent and `load` toggled, returns to idle

## Status LEDs

- `led_initialized` - ON when both sensors are initialized
- `led_error` - ON if either sensor has an error
- `led_heartbeat` - Blinks at ~1Hz to show system is running

## Notes

- Both sensors must be initialized before data is sent to MCU
- Data is only sent when both sensors have new data available
- SPI Mode 0 is used for MCU communication (matches Lab07 pattern)
- BNO085 sensors use SPI Mode 3 (CPOL=1, CPHA=1)

