// Mock FPGA SPI Slave for MCU Tests
// Simulates FPGA SPI slave behavior for MCU unit tests
// Can be used to test MCU SPI master code

#include <stdint.h>
#include <stdio.h>

// Mock FPGA SPI slave state
static uint8_t mock_packet[32] = {
    0xAA,  // Header
    // Sensor 1 Quaternion (MSB,LSB)
    0x40, 0x00,  // W
    0x10, 0x00,  // X
    0x20, 0x00,  // Y
    0x30, 0x00,  // Z
    // Sensor 1 Gyroscope (MSB,LSB)
    0x00, 0x64,  // X = 100
    0x00, 0xC8,  // Y = 200
    0x01, 0x2C,  // Z = 300
    0x03,  // Flags (both valid)
    // Sensor 2 Quaternion (MSB,LSB)
    0x50, 0x00,  // W
    0x11, 0x00,  // X
    0x22, 0x00,  // Y
    0x33, 0x00,  // Z
    // Sensor 2 Gyroscope (MSB,LSB)
    0x01, 0x90,  // X = 400
    0x01, 0xF4,  // Y = 500
    0x02, 0x58,  // Z = 600
    0x03,  // Flags (both valid)
    0x00   // Reserved
};

static uint8_t mock_done = 0;
static uint8_t mock_load = 0;

// Set mock packet data
void mock_fpga_set_packet(uint8_t *packet) {
    for (int i = 0; i < 32; i++) {
        mock_packet[i] = packet[i];
    }
    mock_done = 1;  // Data ready
}

// Get mock packet (simulates FPGA sending data)
void mock_fpga_get_packet(uint8_t *packet) {
    for (int i = 0; i < 32; i++) {
        packet[i] = mock_packet[i];
    }
}

// Simulate DONE signal
uint8_t mock_fpga_get_done(void) {
    return mock_done;
}

// Simulate LOAD signal (MCU acknowledges)
void mock_fpga_set_load(uint8_t value) {
    mock_load = value;
    if (value == 1) {
        mock_done = 0;  // Reset DONE when LOAD goes high
    }
}

// Reset mock FPGA
void mock_fpga_reset(void) {
    mock_done = 0;
    mock_load = 0;
}

