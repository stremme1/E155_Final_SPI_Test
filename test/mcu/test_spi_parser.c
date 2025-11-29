#include <stdio.h>
#include <assert.h>
#include <stdint.h>

// Simulate parseSensorDataPacket function for testing
void parseSensorDataPacket(const uint8_t *packet,
                           int16_t *quat1_w, int16_t *quat1_x, int16_t *quat1_y, int16_t *quat1_z,
                           int16_t *gyro1_x, int16_t *gyro1_y, int16_t *gyro1_z,
                           uint8_t *quat1_valid, uint8_t *gyro1_valid,
                           int16_t *quat2_w, int16_t *quat2_x, int16_t *quat2_y, int16_t *quat2_z,
                           int16_t *gyro2_x, int16_t *gyro2_y, int16_t *gyro2_z,
                           uint8_t *quat2_valid, uint8_t *gyro2_valid) {
    // Verify header
    if (packet[0] != 0xAA) {
        *quat1_w = *quat1_x = *quat1_y = *quat1_z = 0;
        *gyro1_x = *gyro1_y = *gyro1_z = 0;
        *quat1_valid = *gyro1_valid = 0;
        *quat2_w = *quat2_x = *quat2_y = *quat2_z = 0;
        *gyro2_x = *gyro2_y = *gyro2_z = 0;
        *quat2_valid = *gyro2_valid = 0;
        return;
    }
    
    // Sensor 1 Quaternion (bytes 1-8, MSB,LSB format)
    *quat1_w = (int16_t)((packet[1] << 8) | packet[2]);
    *quat1_x = (int16_t)((packet[3] << 8) | packet[4]);
    *quat1_y = (int16_t)((packet[5] << 8) | packet[6]);
    *quat1_z = (int16_t)((packet[7] << 8) | packet[8]);
    
    // Sensor 1 Gyroscope (bytes 9-14, MSB,LSB format)
    *gyro1_x = (int16_t)((packet[9] << 8) | packet[10]);
    *gyro1_y = (int16_t)((packet[11] << 8) | packet[12]);
    *gyro1_z = (int16_t)((packet[13] << 8) | packet[14]);
    
    // Sensor 1 Flags (byte 15)
    *quat1_valid = packet[15] & 0x01;
    *gyro1_valid = (packet[15] >> 1) & 0x01;
    
    // Sensor 2 Quaternion (bytes 16-23, MSB,LSB format)
    *quat2_w = (int16_t)((packet[16] << 8) | packet[17]);
    *quat2_x = (int16_t)((packet[18] << 8) | packet[19]);
    *quat2_y = (int16_t)((packet[20] << 8) | packet[21]);
    *quat2_z = (int16_t)((packet[22] << 8) | packet[23]);
    
    // Sensor 2 Gyroscope (bytes 24-29, MSB,LSB format)
    *gyro2_x = (int16_t)((packet[24] << 8) | packet[25]);
    *gyro2_y = (int16_t)((packet[26] << 8) | packet[27]);
    *gyro2_z = (int16_t)((packet[28] << 8) | packet[29]);
    
    // Sensor 2 Flags (byte 30)
    *quat2_valid = packet[30] & 0x01;
    *gyro2_valid = (packet[30] >> 1) & 0x01;
}

// Test SPI packet parsing
void test_spi_parser(void) {
    uint8_t packet[32];
    int16_t quat1_w, quat1_x, quat1_y, quat1_z;
    int16_t gyro1_x, gyro1_y, gyro1_z;
    uint8_t quat1_valid, gyro1_valid;
    int16_t quat2_w, quat2_x, quat2_y, quat2_z;
    int16_t gyro2_x, gyro2_y, gyro2_z;
    uint8_t quat2_valid, gyro2_valid;
    
    printf("=== Test: SPI Packet Parser ===\n");
    
    // Test 1: Valid packet with known values
    printf("\nTest 1: Valid packet parsing\n");
    packet[0] = 0xAA;  // Header
    
    // Sensor 1 quaternion: W=0x4000 (MSB,LSB)
    packet[1] = 0x40; packet[2] = 0x00;
    packet[3] = 0x10; packet[4] = 0x00;  // X
    packet[5] = 0x20; packet[6] = 0x00;  // Y
    packet[7] = 0x30; packet[8] = 0x00;  // Z
    
    // Sensor 1 gyroscope: X=100, Y=200, Z=300 (MSB,LSB)
    packet[9] = 0x00; packet[10] = 0x64;  // X = 100
    packet[11] = 0x00; packet[12] = 0xC8; // Y = 200
    packet[13] = 0x01; packet[14] = 0x2C; // Z = 300
    
    // Sensor 1 flags: both valid
    packet[15] = 0x03;
    
    // Sensor 2 quaternion: W=0x5000
    packet[16] = 0x50; packet[17] = 0x00;
    packet[18] = 0x11; packet[19] = 0x00;
    packet[20] = 0x22; packet[21] = 0x00;
    packet[22] = 0x33; packet[23] = 0x00;
    
    // Sensor 2 gyroscope
    packet[24] = 0x01; packet[25] = 0x90; // X = 400
    packet[26] = 0x01; packet[27] = 0xF4; // Y = 500
    packet[28] = 0x02; packet[29] = 0x58; // Z = 600
    
    // Sensor 2 flags
    packet[30] = 0x03;
    packet[31] = 0x00;  // Reserved
    
    parseSensorDataPacket(packet,
                          &quat1_w, &quat1_x, &quat1_y, &quat1_z,
                          &gyro1_x, &gyro1_y, &gyro1_z,
                          &quat1_valid, &gyro1_valid,
                          &quat2_w, &quat2_x, &quat2_y, &quat2_z,
                          &gyro2_x, &gyro2_y, &gyro2_z,
                          &quat2_valid, &gyro2_valid);
    
    // Verify Sensor 1
    assert(quat1_w == 0x4000);
    assert(quat1_x == 0x1000);
    assert(gyro1_x == 100);
    assert(gyro1_y == 200);
    assert(gyro1_z == 300);
    assert(quat1_valid == 1);
    assert(gyro1_valid == 1);
    printf("  [PASS] Sensor 1 data parsed correctly\n");
    
    // Verify Sensor 2
    assert(quat2_w == 0x5000);
    assert(gyro2_x == 400);
    assert(gyro2_y == 500);
    assert(gyro2_z == 600);
    assert(quat2_valid == 1);
    assert(gyro2_valid == 1);
    printf("  [PASS] Sensor 2 data parsed correctly\n");
    
    // Test 2: Invalid header
    printf("\nTest 2: Invalid header\n");
    packet[0] = 0x55;  // Wrong header
    parseSensorDataPacket(packet,
                          &quat1_w, &quat1_x, &quat1_y, &quat1_z,
                          &gyro1_x, &gyro1_y, &gyro1_z,
                          &quat1_valid, &gyro1_valid,
                          &quat2_w, &quat2_x, &quat2_y, &quat2_z,
                          &gyro2_x, &gyro2_y, &gyro2_z,
                          &quat2_valid, &gyro2_valid);
    
    assert(quat1_w == 0 && quat1_x == 0);
    assert(quat1_valid == 0 && gyro1_valid == 0);
    printf("  [PASS] Invalid header handled correctly (all zeros)\n");
    
    // Test 3: Flag extraction
    printf("\nTest 3: Flag extraction\n");
    packet[0] = 0xAA;
    packet[15] = 0x01;  // Only quat valid
    parseSensorDataPacket(packet,
                          &quat1_w, &quat1_x, &quat1_y, &quat1_z,
                          &gyro1_x, &gyro1_y, &gyro1_z,
                          &quat1_valid, &gyro1_valid,
                          &quat2_w, &quat2_x, &quat2_y, &quat2_z,
                          &gyro2_x, &gyro2_y, &gyro2_z,
                          &quat2_valid, &gyro2_valid);
    
    assert(quat1_valid == 1);
    assert(gyro1_valid == 0);
    printf("  [PASS] Flags extracted correctly\n");
    
    printf("\n=== All Parser Tests PASSED ===\n");
}

int main(void) {
    printf("========================================\n");
    printf("SPI Parser Unit Tests\n");
    printf("========================================\n");
    
    test_spi_parser();
    
    printf("\n========================================\n");
    printf("All Tests PASSED\n");
    printf("========================================\n");
    
    return 0;
}

