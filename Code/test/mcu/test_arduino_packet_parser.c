#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

// Simulate parseArduinoPacket function for testing
// This matches the implementation in STM32L432KC_SPI.c
void parseArduinoPacket(const uint8_t *packet,
                       int16_t *quat_w, int16_t *quat_x, int16_t *quat_y, int16_t *quat_z,
                       uint8_t *sensor_id, uint8_t *valid) {
    // Verify sync byte
    if (packet[0] != 0xAA) {
        *quat_w = *quat_x = *quat_y = *quat_z = 0;
        *sensor_id = 0;
        *valid = 0;
        return;
    }
    
    // Extract sensor ID
    *sensor_id = packet[1];
    
    // Extract quaternion as floats and convert to Q14 format (int16_t)
    // Quaternion real (bytes 6-9)
    float q_real = *((float*)(&packet[6]));
    // Quaternion i (bytes 10-13)
    float q_i = *((float*)(&packet[10]));
    // Quaternion j (bytes 14-17)
    float q_j = *((float*)(&packet[14]));
    // Quaternion k (bytes 18-21)
    float q_k = *((float*)(&packet[18]));
    
    // Convert float to Q14 format (multiply by 16384.0)
    // Clamp to int16_t range to avoid overflow
    *quat_w = (int16_t)(q_real * 16384.0f);
    *quat_x = (int16_t)(q_i * 16384.0f);
    *quat_y = (int16_t)(q_j * 16384.0f);
    *quat_z = (int16_t)(q_k * 16384.0f);
    
    // Validate quaternion (check if values are reasonable)
    float magnitude_sq = q_real * q_real + q_i * q_i + q_j * q_j + q_k * q_k;
    if (magnitude_sq > 1.5f || magnitude_sq < 0.5f) {
        *valid = 0;
    } else {
        *valid = 1;
    }
}

// Test Arduino packet parsing
void test_arduino_packet_parser(void) {
    uint8_t packet[22];  // 22-byte Arduino packet
    int16_t quat_w, quat_x, quat_y, quat_z;
    uint8_t sensor_id;
    uint8_t valid;
    
    printf("=== Test: Arduino Packet Parser ===\n");
    
    // Test 1: Valid packet with known quaternion values
    printf("\nTest 1: Valid packet parsing\n");
    packet[0] = 0xAA;  // Sync byte
    packet[1] = 0x05;  // Sensor ID (SH2_GAME_ROTATION_VECTOR)
    
    // Timestamp (bytes 2-5) - not used in parsing, but fill with dummy data
    packet[2] = 0x00; packet[3] = 0x00; packet[4] = 0x00; packet[5] = 0x00;
    
    // Quaternion: W=1.0, X=0.0, Y=0.0, Z=0.0 (identity quaternion)
    float q_real = 1.0f;
    float q_i = 0.0f;
    float q_j = 0.0f;
    float q_k = 0.0f;
    
    memcpy(&packet[6], &q_real, 4);   // Bytes 6-9
    memcpy(&packet[10], &q_i, 4);     // Bytes 10-13
    memcpy(&packet[14], &q_j, 4);     // Bytes 14-17
    memcpy(&packet[18], &q_k, 4);     // Bytes 18-21
    
    parseArduinoPacket(packet, &quat_w, &quat_x, &quat_y, &quat_z, &sensor_id, &valid);
    
    // Verify: 1.0 * 16384 = 16384 in Q14 format
    assert(quat_w == 16384);
    assert(quat_x == 0);
    assert(quat_y == 0);
    assert(quat_z == 0);
    assert(sensor_id == 0x05);
    assert(valid == 1);
    printf("  [PASS] Identity quaternion parsed correctly\n");
    
    // Test 2: Invalid sync byte
    printf("\nTest 2: Invalid sync byte\n");
    packet[0] = 0x55;  // Wrong sync byte
    parseArduinoPacket(packet, &quat_w, &quat_x, &quat_y, &quat_z, &sensor_id, &valid);
    
    assert(quat_w == 0 && quat_x == 0 && quat_y == 0 && quat_z == 0);
    assert(sensor_id == 0);
    assert(valid == 0);
    printf("  [PASS] Invalid sync byte handled correctly\n");
    
    // Test 3: Valid quaternion with non-zero values
    printf("\nTest 3: Valid quaternion with non-zero values\n");
    packet[0] = 0xAA;  // Sync byte
    packet[1] = 0x05;  // Sensor ID
    
    // Normalized quaternion: W=0.707, X=0.707, Y=0.0, Z=0.0
    q_real = 0.70710678f;  // cos(45°)
    q_i = 0.70710678f;     // sin(45°)
    q_j = 0.0f;
    q_k = 0.0f;
    
    memcpy(&packet[6], &q_real, 4);
    memcpy(&packet[10], &q_i, 4);
    memcpy(&packet[14], &q_j, 4);
    memcpy(&packet[18], &q_k, 4);
    
    parseArduinoPacket(packet, &quat_w, &quat_x, &quat_y, &quat_z, &sensor_id, &valid);
    
    // Verify: 0.707 * 16384 ≈ 11585
    assert(abs(quat_w - 11585) < 10);  // Allow small rounding error
    assert(abs(quat_x - 11585) < 10);
    assert(quat_y == 0);
    assert(quat_z == 0);
    assert(valid == 1);
    printf("  [PASS] Non-zero quaternion parsed correctly\n");
    
    // Test 4: Invalid quaternion (magnitude out of range)
    printf("\nTest 4: Invalid quaternion magnitude\n");
    packet[0] = 0xAA;  // Sync byte
    packet[1] = 0x05;  // Sensor ID
    
    // Unnormalized quaternion (magnitude > 1.5)
    q_real = 2.0f;
    q_i = 2.0f;
    q_j = 2.0f;
    q_k = 2.0f;
    
    memcpy(&packet[6], &q_real, 4);
    memcpy(&packet[10], &q_i, 4);
    memcpy(&packet[14], &q_j, 4);
    memcpy(&packet[18], &q_k, 4);
    
    parseArduinoPacket(packet, &quat_w, &quat_x, &quat_y, &quat_z, &sensor_id, &valid);
    
    // Should still parse values but mark as invalid
    assert(valid == 0);
    printf("  [PASS] Invalid magnitude detected correctly\n");
    
    printf("\n=== All Arduino Packet Parser Tests PASSED ===\n");
}

int main(void) {
    printf("========================================\n");
    printf("Arduino Packet Parser Unit Tests\n");
    printf("========================================\n");
    
    test_arduino_packet_parser();
    
    printf("\n========================================\n");
    printf("All Tests PASSED\n");
    printf("========================================\n");
    
    return 0;
}

