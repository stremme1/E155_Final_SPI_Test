#ifndef DRUM_DETECTOR_H
#define DRUM_DETECTOR_H

#include <stdint.h>
#include "bno085_decoder.h"

// Drum codes (matches FPGA and Lab4-Final_Project)
#define DRUM_CODE_SNARE      0x00
#define DRUM_CODE_HIHAT      0x01
#define DRUM_CODE_KICK       0x02
#define DRUM_CODE_HIGH_TOM   0x03
#define DRUM_CODE_MID_TOM    0x04
#define DRUM_CODE_CRASH      0x05
#define DRUM_CODE_RIDE       0x06
#define DRUM_CODE_FLOOR_TOM  0x07

// Drum detection state (for debouncing)
typedef struct {
    uint8_t printed_for_gyro1_y;
    uint8_t printed_for_gyro2_y;
} drum_detector_state_t;

// Yaw offset for calibration
typedef struct {
    float yaw_offset1;  // Right hand yaw offset
    float yaw_offset2;  // Left hand yaw offset
} yaw_offset_t;

// Detect drum trigger from sensor data
// Returns drum code (0-7) if trigger detected, 0xFF if no trigger
// Based on Code_for_C_imp/main.c drum detection logic
uint8_t detect_drum_trigger(
    // Sensor 1 (Right Hand)
    euler_t *euler1,
    int16_t gyro1_y,
    int16_t gyro1_z,
    // Sensor 2 (Left Hand)
    euler_t *euler2,
    int16_t gyro2_y,
    int16_t gyro2_z,
    // State and offsets
    drum_detector_state_t *state,
    yaw_offset_t *offsets
);

// Update yaw offsets for calibration
// Based on Code_for_C_imp/main.c calibration button logic
void update_yaw_offsets(euler_t *euler1, euler_t *euler2, yaw_offset_t *offsets);

#endif // DRUM_DETECTOR_H

