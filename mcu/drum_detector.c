#include "drum_detector.h"
#include "bno085_decoder.h"
#include <stdint.h>

// Detect drum trigger from sensor data
// Based on Code_for_C_imp/main.c drum detection logic
uint8_t detect_drum_trigger(
    euler_t *euler1,
    int16_t gyro1_y,
    int16_t gyro1_z,
    euler_t *euler2,
    int16_t gyro2_y,
    int16_t gyro2_z,
    drum_detector_state_t *state,
    yaw_offset_t *offsets
) {
    float yaw1, yaw2, pitch1, pitch2;
    
    // Apply yaw offsets and normalize
    yaw1 = normalize_yaw(euler1->yaw - offsets->yaw_offset1);
    yaw2 = normalize_yaw(euler2->yaw - offsets->yaw_offset2);
    pitch1 = euler1->pitch;
    pitch2 = euler2->pitch;
    
    // Right hand logic (Sensor 1)
    // if yaw in the range of 20-120 then play snare drum
    if (yaw1 >= 20.0f && yaw1 <= 120.0f) {
        if (gyro1_y < -2500 && !state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 1;
            return DRUM_CODE_SNARE;
        } else if (gyro1_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    // if yaw in the range of 340 - 20 then play high tom
    else if ((yaw1 >= 340.0f) || (yaw1 <= 20.0f)) {
        if (gyro1_y < -2500 && !state->printed_for_gyro1_y) {
            // if pitch over 55 degrees play crash cymbal
            if (pitch1 > 50.0f) {
                state->printed_for_gyro1_y = 1;
                return DRUM_CODE_CRASH;
            } else {
                state->printed_for_gyro1_y = 1;
                return DRUM_CODE_HIGH_TOM;
            }
        } else if (gyro1_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    // if yaw in the range of 305-340 then play mid tom
    else if (yaw1 >= 305.0f && yaw1 <= 340.0f) {
        if (gyro1_y < -2500 && !state->printed_for_gyro1_y) {
            // if pitch over 55 degrees play ride cymbal
            if (pitch1 > 50.0f) {
                state->printed_for_gyro1_y = 1;
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro1_y = 1;
                return DRUM_CODE_MID_TOM;
            }
        } else if (gyro1_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    // if yaw in the range of 200-305 then play floor tom
    else if (yaw1 >= 200.0f && yaw1 <= 305.0f) {
        if (gyro1_y < -2500 && !state->printed_for_gyro1_y) {
            if (pitch1 > 30.0f) {
                state->printed_for_gyro1_y = 1;
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro1_y = 1;
                return DRUM_CODE_FLOOR_TOM;
            }
        } else if (gyro1_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    
    // Left hand logic (Sensor 2)
    // if yaw in the range of 350-100 then play snare drum or hi-hat
    if ((yaw2 >= 350.0f) || (yaw2 <= 100.0f)) {
        if (gyro2_y < -2500 && !state->printed_for_gyro2_y) {
            // if facing up and not rotating fast around z axis
            if (pitch2 > 30.0f && gyro2_z > -2000) {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_HIHAT;
            } else {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_SNARE;
            }
        } else if (gyro2_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    // if yaw in the range of 325-350 then play high tom or crash cymbal
    else if (yaw2 >= 325.0f && yaw2 <= 350.0f) {
        if (gyro2_y < -2500 && !state->printed_for_gyro2_y) {
            // if pitch over 55 degrees play crash cymbal
            if (pitch2 > 50.0f) {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_CRASH;
            } else {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_HIGH_TOM;
            }
        } else if (gyro2_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    // if yaw in the range of 300-325 then play mid tom or ride cymbal
    else if (yaw2 >= 300.0f && yaw2 <= 325.0f) {
        if (gyro2_y < -2500 && !state->printed_for_gyro2_y) {
            // if pitch over 55 degrees play ride cymbal
            if (pitch2 > 50.0f) {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_MID_TOM;
            }
        } else if (gyro2_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    // if yaw in the range of 200-300 then play floor tom
    else if (yaw2 >= 200.0f && yaw2 <= 300.0f) {
        if (gyro2_y < -2500 && !state->printed_for_gyro2_y) {
            if (pitch2 > 30.0f) {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro2_y = 1;
                return DRUM_CODE_FLOOR_TOM;
            }
        } else if (gyro2_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    
    // No trigger detected
    return 0xFF;
}

// Update yaw offsets for calibration
// Based on Code_for_C_imp/main.c calibration button logic
void update_yaw_offsets(euler_t *euler1, euler_t *euler2, yaw_offset_t *offsets) {
    offsets->yaw_offset1 = euler1->yaw;
    offsets->yaw_offset2 = euler2->yaw;
}

