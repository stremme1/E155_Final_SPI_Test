#include "drum_detector.h"
#include "bno085_decoder.h"
#include "debug_print.h"
#include <stdint.h>

// Detect drum trigger from LEFT HAND sensor data only
// Based on Code_for_C_imp/main.c drum detection logic for left hand
uint8_t detect_drum_trigger_left(
    euler_t *euler,
    int16_t gyro_y,
    int16_t gyro_z,
    drum_detector_state_t *state,
    float *yaw_offset
) {
    float yaw, pitch;
    
    // Apply yaw offset and normalize
    yaw = normalize_yaw(euler->yaw - *yaw_offset);
    pitch = euler->pitch;
    
    // Debug: log input values for trigger detection
    debug_printf("[TRIGGER] Left Hand: yaw=%.2f pitch=%.2f gyro_y=%d gyro_z=%d\r\n", yaw, pitch, gyro_y, gyro_z);
    
    // Left hand logic (Sensor 2)
    // if yaw in the range of 350-100 then play snare drum or hi-hat
    if ((yaw >= 350.0f) || (yaw <= 100.0f)) {
        debug_print("[TRIGGER] Left hand in SNARE/HIHAT zone (yaw 350-100)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro2_y) {
            // if facing up and not rotating fast around z axis
            if (pitch > 30.0f && gyro_z > -2000) {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> HIHAT triggered (pitch > 30, gyro_z > -2000)\r\n");
                return DRUM_CODE_HIHAT;
            } else {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> SNARE triggered\r\n");
                return DRUM_CODE_SNARE;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    // if yaw in the range of 325-350 then play high tom or crash cymbal
    else if (yaw >= 325.0f && yaw <= 350.0f) {
        debug_print("[TRIGGER] Left hand in HIGH_TOM/CRASH zone (yaw 325-350)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro2_y) {
            // if pitch over 55 degrees play crash cymbal
            if (pitch > 50.0f) {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> CRASH triggered (pitch > 50)\r\n");
                return DRUM_CODE_CRASH;
            } else {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> HIGH_TOM triggered\r\n");
                return DRUM_CODE_HIGH_TOM;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    // if yaw in the range of 300-325 then play mid tom or ride cymbal
    else if (yaw >= 300.0f && yaw <= 325.0f) {
        debug_print("[TRIGGER] Left hand in MID_TOM/RIDE zone (yaw 300-325)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro2_y) {
            // if pitch over 55 degrees play ride cymbal
            if (pitch > 50.0f) {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> RIDE triggered (pitch > 50)\r\n");
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> MID_TOM triggered\r\n");
                return DRUM_CODE_MID_TOM;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    // if yaw in the range of 200-300 then play floor tom
    else if (yaw >= 200.0f && yaw <= 300.0f) {
        debug_print("[TRIGGER] Left hand in FLOOR_TOM zone (yaw 200-300)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro2_y) {
            if (pitch > 30.0f) {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> RIDE triggered (pitch > 30)\r\n");
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro2_y = 1;
                debug_print("[TRIGGER] -> FLOOR_TOM triggered\r\n");
                return DRUM_CODE_FLOOR_TOM;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro2_y) {
            state->printed_for_gyro2_y = 0;
        }
    }
    
    // No trigger detected
    return 0xFF;
}

// Update yaw offset for left hand calibration
void update_yaw_offset_left(euler_t *euler, float *yaw_offset) {
    *yaw_offset = euler->yaw;
}

