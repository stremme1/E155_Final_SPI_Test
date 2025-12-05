#include "drum_detector.h"
#include "bno085_decoder.h"
#include "debug_print.h"
#include <stdint.h>

// Detect drum trigger from RIGHT HAND sensor data only
// Based on Code_for_C_imp/main.c drum detection logic for right hand
uint8_t detect_drum_trigger_right(
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
    debug_printf("[TRIGGER] Right Hand: yaw=%.2f pitch=%.2f gyro_y=%d gyro_z=%d\r\n", yaw, pitch, gyro_y, gyro_z);
    
    // Right hand logic (Sensor 1)
    // if yaw in the range of 20-120 then play snare drum
    if (yaw >= 20.0f && yaw <= 120.0f) {
        debug_print("[TRIGGER] Right hand in SNARE zone (yaw 20-120)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 1;
            debug_print("[TRIGGER] -> SNARE triggered (gyro_y < -2500)\r\n");
            return DRUM_CODE_SNARE;
        } else if (gyro_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    // if yaw in the range of 340 - 20 then play high tom
    else if ((yaw >= 340.0f) || (yaw <= 20.0f)) {
        debug_print("[TRIGGER] Right hand in HIGH_TOM/CRASH zone (yaw 340-20)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro1_y) {
            // if pitch over 55 degrees play crash cymbal
            if (pitch > 50.0f) {
                state->printed_for_gyro1_y = 1;
                debug_print("[TRIGGER] -> CRASH triggered (pitch > 50)\r\n");
                return DRUM_CODE_CRASH;
            } else {
                state->printed_for_gyro1_y = 1;
                debug_print("[TRIGGER] -> HIGH_TOM triggered\r\n");
                return DRUM_CODE_HIGH_TOM;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    // if yaw in the range of 305-340 then play mid tom
    else if (yaw >= 305.0f && yaw <= 340.0f) {
        debug_print("[TRIGGER] Right hand in MID_TOM/RIDE zone (yaw 305-340)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro1_y) {
            // if pitch over 55 degrees play ride cymbal
            if (pitch > 50.0f) {
                state->printed_for_gyro1_y = 1;
                debug_print("[TRIGGER] -> RIDE triggered (pitch > 50)\r\n");
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro1_y = 1;
                debug_print("[TRIGGER] -> MID_TOM triggered\r\n");
                return DRUM_CODE_MID_TOM;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    // if yaw in the range of 200-305 then play floor tom
    else if (yaw >= 200.0f && yaw <= 305.0f) {
        debug_print("[TRIGGER] Right hand in FLOOR_TOM zone (yaw 200-305)\r\n");
        if (gyro_y < -2500 && !state->printed_for_gyro1_y) {
            if (pitch > 30.0f) {
                state->printed_for_gyro1_y = 1;
                debug_print("[TRIGGER] -> RIDE triggered (pitch > 30)\r\n");
                return DRUM_CODE_RIDE;
            } else {
                state->printed_for_gyro1_y = 1;
                debug_print("[TRIGGER] -> FLOOR_TOM triggered\r\n");
                return DRUM_CODE_FLOOR_TOM;
            }
        } else if (gyro_y >= -2500 && state->printed_for_gyro1_y) {
            state->printed_for_gyro1_y = 0;
        }
    }
    
    // No trigger detected
    return 0xFF;
}

// Update yaw offset for right hand calibration
void update_yaw_offset_right(euler_t *euler, float *yaw_offset) {
    *yaw_offset = euler->yaw;
}

