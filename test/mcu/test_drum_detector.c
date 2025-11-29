#include <stdio.h>
#include <assert.h>
#include "../../mcu/drum_detector.h"
#include "../../mcu/bno085_decoder.h"

// Test drum detection with known values
void test_drum_detection(void) {
    euler_t euler1, euler2;
    drum_detector_state_t state = {0, 0};
    yaw_offset_t offsets = {0.0f, 0.0f};
    uint8_t drum_code;
    
    printf("=== Test: Drum Detection ===\n");
    
    // Test 1: Snare drum (yaw 20-120, gyro_y < -2500)
    printf("\nTest 1: Snare drum detection\n");
    euler1.yaw = 60.0f;  // In range 20-120
    euler1.pitch = 0.0f;
    euler1.roll = 0.0f;
    euler2.yaw = 0.0f;
    euler2.pitch = 0.0f;
    euler2.roll = 0.0f;
    
    drum_code = detect_drum_trigger(&euler1, -3000, 0, &euler2, 0, 0, &state, &offsets);
    assert(drum_code == DRUM_CODE_SNARE);
    printf("  [PASS] Snare detected (yaw=60, gyro_y=-3000)\n");
    
    // Test 2: Kick drum (button - not tested here, handled separately)
    
    // Test 3: High tom (yaw 340-20, gyro_y < -2500, pitch <= 50)
    printf("\nTest 2: High tom detection\n");
    state.printed_for_gyro1_y = 0; // Reset state
    euler1.yaw = 10.0f;  // In range 340-20
    euler1.pitch = 30.0f;  // Below 50
    drum_code = detect_drum_trigger(&euler1, -3000, 0, &euler2, 0, 0, &state, &offsets);
    assert(drum_code == DRUM_CODE_HIGH_TOM);
    printf("  [PASS] High tom detected (yaw=10, pitch=30)\n");
    
    // Test 4: Crash cymbal (yaw 340-20, pitch > 50)
    printf("\nTest 3: Crash cymbal detection\n");
    state.printed_for_gyro1_y = 0;
    euler1.yaw = 10.0f;
    euler1.pitch = 60.0f;  // Above 50
    drum_code = detect_drum_trigger(&euler1, -3000, 0, &euler2, 0, 0, &state, &offsets);
    assert(drum_code == DRUM_CODE_CRASH);
    printf("  [PASS] Crash detected (yaw=10, pitch=60)\n");
    
    // Test 5: Hi-hat (left hand, yaw 350-100, pitch > 30, gyro_z > -2000)
    printf("\nTest 4: Hi-hat detection\n");
    state.printed_for_gyro2_y = 0;
    euler2.yaw = 50.0f;  // In range 350-100
    euler2.pitch = 40.0f;  // Above 30
    euler1.yaw = 0.0f;
    drum_code = detect_drum_trigger(&euler1, 0, 0, &euler2, -3000, -1500, &state, &offsets);
    assert(drum_code == DRUM_CODE_HIHAT);
    printf("  [PASS] Hi-hat detected (yaw=50, pitch=40, gyro_z=-1500)\n");
    
    // Test 6: No trigger (gyro_y not below threshold)
    printf("\nTest 5: No trigger (gyro_y above threshold)\n");
    state.printed_for_gyro1_y = 0;
    euler1.yaw = 60.0f;
    drum_code = detect_drum_trigger(&euler1, -2000, 0, &euler2, 0, 0, &state, &offsets);
    assert(drum_code == 0xFF); // No trigger
    printf("  [PASS] No trigger (gyro_y=-2000, threshold=-2500)\n");
    
    printf("\n=== All Drum Detection Tests PASSED ===\n");
}

int main(void) {
    printf("========================================\n");
    printf("Drum Detector Unit Tests\n");
    printf("========================================\n");
    
    test_drum_detection();
    
    printf("\n========================================\n");
    printf("All Tests PASSED\n");
    printf("========================================\n");
    
    return 0;
}

