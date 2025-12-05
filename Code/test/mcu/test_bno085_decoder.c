#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "../../mcu/bno085_decoder.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Test quaternion to Euler conversion
void test_quaternion_to_euler(void) {
    euler_t euler;
    
    printf("=== Test: Quaternion to Euler Conversion ===\n");
    
    // Test 1: Identity quaternion (W=1, X=Y=Z=0) should give 0,0,0
    printf("\nTest 1: Identity quaternion\n");
    quaternion_to_euler(16384, 0, 0, 0, &euler); // Q14: 1.0 = 16384
    printf("  Input: W=16384 (1.0), X=0, Y=0, Z=0\n");
    printf("  Output: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", euler.roll, euler.pitch, euler.yaw);
    assert(fabs(euler.roll) < 1.0);
    assert(fabs(euler.pitch) < 1.0);
    assert(fabs(euler.yaw) < 1.0);
    printf("  [PASS] Identity quaternion gives ~0,0,0\n");
    
    // Test 2: 90 degree rotation around Z axis
    printf("\nTest 2: 90 degree rotation around Z\n");
    // W=cos(45°), Z=sin(45°) in Q14 format
    int16_t w = (int16_t)(cos(M_PI/4) * 16384);
    int16_t z = (int16_t)(sin(M_PI/4) * 16384);
    quaternion_to_euler(w, 0, 0, z, &euler);
    printf("  Input: W=%d, X=0, Y=0, Z=%d\n", w, z);
    printf("  Output: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", euler.roll, euler.pitch, euler.yaw);
    assert(fabs(euler.yaw - 90.0) < 5.0); // Allow 5 degree tolerance
    printf("  [PASS] Yaw is approximately 90 degrees\n");
    
    // Test 3: Known quaternion from Code_for_C_imp
    printf("\nTest 3: Known quaternion values\n");
    quaternion_to_euler(16384, 8192, 4096, 2048, &euler);
    printf("  Input: W=16384, X=8192, Y=4096, Z=2048\n");
    printf("  Output: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", euler.roll, euler.pitch, euler.yaw);
    printf("  [PASS] Conversion completed\n");
}

// Test yaw normalization
void test_normalize_yaw(void) {
    printf("\n=== Test: Yaw Normalization ===\n");
    
    // Test 1: Normal range
    float yaw1 = normalize_yaw(45.0f);
    assert(fabs(yaw1 - 45.0f) < 0.1f);
    printf("  [PASS] 45.0 -> 45.0\n");
    
    // Test 2: Negative value
    float yaw2 = normalize_yaw(-45.0f);
    assert(fabs(yaw2 - 315.0f) < 0.1f);
    printf("  [PASS] -45.0 -> 315.0\n");
    
    // Test 3: Over 360
    float yaw3 = normalize_yaw(450.0f);
    assert(fabs(yaw3 - 90.0f) < 0.1f);
    printf("  [PASS] 450.0 -> 90.0\n");
    
    // Test 4: Exactly 360
    float yaw4 = normalize_yaw(360.0f);
    assert(fabs(yaw4) < 0.1f);
    printf("  [PASS] 360.0 -> 0.0\n");
}

int main(void) {
    printf("========================================\n");
    printf("BNO085 Decoder Unit Tests\n");
    printf("========================================\n");
    
    test_quaternion_to_euler();
    test_normalize_yaw();
    
    printf("\n========================================\n");
    printf("All Tests PASSED\n");
    printf("========================================\n");
    
    return 0;
}

