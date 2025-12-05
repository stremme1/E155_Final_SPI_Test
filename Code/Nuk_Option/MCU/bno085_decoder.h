#ifndef BNO085_DECODER_H
#define BNO085_DECODER_H

#include <stdint.h>

// Quaternion structure (from BNO085, Q14 format)
typedef struct {
    int16_t w, x, y, z;  // Q14 format (divide by 16384.0 to get float)
} quaternion_t;

// Gyroscope structure (from BNO085)
typedef struct {
    int16_t x, y, z;
} gyroscope_t;

// Euler angles (in degrees)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_t;

// Convert BNO085 quaternion (Q14 format) to Euler angles (degrees)
// Based on Code_for_C_imp/lib/src/bno055.c bno055_quaternion_to_euler()
void quaternion_to_euler(int16_t qw, int16_t qx, int16_t qy, int16_t qz, euler_t *euler);

// Normalize yaw to 0-360 range
// Based on Code_for_C_imp/main.c normalizeYaw()
float normalize_yaw(float yaw);

#endif // BNO085_DECODER_H

