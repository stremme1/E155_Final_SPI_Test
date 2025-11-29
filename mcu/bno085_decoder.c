#include "bno085_decoder.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Convert BNO085 quaternion (Q14 format) to Euler angles (degrees)
// Based on Code_for_C_imp/lib/src/bno055.c bno055_quaternion_to_euler()
// BNO085 quaternion is in Q14 format: divide by 16384.0 to get float
void quaternion_to_euler(int16_t qw, int16_t qx, int16_t qy, int16_t qz, euler_t *euler) {
    // Convert Q14 format to float
    float w = (float)qw / 16384.0f;
    float x = (float)qx / 16384.0f;
    float y = (float)qy / 16384.0f;
    float z = (float)qz / 16384.0f;
    
    // Convert quaternion to Euler angles (same formula as bno055_quaternion_to_euler)
    euler->roll = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    euler->pitch = asin(2.0f * (w * y - z * x));
    euler->yaw = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    
    // Convert to degrees
    euler->roll = euler->roll * 180.0f / M_PI;
    euler->pitch = euler->pitch * 180.0f / M_PI;
    euler->yaw = euler->yaw * 180.0f / M_PI;
}

// Normalize yaw to 0-360 range
// Based on Code_for_C_imp/main.c normalizeYaw()
float normalize_yaw(float yaw) {
    // Use floating point modulo to wrap yaw
    yaw = fmod(yaw, 360.0f);
    // Ensure yaw is positive
    if (yaw < 0) {
        yaw += 360.0f;
    }
    return yaw;
}

