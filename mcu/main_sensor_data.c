// main_sensor_data.c
// MCU receives raw sensor data from FPGA and processes it using existing C logic
// Based on Code_for_C_imp/main.c drum detection logic
//
// Description: 
// - Receives raw BNO085 sensor data (quaternion + gyro) from FPGA via SPI
// - Processes data using existing C code logic (quaternion→euler, zone detection, etc.)
// - Plays corresponding WAV drum samples using DAC

#include "STM32L432KC_RCC.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_FLASH.h"
#include "STM32L432KC_DAC.h"
#include "STM32L432KC_TIMER.h"
#include "STM32L432KC_SPI.h"
#include <math.h>

// Include drum sample arrays header
#include "wav_arrays/drum_samples.h"

// Drum sample rate (all samples converted to 22.05 kHz)
#define DRUM_SAMPLE_RATE 22050

// Drum command codes (matches C code)
#define DRUM_CODE_SNARE      0x00
#define DRUM_CODE_HIHAT      0x01
#define DRUM_CODE_KICK       0x02
#define DRUM_CODE_HIGH_TOM   0x03
#define DRUM_CODE_MID_TOM    0x04
#define DRUM_CODE_CRASH      0x05
#define DRUM_CODE_RIDE       0x06
#define DRUM_CODE_FLOOR_TOM  0x07

// Sensor data packet structure (32 bytes from FPGA)
// Byte 0-7:   Sensor 1 quaternion (w, x, y, z - MSB,LSB each)
// Byte 8-13:  Sensor 1 gyroscope (x, y, z - MSB,LSB each)
// Byte 14:    Sensor 1 flags (bit 0=quat_valid, bit 1=gyro_valid)
// Byte 15-22: Sensor 2 quaternion (w, x, y, z - MSB,LSB each)
// Byte 23-28: Sensor 2 gyroscope (x, y, z - MSB,LSB each)
// Byte 29:    Sensor 2 flags (bit 0=quat_valid, bit 1=gyro_valid)
// Byte 30:    Buttons (bit 0=kick_btn)
// Byte 31:    Buttons (bit 0=calibrate_btn)

typedef struct {
    // Sensor 1 (Right Hand)
    int16_t quat1_w, quat1_x, quat1_y, quat1_z;
    int16_t gyro1_x, gyro1_y, gyro1_z;
    uint8_t quat1_valid, gyro1_valid;
    
    // Sensor 2 (Left Hand)
    int16_t quat2_w, quat2_x, quat2_y, quat2_z;
    int16_t gyro2_x, gyro2_y, gyro2_z;
    uint8_t quat2_valid, gyro2_valid;
    
    // Buttons
    uint8_t kick_btn, calibrate_btn;
} sensor_data_t;

// Flag to prevent overlapping playback
static volatile uint8_t is_playing = 0;

// Yaw offset for calibration (from C code)
static float yawOffset1 = 0.0f, yawOffset2 = 0.0f;

// Strike detection flags (from C code)
static uint8_t printedForGyro1y = 0;
static uint8_t printedForGyro2y = 0;

// Function to normalize yaw values to 0-360 range (from C code)
float normalizeYaw(float yaw) {
    yaw = fmod(yaw, 360.0f);
    if (yaw < 0) {
        yaw += 360.0f;
    }
    return yaw;
}

// Quaternion to Euler conversion (from C code / Adafruit library)
void quaternion_to_euler(int16_t qw, int16_t qx, int16_t qy, int16_t qz,
                         float *roll, float *pitch, float *yaw) {
    // Convert from Q14 format (divide by 16384)
    float w = (float)qw / 16384.0f;
    float x = (float)qx / 16384.0f;
    float y = (float)qy / 16384.0f;
    float z = (float)qz / 16384.0f;
    
    // Quaternion to Euler conversion (from Adafruit BNO08x library)
    *roll = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * 180.0f / M_PI;
    *pitch = asin(2.0f * (w * y - z * x)) * 180.0f / M_PI;
    *yaw = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / M_PI;
}

// Function to play a drum sample
void play_drum_sample(const int16_t* data, uint32_t length, uint32_t sample_rate) {
    if (data == NULL || length == 0 || is_playing) {
        return;
    }
    is_playing = 1;
    DAC_PlayWAV(data, length, sample_rate);
    is_playing = 0;
}

// Process sensor data and play drum sounds (from C code logic)
void process_sensor_data(sensor_data_t *data) {
    float roll1, pitch1, yaw1;
    float roll2, pitch2, yaw2;
    
    // Handle calibration button
    if (data->calibrate_btn) {
        // Set yaw offset (simplified - would need current yaw reading)
        // For now, just reset offsets
        yawOffset1 = 0.0f;
        yawOffset2 = 0.0f;
    }
    
    // Handle kick button (highest priority)
    if (data->kick_btn) {
        play_drum_sample(kick_sample_data, kick_sample_length, kick_sample_sample_rate);
        return;
    }
    
    // Process Sensor 1 (Right Hand) - only if valid data
    if (data->quat1_valid && data->gyro1_valid) {
        quaternion_to_euler(data->quat1_w, data->quat1_x, data->quat1_y, data->quat1_z,
                           &roll1, &pitch1, &yaw1);
        yaw1 = normalizeYaw(yaw1 - yawOffset1);
        
        // Right hand logic (from C code)
        if (yaw1 >= 20.0f && yaw1 <= 120.0f) {
            if (data->gyro1_y < -2500 && !printedForGyro1y) {
                play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
                printedForGyro1y = 1;
            } else if (data->gyro1_y >= -2500 && printedForGyro1y) {
                printedForGyro1y = 0;
            }
        } else if ((yaw1 >= 340.0f) || (yaw1 <= 20.0f)) {
            if (data->gyro1_y < -2500 && !printedForGyro1y) {
                if (pitch1 > 50.0f) {
                    play_drum_sample(crash_sample_data, crash_sample_length, crash_sample_sample_rate);
                } else {
                    play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
                }
                printedForGyro1y = 1;
            } else if (data->gyro1_y >= -2500 && printedForGyro1y) {
                printedForGyro1y = 0;
            }
        } else if (yaw1 >= 305.0f && yaw1 <= 340.0f) {
            if (data->gyro1_y < -2500 && !printedForGyro1y) {
                if (pitch1 > 50.0f) {
                    play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
                } else {
                    play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
                }
                printedForGyro1y = 1;
            } else if (data->gyro1_y >= -2500 && printedForGyro1y) {
                printedForGyro1y = 0;
            }
        } else if (yaw1 >= 200.0f && yaw1 <= 305.0f) {
            if (data->gyro1_y < -2500 && !printedForGyro1y) {
                if (pitch1 > 30.0f) {
                    play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
                } else {
                    play_drum_sample(tom_low_sample_data, tom_low_sample_length, tom_low_sample_sample_rate);
                }
                printedForGyro1y = 1;
            } else if (data->gyro1_y >= -2500 && printedForGyro1y) {
                printedForGyro1y = 0;
            }
        }
    }
    
    // Process Sensor 2 (Left Hand) - only if valid data
    if (data->quat2_valid && data->gyro2_valid) {
        quaternion_to_euler(data->quat2_w, data->quat2_x, data->quat2_y, data->quat2_z,
                           &roll2, &pitch2, &yaw2);
        yaw2 = normalizeYaw(yaw2 - yawOffset2);
        
        // Left hand logic (from C code)
        if ((yaw2 >= 350.0f) || (yaw2 <= 100.0f)) {
            if (data->gyro2_y < -2500 && !printedForGyro2y) {
                if (pitch2 > 30.0f && data->gyro2_z > -2000) {
                    play_drum_sample(hihat_closed_sample_data, hihat_closed_sample_length, hihat_closed_sample_sample_rate);
                } else {
                    play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
                }
                printedForGyro2y = 1;
            } else if (data->gyro2_y >= -2500 && printedForGyro2y) {
                printedForGyro2y = 0;
            }
        }
        // Add other left hand zones as needed...
    }
}

// Read 32-byte sensor data packet from FPGA
void readSensorDataPacket(sensor_data_t *data) {
    uint8_t packet[32];
    uint8_t i;
    
    // Wait for DONE signal
    while(!digitalReadPortA(SPI_DONE));
    
    // Lower CE to start transaction
    digitalWritePortA(SPI_CE, GPIO_LOW);
    
    // Read 32 bytes
    for (i = 0; i < 32; i++) {
        packet[i] = (uint8_t)spiSendReceive(0x00);
    }
    
    // Wait for SPI transaction to complete
    while(SPI1->SR & SPI_SR_BSY);
    
    // Raise CE to end transaction
    digitalWritePortA(SPI_CE, GPIO_HIGH);
    
    // Acknowledge by toggling LOAD
    digitalWritePortA(SPI_LOAD, GPIO_HIGH);
    digitalWritePortA(SPI_LOAD, GPIO_LOW);
    
    // Parse packet into structure
    // Sensor 1 Quaternion (bytes 0-7)
    data->quat1_w = (int16_t)((packet[0] << 8) | packet[1]);
    data->quat1_x = (int16_t)((packet[2] << 8) | packet[3]);
    data->quat1_y = (int16_t)((packet[4] << 8) | packet[5]);
    data->quat1_z = (int16_t)((packet[6] << 8) | packet[7]);
    
    // Sensor 1 Gyroscope (bytes 8-13)
    data->gyro1_x = (int16_t)((packet[8] << 8) | packet[9]);
    data->gyro1_y = (int16_t)((packet[10] << 8) | packet[11]);
    data->gyro1_z = (int16_t)((packet[12] << 8) | packet[13]);
    
    // Sensor 1 Flags (byte 14)
    data->quat1_valid = packet[14] & 0x01;
    data->gyro1_valid = (packet[14] >> 1) & 0x01;
    
    // Sensor 2 Quaternion (bytes 15-22)
    data->quat2_w = (int16_t)((packet[15] << 8) | packet[16]);
    data->quat2_x = (int16_t)((packet[17] << 8) | packet[18]);
    data->quat2_y = (int16_t)((packet[19] << 8) | packet[20]);
    data->quat2_z = (int16_t)((packet[21] << 8) | packet[22]);
    
    // Sensor 2 Gyroscope (bytes 23-28)
    data->gyro2_x = (int16_t)((packet[23] << 8) | packet[24]);
    data->gyro2_y = (int16_t)((packet[25] << 8) | packet[26]);
    data->gyro2_z = (int16_t)((packet[27] << 8) | packet[28]);
    
    // Sensor 2 Flags (byte 29)
    data->quat2_valid = packet[29] & 0x01;
    data->gyro2_valid = (packet[29] >> 1) & 0x01;
    
    // Buttons (bytes 30-31)
    data->kick_btn = packet[30] & 0x01;
    data->calibrate_btn = packet[31] & 0x01;
}

// Main function
int main(void) {
    sensor_data_t sensor_data;
    
    // Initialize system
    configureFlash();
    configureClock();
    
    // Initialize SPI for FPGA communication
    initSPI(3, 0, 0);  // br=3 (5MHz), cpol=0, cpha=0
    initSPIControlPins();
    
    // Initialize DAC for audio output
    DAC_InitAudio(DAC_CHANNEL_1);
    
    // Main loop: read sensor data and process
    while(1) {
        // Read sensor data packet from FPGA
        readSensorDataPacket(&sensor_data);
        
        // Process data and play drum sounds
        process_sensor_data(&sensor_data);
    }
    
    return 0;
}

