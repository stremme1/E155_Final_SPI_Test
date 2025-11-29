// main.c
// Integrated BNO085 Drum Trigger System
// MCU receives raw sensor data from FPGA, decodes, detects drum triggers, and plays sounds
//
// Description:
// - MCU is SPI master, FPGA is SPI slave
// - Reads raw sensor data (quaternion + gyro) from FPGA via SPI
// - Decodes quaternion to Euler angles using Code_for_C_imp logic
// - Detects drum triggers using Code_for_C_imp zone detection
// - Plays WAV drum samples using Lab4-Final_Project DAC code

#include "STM32L432KC_RCC.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_FLASH.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_DAC.h"
#include "bno085_decoder.h"
#include "drum_detector.h"
#include "audio_player.h"
#include <stdint.h>

// Button pins for calibration (optional)
#define BUTTON_CALIBRATE_PIN PA8  // D7 on Nucleo

// Sensor data packet structure
typedef struct {
    // Sensor 1 (Right Hand)
    int16_t quat1_w, quat1_x, quat1_y, quat1_z;
    int16_t gyro1_x, gyro1_y, gyro1_z;
    uint8_t quat1_valid, gyro1_valid;
    
    // Sensor 2 (Left Hand)
    int16_t quat2_w, quat2_x, quat2_y, quat2_z;
    int16_t gyro2_x, gyro2_y, gyro2_z;
    uint8_t quat2_valid, gyro2_valid;
} sensor_data_t;

// Global state
static drum_detector_state_t drum_state = {0, 0};
static yaw_offset_t yaw_offsets = {0.0f, 0.0f};

// Read sensor data packet from FPGA
void read_sensor_data(sensor_data_t *data) {
    uint8_t packet[32];
    
    // Read 32-byte packet from FPGA
    readSensorDataPacket(packet);
    
    // Parse packet into structure
    parseSensorDataPacket(packet,
                          // Sensor 1
                          &data->quat1_w, &data->quat1_x, &data->quat1_y, &data->quat1_z,
                          &data->gyro1_x, &data->gyro1_y, &data->gyro1_z,
                          &data->quat1_valid, &data->gyro1_valid,
                          // Sensor 2
                          &data->quat2_w, &data->quat2_x, &data->quat2_y, &data->quat2_z,
                          &data->gyro2_x, &data->gyro2_y, &data->gyro2_z,
                          &data->quat2_valid, &data->gyro2_valid);
}

// Process sensor data and detect drum triggers
void process_sensor_data(sensor_data_t *data) {
    euler_t euler1, euler2;
    uint8_t drum_code;
    
    // Only process if we have valid quaternion data
    if (!data->quat1_valid && !data->quat2_valid) {
        return;
    }
    
    // Convert quaternion to Euler angles (Sensor 1)
    if (data->quat1_valid) {
        quaternion_to_euler(data->quat1_w, data->quat1_x, data->quat1_y, data->quat1_z, &euler1);
    } else {
        // Set to zero if not valid
        euler1.roll = euler1.pitch = euler1.yaw = 0.0f;
    }
    
    // Convert quaternion to Euler angles (Sensor 2)
    if (data->quat2_valid) {
        quaternion_to_euler(data->quat2_w, data->quat2_x, data->quat2_y, data->quat2_z, &euler2);
    } else {
        // Set to zero if not valid
        euler2.roll = euler2.pitch = euler2.yaw = 0.0f;
    }
    
    // Detect drum trigger
    drum_code = detect_drum_trigger(
        &euler1, data->gyro1_y, data->gyro1_z,
        &euler2, data->gyro2_y, data->gyro2_z,
        &drum_state,
        &yaw_offsets
    );
    
    // Play sound if trigger detected
    if (drum_code != 0xFF) {
        handle_drum_command(drum_code);
    }
}

// Check calibration button (optional)
void check_calibration_button(void) {
    static uint8_t last_state = 0;
    static uint32_t debounce_counter = 0;
    uint8_t current_state = digitalRead(BUTTON_CALIBRATE_PIN);
    
    if (current_state != last_state) {
        debounce_counter++;
        if (debounce_counter > 1000) {  // Debounce threshold
            last_state = current_state;
            debounce_counter = 0;
            
            if (current_state == 1) {
                // Button pressed - calibrate yaw offsets
                sensor_data_t data;
                euler_t euler1, euler2;
                
                // Read current sensor data
                read_sensor_data(&data);
                
                // Convert to Euler
                if (data.quat1_valid) {
                    quaternion_to_euler(data.quat1_w, data.quat1_x, data.quat1_y, data.quat1_z, &euler1);
                }
                if (data.quat2_valid) {
                    quaternion_to_euler(data.quat2_w, data.quat2_x, data.quat2_y, data.quat2_z, &euler2);
                }
                
                // Update offsets
                update_yaw_offsets(&euler1, &euler2, &yaw_offsets);
            }
        }
    } else {
        debounce_counter = 0;
    }
}

// Main function
int main(void) {
    sensor_data_t sensor_data;
    
    // Initialize system
    configureFlash();
    configureClock();
    
    // Enable GPIO clocks
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
    
    // Initialize SPI as MASTER for FPGA communication
    // br=1 (divide by 4 = 80MHz/4 = 20MHz), cpol=0, cpha=0 (SPI Mode 0) - matches Lab07
    initSPI(1, 0, 0);
    
    // Load and done pins (Lab07 style)
    pinMode(PA5, GPIO_OUTPUT);  // LOAD
    pinMode(PA6, GPIO_INPUT);   // DONE
    
    // CE pin (Lab07 style)
    pinMode(PA11, GPIO_OUTPUT);
    digitalWrite(PA11, 1);  // CE high initially
    
    // Initialize calibration button (optional)
    pinMode(BUTTON_CALIBRATE_PIN, GPIO_INPUT);
    
    // Initialize DAC for audio output
    DAC_InitAudio(DAC_CHANNEL_1);
    
    // Main loop: read sensor data, decode, detect triggers, play sounds
    while(1) {
        // Read raw sensor data from FPGA via SPI (blocking - waits for DONE signal)
        read_sensor_data(&sensor_data);
        
        // Process sensor data and detect drum triggers
        process_sensor_data(&sensor_data);
        
        // Check calibration button (optional)
        check_calibration_button();
        
        // No delay needed - read_sensor_data() blocks until next packet is ready
    }
    
    return 0;
}
