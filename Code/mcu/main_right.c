// main_right.c
// RIGHT HAND Drum Trigger System
// MCU receives raw sensor data from FPGA, decodes, detects drum triggers, and plays sounds
// This version processes ONLY the right hand sensor
//
// Description:
// - MCU is SPI master, FPGA is SPI slave
// - Reads raw sensor data (quaternion + gyro) from FPGA via SPI
// - Decodes quaternion to Euler angles using Code_for_C_imp logic
// - Detects drum triggers using RIGHT HAND zone detection
// - Plays WAV drum samples using Lab4-Final_Project DAC code

#include "STM32L432KC_RCC.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_FLASH.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_DAC.h"
#include "STM32L432KC_TIMER.h"  // For ms_delay
#include "STM32L432KC_USART.h"
#include "debug_print.h"
#include "bno085_decoder.h"
#include "drum_detector.h"
#include "audio_player.h"
#include "wav_arrays/drum_samples.h"  // For test sound loop
#include <stdint.h>

// Button pins
// Buttons are pulled up (normally HIGH), go LOW when pressed
#define BUTTON_CALIBRATE_PIN PA8   // Calibration button
#define BUTTON_KICK_PIN      PA10  // Kick drum trigger button

// Sensor data packet structure (simplified for single sensor)
typedef struct {
    int16_t quat_w, quat_x, quat_y, quat_z;
    int16_t gyro_x, gyro_y, gyro_z;
    uint8_t quat_valid, gyro_valid;
} sensor_data_t;

// Global state
static drum_detector_state_t drum_state = {0, 0};
static float yaw_offset = 0.0f;

// Function to play a drum sample - EXACTLY like Lab4
void play_drum_sample(const int16_t* data, uint32_t length, uint32_t sample_rate) {
    DAC_PlayWAV(data, length, sample_rate);
}

// Read sensor data packet from FPGA (16-byte packet for single sensor)
void read_sensor_data(sensor_data_t *data) {
    uint8_t packet[16];  // 16-byte packet
    
    // Read 16-byte packet from FPGA
    readSensorDataPacket(packet);
    
    // Parse packet into structure (treating as sensor 1 / right hand)
    int16_t quat1_w, quat1_x, quat1_y, quat1_z;
    int16_t gyro1_x, gyro1_y, gyro1_z;
    uint8_t quat1_valid, gyro1_valid;
    int16_t quat2_w, quat2_x, quat2_y, quat2_z;  // Dummy for sensor 2
    int16_t gyro2_x, gyro2_y, gyro2_z;  // Dummy for sensor 2
    uint8_t quat2_valid, gyro2_valid;  // Dummy for sensor 2
    
    parseSensorDataPacket(packet,
                          // Sensor 1 (right hand - this is what we use)
                          &quat1_w, &quat1_x, &quat1_y, &quat1_z,
                          &gyro1_x, &gyro1_y, &gyro1_z,
                          &quat1_valid, &gyro1_valid,
                          // Sensor 2 (unused dummy variables)
                          &quat2_w, &quat2_x, &quat2_y, &quat2_z,
                          &gyro2_x, &gyro2_y, &gyro2_z,
                          &quat2_valid, &gyro2_valid);
    
    // Copy to our simplified structure
    data->quat_w = quat1_w;
    data->quat_x = quat1_x;
    data->quat_y = quat1_y;
    data->quat_z = quat1_z;
    data->gyro_x = gyro1_x;
    data->gyro_y = gyro1_y;
    data->gyro_z = gyro1_z;
    data->quat_valid = quat1_valid;
    data->gyro_valid = gyro1_valid;
}

// Process sensor data and detect drum triggers (RIGHT HAND ONLY)
void process_sensor_data(sensor_data_t *data) {
    euler_t euler;
    uint8_t drum_code;
    
    // Only process if we have valid quaternion data
    if (!data->quat_valid) {
        debug_print("[SENSOR] No valid quaternion data, skipping processing\r\n");
        return;
    }
    
    // Convert quaternion to Euler angles
    quaternion_to_euler(data->quat_w, data->quat_x, data->quat_y, data->quat_z, &euler);
    debug_printf("[SENSOR] Right Hand Euler: roll=%.2f pitch=%.2f yaw=%.2f\r\n", euler.roll, euler.pitch, euler.yaw);
    
    // Detect drum trigger using RIGHT HAND logic
    drum_code = detect_drum_trigger_right(
        &euler,
        data->gyro_y,
        data->gyro_z,
        &drum_state,
        &yaw_offset
    );
    
    // Play sound if trigger detected
    if (drum_code != 0xFF) {
        debug_printf("[TRIGGER] Drum trigger detected: code=0x%x\r\n", drum_code);
        handle_drum_command(drum_code);
    }
}

// Check calibration button
// Button is pulled up (normally HIGH), goes LOW when pressed
void check_calibration_button(void) {
    static uint8_t last_state = 1;  // Start with HIGH (not pressed)
    static uint32_t debounce_counter = 0;
    uint8_t current_state = digitalRead(BUTTON_CALIBRATE_PIN);
    
    if (current_state != last_state) {
        debounce_counter++;
        if (debounce_counter > 1000) {  // Debounce threshold
            last_state = current_state;
            debounce_counter = 0;
            
            if (current_state == 0) {  // Button pressed (LOW when pressed)
                debug_print("[BUTTON] Calibration button pressed\r\n");
                // Calibrate yaw offset
                sensor_data_t data;
                euler_t euler;
                
                // Read current sensor data
                read_sensor_data(&data);
                
                // Convert to Euler
                if (data.quat_valid) {
                    quaternion_to_euler(data.quat_w, data.quat_x, data.quat_y, data.quat_z, &euler);
                    // Update offset
                    update_yaw_offset_right(&euler, &yaw_offset);
                    debug_printf("[BUTTON] Right hand yaw offset updated: offset=%.2f\r\n", yaw_offset);
                }
            }
        }
    } else {
        debounce_counter = 0;
    }
}

// Check kick button
// Button is pulled up (normally HIGH), goes LOW when pressed
// Triggers kick drum sound when pressed
// Simple edge detection: trigger on falling edge (HIGH -> LOW)
void check_kick_button(void) {
    static uint8_t last_state = 1;  // Start with HIGH (not pressed)
    static uint32_t debounce_counter = 0;
    uint8_t current_state = digitalRead(BUTTON_KICK_PIN);
    
    // Detect falling edge: was HIGH, now LOW (button pressed)
    if (last_state == 1 && current_state == 0) {
        debounce_counter++;
        // Very low debounce threshold for immediate response
        // Only need 3 consecutive LOW readings to confirm press
        if (debounce_counter >= 3) {
            // Button press confirmed - trigger kick drum
            debug_print("[BUTTON] Kick button pressed - triggering kick drum\r\n");
            // Note: handle_drum_command will check is_playing flag internally
            handle_drum_command(DRUM_CODE_KICK);
            last_state = current_state;  // Update state to prevent retrigger until release
            debounce_counter = 0;
        }
    } else if (last_state == 0 && current_state == 1) {
        // Rising edge: button released - allow new press
        last_state = current_state;
        debounce_counter = 0;
    } else if (last_state == 1 && current_state == 1) {
        // Button not pressed - reset counter
        debounce_counter = 0;
    }
    // If last_state == 0 && current_state == 0, keep counting (button still pressed)
}

// Main function
int main(void) {
    sensor_data_t sensor_data;
    
    // Initialize system - EXACTLY like Lab4
    // NOTE: Must configure clock BEFORE USART initialization
    configureFlash();
    configureClock();
    
    // Initialize debug USART AFTER clock is configured (so baud rate is correct)
    debug_init();
    
    // Small delay to ensure USART is fully initialized
    volatile int init_delay = 100000;
    while(init_delay-- > 0) __asm("nop");
    
    // Test USART with simple characters first
    debug_print("XXX\r\n");
    
    // Another delay to ensure transmission completes
    init_delay = 50000;
    while(init_delay-- > 0) __asm("nop");
    
    debug_print("\r\n\r\n=== MCU Drum Trigger System - RIGHT HAND ===\r\n");
    debug_print("[INIT] Flash and clock configured (80MHz)\r\n");
    
    // Initialize DAC for audio output (using channel 1 on PA4) - EXACTLY like Lab4
    debug_print("[INIT] Initializing DAC (Channel 1, PA4)...\r\n");
    DAC_InitAudio(DAC_CHANNEL_1);
    debug_print("[INIT] DAC initialized successfully\r\n");
    
    // Initialize multi-channel audio system (non-blocking, timer interrupt based)
    debug_print("[INIT] Initializing audio player (multi-channel, non-blocking)...\r\n");
    audio_player_init();
    debug_print("[INIT] Audio player initialized successfully\r\n");
    
    // CRITICAL TEST: Direct register write test - EXACTLY like Lab4
    // This bypasses all functions to test if the DAC hardware works at all
    // Measure PA4 with multimeter - you should see voltage changes
    debug_print("[TEST] DAC hardware test - setting different voltage levels\r\n");
    
    // Test 1: Maximum value (4095) - should be ~3.1V
    DAC->DHR12R1 = 4095;
    ms_delay(2000);  // Hold for 2 seconds
    
    // Test 2: Mid-point (2048) - should be ~1.65V  
    DAC->DHR12R1 = 2048;
    ms_delay(2000);  // Hold for 2 seconds
    
    // Test 3: Quarter (1024) - should be ~0.825V
    DAC->DHR12R1 = 1024;
    ms_delay(2000);  // Hold for 2 seconds
    
    // Test 4: Minimum (0) - should be ~0.2V (buffer minimum)
    DAC->DHR12R1 = 0;
    ms_delay(2000);  // Hold for 2 seconds
    
    // Test 5: Back to maximum
    DAC->DHR12R1 = 4095;
    ms_delay(2000);  // Hold for 2 seconds
    
    // NOW initialize other peripherals (SPI and buttons) AFTER DAC is set up
    // Enable GPIO clocks (needed for SPI and buttons)
    debug_print("[INIT] Enabling GPIO clocks...\r\n");
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
    
    // Initialize SPI as SLAVE for Arduino/ESP32 communication
    // Arduino/ESP32 acts as master and sends data at 625kHz
    // MCU follows master's clock (no baud rate setting needed in slave mode)
    // Mode 0: CPOL=0, CPHA=0 (matches Arduino configuration)
    debug_print("[INIT] Initializing SPI as SLAVE (Mode 0, follows master clock)...\r\n");
    initSPI(2, 0, 0);  // br parameter ignored in slave mode, CPOL=0, CPHA=0
    
    // CS pin (chip select, active low) - PA11
    // Note: CS pin is configured as INPUT in initSPI() since master controls it
    // Master (Arduino) will pull CS low to start transaction, high to end
    debug_print("[INIT] SPI CS pin (PA11) configured as INPUT (master controlled)\r\n");
    
    // Initialize buttons (pulled up, go LOW when pressed)
    debug_print("[INIT] Initializing buttons (PA8=Calibrate, PA10=Kick)...\r\n");
    pinMode(BUTTON_CALIBRATE_PIN, GPIO_INPUT);
    pinMode(BUTTON_KICK_PIN, GPIO_INPUT);
    
    // Configure pull-up resistors for buttons
    // PUPDR register: 00 = no pull, 01 = pull-up, 10 = pull-down
    // PA8: bits 16-17, PA10: bits 20-21
    int pin8_offset = BUTTON_CALIBRATE_PIN & 0x0F;  // Get pin offset (8)
    int pin10_offset = BUTTON_KICK_PIN & 0x0F;       // Get pin offset (10)
    
    // Clear existing pull configuration and set to pull-up (01)
    GPIOA->PUPDR &= ~(0b11 << (2 * pin8_offset));   // Clear bits for PA8
    GPIOA->PUPDR |= (0b01 << (2 * pin8_offset));     // Set pull-up for PA8
    
    GPIOA->PUPDR &= ~(0b11 << (2 * pin10_offset));   // Clear bits for PA10
    GPIOA->PUPDR |= (0b01 << (2 * pin10_offset));    // Set pull-up for PA10
    debug_print("[INIT] Buttons configured with pull-up resistors\r\n");
    
    // TEST MODE: Play drum samples in sequence for testing - EXACTLY like Lab4
    // Loop 5 times to verify DAC is configured and working
    debug_print("[TEST] Starting DAC test mode - playing all drum samples 5 times\r\n");
    for(int test_loop = 0; test_loop < 1; test_loop++) {
        debug_printf("[TEST] Test loop %d/5\r\n", test_loop + 1);
        // Kick
        debug_print("[TEST] Playing KICK\r\n");
        play_drum_sample(kick_sample_data, kick_sample_length, kick_sample_sample_rate);
        ms_delay(200);
        
        // Snare
        debug_print("[TEST] Playing SNARE\r\n");
        play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
        ms_delay(200);
        
        // Hi-Hat Closed
        debug_print("[TEST] Playing HIHAT_CLOSED\r\n");
        play_drum_sample(hihat_closed_sample_data, hihat_closed_sample_length, hihat_closed_sample_sample_rate);
        ms_delay(200);
        
        // Hi-Hat Open
        debug_print("[TEST] Playing HIHAT_OPEN\r\n");
        play_drum_sample(hihat_open_sample_data, hihat_open_sample_length, hihat_open_sample_sample_rate);
        ms_delay(200);
        
        // Crash
        debug_print("[TEST] Playing CRASH\r\n");
        play_drum_sample(crash_sample_data, crash_sample_length, crash_sample_sample_rate);
        ms_delay(200);
        
        // Ride
        debug_print("[TEST] Playing RIDE\r\n");
        play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
        ms_delay(200);
        
        // Tom High
        debug_print("[TEST] Playing TOM_HIGH\r\n");
        play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
        ms_delay(200);
        
        // Tom Low
        debug_print("[TEST] Playing TOM_LOW\r\n");
        play_drum_sample(tom_low_sample_data, tom_low_sample_length, tom_low_sample_sample_rate);
        ms_delay(500);  // Longer pause before repeating
    }
    debug_print("[TEST] DAC test mode complete\r\n");
    debug_print("[MAIN] Entering main loop - reading RIGHT HAND sensor data from FPGA\r\n");
    
    // Main loop: read sensor data, decode, detect triggers, play sounds
    while(1) {
        // Check buttons FIRST (before blocking read) to catch button presses quickly
        check_calibration_button();
        check_kick_button();
        
        // Read raw sensor data from FPGA via SPI (slave mode)
        // Note: This waits for master to pull CS low and send data
        read_sensor_data(&sensor_data);
        
        // Process sensor data and detect drum triggers (RIGHT HAND ONLY)
        process_sensor_data(&sensor_data);
        
        // Check buttons again after processing (in case button was pressed during SPI read)
        check_calibration_button();
        check_kick_button();
        
        // Small delay to allow button state to stabilize and prevent too-fast polling
        // This also gives the audio system time to process
        volatile int delay = 100;
        while (delay-- > 0) {
            __asm("nop");
        }
    }
    
    return 0;
}

