// main.c
// MCU-Only BNO085 Drum Trigger System
// MCU directly communicates with BNO085 sensor, decodes, detects drum triggers, and plays sounds
//
// Description:
// - MCU directly connected to BNO085 sensor via SPI
// - MCU handles all SHTP protocol communication
// - Decodes quaternion to Euler angles using Code_for_C_imp logic
// - Detects drum triggers using Code_for_C_imp zone detection
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
#include "bno085_driver.h"
#include "wav_arrays/drum_samples.h"  // For test sound loop
#include <stdint.h>
#include <stddef.h>  // For NULL

// Button pins
// Buttons are pulled up (normally HIGH), go LOW when pressed
#define BUTTON_CALIBRATE_PIN PA8   // Calibration button
#define BUTTON_KICK_PIN      PA10  // Kick drum trigger button

// BNO085 Pin Configuration
// User must configure these pins based on hardware connections
// SPI pins are configured in initSPI() - these are control pins
// Note: BNO085_CS_PIN, BNO085_PS0_WAKE_PIN, and BNO085_INT_PIN are extern variables
// defined in bno085_driver.c - they will be assigned values in main() before calling bno085_init()
// Note: SPI pins (SCK, MOSI, MISO) are configured in initSPI() as PB3, PB5, PB4

// Global state
static drum_detector_state_t drum_state = {0, 0};
static yaw_offset_t yaw_offsets = {0.0f, 0.0f};

// Function to play a drum sample - EXACTLY like Lab4
void play_drum_sample(const int16_t* data, uint32_t length, uint32_t sample_rate) {
    DAC_PlayWAV(data, length, sample_rate);
}

// Process sensor data and detect drum triggers
// NOTE: This implementation uses a SINGLE sensor (Sensor 1 only)
// The detection logic supports dual sensors, but Sensor 2 will always be zero
// Only Sensor 1 (right hand) triggers will work in this configuration
void process_sensor_data(bno085_data_t *data) {
    euler_t euler1, euler2;
    uint8_t drum_code;
    
    // Null pointer check
    if (data == NULL) {
        debug_print("[SENSOR] ERROR: process_sensor_data called with NULL pointer\r\n");
        return;
    }
    
    // Only process if we have valid quaternion data
    if (!data->quat_valid) {
        return;  // Skip if no valid data
    }
    
    // Validate quaternion values are reasonable (Q14 format: -16384 to +16384)
    // Check for all zeros or extreme values that might indicate bad data
    if (data->quat_w == 0 && data->quat_x == 0 && 
        data->quat_y == 0 && data->quat_z == 0) {
        debug_print("[SENSOR] WARN: Quaternion is all zeros (invalid data)\r\n");
        return;
    }
    
    // Convert quaternion to Euler angles (Sensor 1 - right hand)
    quaternion_to_euler(data->quat_w, data->quat_x, data->quat_y, data->quat_z, &euler1);
    debug_printf("[SENSOR] Euler1: roll=%.2f pitch=%.2f yaw=%.2f\r\n", 
                euler1.roll, euler1.pitch, euler1.yaw);
    
    // Validate Euler angles are reasonable (not NaN, not infinite)
    // Angles should be in range: roll/pitch: -90 to +90, yaw: 0 to 360
    if (euler1.roll < -180.0f || euler1.roll > 180.0f ||
        euler1.pitch < -180.0f || euler1.pitch > 180.0f ||
        euler1.yaw < -360.0f || euler1.yaw > 360.0f) {
        debug_printf("[SENSOR] WARN: Euler angles out of range: roll=%.2f pitch=%.2f yaw=%.2f\r\n",
                    euler1.roll, euler1.pitch, euler1.yaw);
        return;
    }
    
    // Sensor 2 (left hand) - not used in single sensor setup
    // Set to zero - detection logic will check Sensor 2 but it will never trigger
    euler2.roll = euler2.pitch = euler2.yaw = 0.0f;
    
    // Detect drum trigger (uses same logic as original dual-sensor code)
    // NOTE: Only Sensor 1 triggers will work since Sensor 2 is always zero
    drum_code = detect_drum_trigger(
        &euler1, data->gyro_y, data->gyro_z,  // Sensor 1: right hand
        &euler2, 0, 0,  // Sensor 2: left hand (not used, always zero)
        &drum_state,
        &yaw_offsets
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
                // Calibrate yaw offsets
                bno085_data_t data;
                euler_t euler1, euler2;
                
                // Read current sensor data
                if (bno085_read_data(&data) && data.quat_valid) {
                    quaternion_to_euler(data.quat_w, data.quat_x, data.quat_y, data.quat_z, &euler1);
                    // Sensor 2 not used
                    euler2.roll = euler2.pitch = euler2.yaw = 0.0f;
                    
                    // Update offsets
                    update_yaw_offsets(&euler1, &euler2, &yaw_offsets);
                    debug_printf("[BUTTON] Yaw offsets updated: offset1=%.2f offset2=%.2f\r\n", yaw_offsets.yaw_offset1, yaw_offsets.yaw_offset2);
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
void check_kick_button(void) {
    static uint8_t last_state = 1;  // Start with HIGH (not pressed)
    static uint32_t debounce_counter = 0;
    uint8_t current_state = digitalRead(BUTTON_KICK_PIN);
    
    // Detect falling edge: was HIGH, now LOW (button pressed)
    if (last_state == 1 && current_state == 0) {
        debounce_counter++;
        // Very low debounce threshold for immediate response
        if (debounce_counter >= 3) {
            // Button press confirmed - trigger kick drum
            debug_print("[BUTTON] Kick button pressed - triggering kick drum\r\n");
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
}

// Main function
int main(void) {
    bno085_data_t sensor_data;
    
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
    
    debug_print("\r\n\r\n=== MCU-Only BNO085 Drum Trigger System Starting ===\r\n");
    debug_print("[INIT] Flash and clock configured (80MHz)\r\n");
    
    // Initialize DAC for audio output (using channel 1 on PA4) - EXACTLY like Lab4
    debug_print("[INIT] Initializing DAC (Channel 1, PA4)...\r\n");
    DAC_InitAudio(DAC_CHANNEL_1);
    debug_print("[INIT] DAC initialized successfully\r\n");
    
    // Enable GPIO clocks (needed for SPI, buttons, and BNO085 control pins)
    debug_print("[INIT] Enabling GPIO clocks...\r\n");
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
    
    // Initialize SPI as MASTER for BNO085 communication
    // BNO085 requires SPI Mode 3 (CPOL=1, CPHA=1) per datasheet and FPGA code
    // BNO085 supports up to 3MHz SPI clock
    // br=5 (divide by 64 = 80MHz/64 = 1.25MHz) is safe for BNO085
    // br=4 (divide by 32 = 80MHz/32 = 2.5MHz) is also safe
    // Using br=5 for conservative timing
    debug_print("[INIT] Initializing SPI (Mode 3, 1.25MHz for BNO085)...\r\n");
    debug_print("[INIT] SPI Mode 3: CPOL=1 (idle high), CPHA=1 (sample on rising edge)\r\n");
    initSPI(5, 1, 1);  // br=5, cpol=1, cpha=1 (Mode 3) - matches FPGA implementation
    
    // Configure BNO085 pin assignments
    debug_print("[INIT] Configuring BNO085 pins...\r\n");
    BNO085_CS_PIN = PA11;        // CS pin
    BNO085_PS0_WAKE_PIN = PA12;  // PS0/WAKE pin
    BNO085_INT_PIN = PA15;       // INT pin
    
    // Initialize BNO085 sensor
    debug_print("[INIT] Initializing BNO085 sensor...\r\n");
    if (!bno085_init()) {
        debug_print("[INIT] ERROR: BNO085 initialization failed!\r\n");
        // Continue anyway - bno085_service() will handle errors
    }
    
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
    debug_print("[TEST] Starting DAC test mode - playing all drum samples once\r\n");
    for(int test_loop = 0; test_loop < 1; test_loop++) {
        debug_printf("[TEST] Test loop %d/1\r\n", test_loop + 1);
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
    debug_print("[MAIN] Entering main loop - reading sensor data from BNO085\r\n");
    
    // Main loop: service BNO085, read sensor data, decode, detect triggers, play sounds
    while(1) {
        // Check for sensor errors and handle recovery
        if (bno085_has_error() && !bno085_is_initialized()) {
            debug_print("[MAIN] Sensor error detected - driver will attempt recovery\r\n");
        }
        
        // Service BNO085 state machine (handles initialization and data reading)
        bno085_service();
        
        // Check buttons FIRST (before blocking operations) to catch button presses quickly
        check_calibration_button();
        check_kick_button();
        
        // Only process sensor data if sensor is initialized
        if (bno085_is_initialized() && !bno085_has_error()) {
            // Read sensor data from BNO085 (non-blocking)
            if (bno085_read_data(&sensor_data)) {
                // Process sensor data and detect drum triggers
                process_sensor_data(&sensor_data);
            }
        } else if (!bno085_is_initialized()) {
            // Sensor not initialized yet - just service the state machine
            static uint32_t init_wait_counter = 0;
            if (init_wait_counter++ % 100000 == 0) {  // Log every ~1.25 seconds
                debug_print("[MAIN] Waiting for sensor initialization...\r\n");
            }
        }
        
        // Check buttons again after processing
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

