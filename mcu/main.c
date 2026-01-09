// main.c
// drum trigger system

#include "STM32L432KC_RCC.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_FLASH.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_DAC.h"
#include "STM32L432KC_TIMER.h"
#include "STM32L432KC_USART.h"
#include "debug_print.h"
#include "bno085_decoder.h"
#include "drum_detector.h"
#include "audio_player.h"
#include "wav_arrays/drum_samples.h"
#include <stdint.h>

// button pins
#define BUTTON_CALIBRATE_PIN PA8
#define BUTTON_KICK_PIN      PA10

// sensor data structure
typedef struct {
    // sensor 1
    int16_t quat1_w, quat1_x, quat1_y, quat1_z;
    int16_t gyro1_x, gyro1_y, gyro1_z;
    uint8_t quat1_valid, gyro1_valid;
    
    // sensor 2
    int16_t quat2_w, quat2_x, quat2_y, quat2_z;
    int16_t gyro2_x, gyro2_y, gyro2_z;
    uint8_t quat2_valid, gyro2_valid;
} sensor_data_t;

// global state
static drum_detector_state_t drum_state = {0, 0};
static yaw_offset_t yaw_offsets = {0.0f, 0.0f};

// play drum sample
void play_drum_sample(const int16_t* data, uint32_t length, uint32_t sample_rate) {
    DAC_PlayWAV(data, length, sample_rate);
}

// read sensor data
void read_sensor_data(sensor_data_t *data) {
    uint8_t packet[16];
    
    readSensorDataPacket(packet);
    
    parseSensorDataPacket(packet,
                          &data->quat1_w, &data->quat1_x, &data->quat1_y, &data->quat1_z,
                          &data->gyro1_x, &data->gyro1_y, &data->gyro1_z,
                          &data->quat1_valid, &data->gyro1_valid,
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
        debug_print("[SENSOR] No valid quaternion data, skipping processing\r\n");
        return;
    }
    
    // convert quaternion to euler
    if (data->quat1_valid) {
        quaternion_to_euler(data->quat1_w, data->quat1_x, data->quat1_y, data->quat1_z, &euler1);
        debug_printf("[SENSOR] Euler1: roll=%.2f pitch=%.2f yaw=%.2f\r\n", euler1.roll, euler1.pitch, euler1.yaw);
    } else {
        euler1.roll = euler1.pitch = euler1.yaw = 0.0f;
    }
    
    // convert quaternion to euler
    if (data->quat2_valid) {
        quaternion_to_euler(data->quat2_w, data->quat2_x, data->quat2_y, data->quat2_z, &euler2);
        debug_printf("[SENSOR] Euler2: roll=%.2f pitch=%.2f yaw=%.2f\r\n", euler2.roll, euler2.pitch, euler2.yaw);
    } else {
        euler2.roll = euler2.pitch = euler2.yaw = 0.0f;
    }
    
    // detect drum trigger
    drum_code = detect_drum_trigger(
        &euler1, data->gyro1_y, data->gyro1_z,
        &euler2, data->gyro2_y, data->gyro2_z,
        &drum_state,
        &yaw_offsets
    );
    
    // play sound
    if (drum_code != 0xFF) {
        debug_printf("[TRIGGER] Drum trigger detected: code=0x%x\r\n", drum_code);
        handle_drum_command(drum_code);
    }
}

// check calibration button
void check_calibration_button(void) {
    static uint8_t last_state = 1;  // Start with HIGH (not pressed)
    static uint32_t debounce_counter = 0;
    uint8_t current_state = digitalRead(BUTTON_CALIBRATE_PIN);
    
    if (current_state != last_state) {
        debounce_counter++;
        if (debounce_counter > 1000) {
            last_state = current_state;
            debounce_counter = 0;
            
            if (current_state == 0) {
                debug_print("[BUTTON] Calibration button pressed\r\n");
                sensor_data_t data;
                euler_t euler1, euler2;
                
                read_sensor_data(&data);
                
                if (data.quat1_valid) {
                    quaternion_to_euler(data.quat1_w, data.quat1_x, data.quat1_y, data.quat1_z, &euler1);
                }
                if (data.quat2_valid) {
                    quaternion_to_euler(data.quat2_w, data.quat2_x, data.quat2_y, data.quat2_z, &euler2);
                }
                
                // Update offsets
                update_yaw_offsets(&euler1, &euler2, &yaw_offsets);
                debug_printf("[BUTTON] Yaw offsets updated: offset1=%.2f offset2=%.2f\r\n", yaw_offsets.yaw_offset1, yaw_offsets.yaw_offset2);
            }
        }
    } else {
        debounce_counter = 0;
    }
}

// check kick button
void check_kick_button(void) {
    static uint8_t last_state = 1;  // Start with HIGH (not pressed)
    static uint32_t debounce_counter = 0;
    uint8_t current_state = digitalRead(BUTTON_KICK_PIN);
    
    if (last_state == 1 && current_state == 0) {
        debounce_counter++;
        if (debounce_counter >= 3) {
            debug_print("[BUTTON] Kick button pressed - triggering kick drum\r\n");
            handle_drum_command(DRUM_CODE_KICK);
            last_state = current_state;
            debounce_counter = 0;
        }
    } else if (last_state == 0 && current_state == 1) {
        last_state = current_state;
        debounce_counter = 0;
    } else if (last_state == 1 && current_state == 1) {
        debounce_counter = 0;
    }
}

// main
int main(void) {
    sensor_data_t sensor_data;
    
    // init system
    configureFlash();
    configureClock();
    
    // init USART
    debug_init();
    
    // delay for USART
    volatile int init_delay = 100000;
    while(init_delay-- > 0) __asm("nop");
    
    // test USART
    debug_print("XXX\r\n");
    
    // delay
    init_delay = 50000;
    while(init_delay-- > 0) __asm("nop");
    
    debug_print("\r\n\r\n=== MCU Drum Trigger System Starting ===\r\n");
    debug_print("[INIT] Flash and clock configured (80MHz)\r\n");
    
    // Initialize DAC for audio output (using channel 1 on PA4) - EXACTLY like Lab4
    debug_print("[INIT] Initializing DAC (Channel 1, PA4)...\r\n");
    DAC_InitAudio(DAC_CHANNEL_1);
    debug_print("[INIT] DAC initialized successfully\r\n");
    
    // DAC hardware test
    debug_print("[TEST] DAC hardware test - setting different voltage levels\r\n");
    
    DAC->DHR12R1 = 4095;
    ms_delay(2000);
    DAC->DHR12R1 = 2048;
    ms_delay(2000);
    DAC->DHR12R1 = 1024;
    ms_delay(2000);
    DAC->DHR12R1 = 0;
    ms_delay(2000);
    DAC->DHR12R1 = 4095;
    ms_delay(2000);
    
    // NOW initialize other peripherals (SPI and buttons) AFTER DAC is set up
    // Enable GPIO clocks (needed for SPI and buttons)
    debug_print("[INIT] Enabling GPIO clocks...\r\n");
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
    
    // init SPI as slave
    debug_print("[INIT] Initializing SPI as SLAVE (Mode 0, follows master clock)...\r\n");
    initSPI(2, 0, 0);
    
    // CS pin PA11
    debug_print("[INIT] SPI CS pin (PA11) configured as INPUT (master controlled)\r\n");
    
    // init buttons
    debug_print("[INIT] Initializing buttons (PA8=Calibrate, PA10=Kick)...\r\n");
    pinMode(BUTTON_CALIBRATE_PIN, GPIO_INPUT);
    pinMode(BUTTON_KICK_PIN, GPIO_INPUT);
    
    // configure pull-up resistors
    int pin8_offset = BUTTON_CALIBRATE_PIN & 0x0F;
    int pin10_offset = BUTTON_KICK_PIN & 0x0F;
    
    GPIOA->PUPDR &= ~(0b11 << (2 * pin8_offset));
    GPIOA->PUPDR |= (0b01 << (2 * pin8_offset));
    
    GPIOA->PUPDR &= ~(0b11 << (2 * pin10_offset));
    GPIOA->PUPDR |= (0b01 << (2 * pin10_offset));
    debug_print("[INIT] Buttons configured with pull-up resistors\r\n");
    
    // test mode: play all drum samples
    debug_print("[TEST] Starting DAC test mode - playing all drum samples 5 times\r\n");
    for(int test_loop = 0; test_loop < 1; test_loop++) {
        debug_printf("[TEST] Test loop %d/5\r\n", test_loop + 1);
        play_drum_sample(kick_sample_data, kick_sample_length, kick_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(hihat_closed_sample_data, hihat_closed_sample_length, hihat_closed_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(hihat_open_sample_data, hihat_open_sample_length, hihat_open_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(crash_sample_data, crash_sample_length, crash_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
        ms_delay(200);
        play_drum_sample(tom_low_sample_data, tom_low_sample_length, tom_low_sample_sample_rate);
        ms_delay(500);
    }
    debug_print("[TEST] DAC test mode complete\r\n");
    debug_print("[MAIN] Entering main loop - reading sensor data from Arduino/ESP32\r\n");
    
    // main loop
    while(1) {
        check_calibration_button();
        check_kick_button();
        
        read_sensor_data(&sensor_data);
        process_sensor_data(&sensor_data);
        
        check_calibration_button();
        check_kick_button();
        
        // delay
        volatile int delay = 100;
        while (delay-- > 0) {
            __asm("nop");
        }
    }
    
    return 0;
}
