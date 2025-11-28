// main_integrated.c
// Integrated Drum Sample Player with SPI Communication
// Combines Lab4 drum playback with Lab07 SPI communication
//
// Author: Integration based on Lab4 and Lab07
// Date: Integration implementation
//
// Description: 
// - Receives drum trigger commands from FPGA via SPI
// - Plays corresponding WAV drum samples using DAC
// - Maps FPGA drum codes (0-7) to Lab4 drum samples
// - Handles Kick Button (PB1/D6) and Calibrate Button (PA8/D7)
// - NO USART DEBUGGING (Removed to prevent undefined symbol errors)

#include <stddef.h>  // For NULL
#include "STM32L432KC_RCC.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_FLASH.h"
#include "STM32L432KC_DAC.h"
#include "STM32L432KC_TIMER.h"
#include "STM32L432KC_SPI.h"

// Include drum sample arrays header
#include "wav_arrays/drum_samples.h"

// Drum sample rate (all samples converted to 22.05 kHz)
#define DRUM_SAMPLE_RATE 22050

// Drum command codes from FPGA (matches FPGA drum_code output)
#define DRUM_CODE_SNARE      0x00
#define DRUM_CODE_HIHAT      0x01
#define DRUM_CODE_KICK       0x02
#define DRUM_CODE_HIGH_TOM   0x03
#define DRUM_CODE_MID_TOM    0x04
#define DRUM_CODE_CRASH      0x05
#define DRUM_CODE_RIDE       0x06
#define DRUM_CODE_FLOOR_TOM  0x07

// Button Pins
#define BUTTON_KICK_PIN      PB1  // D6 on Nucleo
#define BUTTON_CALIBRATE_PIN PA8  // D7 on Nucleo

// Debounce configuration
#define DEBOUNCE_DELAY_MS    50

// Flag to prevent overlapping playback
static volatile uint8_t is_playing = 0;

// Function to play a drum sample
static void play_drum_sample(const int16_t* data, uint32_t length, uint32_t sample_rate) {
    if (data == NULL || length == 0) {
        return;
    }
    
    is_playing = 1;
    DAC_PlayWAV(data, length, sample_rate);
    is_playing = 0;
}

// Handle drum command from FPGA or Buttons
// Maps FPGA drum codes (0-7) to Lab4 drum samples
void handle_drum_command(uint8_t command) {
    // Ignore commands while playing to prevent overlapping sounds
    if (is_playing) {
        return;
    }
    
    // Ignore invalid commands
    if (command > 0x07) {
        return;
    }
    
    switch(command) {
        case DRUM_CODE_SNARE:  // 0x00: Snare drum
            play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
            break;
            
        case DRUM_CODE_HIHAT:  // 0x01: Hi-hat (use closed for now)
            play_drum_sample(hihat_closed_sample_data, hihat_closed_sample_length, hihat_closed_sample_sample_rate);
            break;
            
        case DRUM_CODE_KICK:  // 0x02: Kick drum
            play_drum_sample(kick_sample_data, kick_sample_length, kick_sample_sample_rate);
            break;
            
        case DRUM_CODE_HIGH_TOM:  // 0x03: High tom
            play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
            break;
            
        case DRUM_CODE_MID_TOM:  // 0x04: Mid tom (use tom_high sample for now)
            // Note: Lab4 doesn't have a separate mid_tom sample, using tom_high
            play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
            break;
            
        case DRUM_CODE_CRASH:  // 0x05: Crash cymbal
            play_drum_sample(crash_sample_data, crash_sample_length, crash_sample_sample_rate);
            break;
            
        case DRUM_CODE_RIDE:  // 0x06: Ride cymbal
            play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
            break;
            
        case DRUM_CODE_FLOOR_TOM:  // 0x07: Floor tom
            play_drum_sample(tom_low_sample_data, tom_low_sample_length, tom_low_sample_sample_rate);
            break;
            
        default:
            // Invalid command, ignore
            break;
    }
}

// Helper for simple non-blocking delay check (using system tick or loop counter)
uint32_t debounce_counter_kick = 0;
uint32_t debounce_counter_calib = 0;
uint8_t kick_last_state = 0;
uint8_t calib_last_state = 0;

void check_buttons(void) {
    int kick_state = digitalRead(BUTTON_KICK_PIN);
    int calib_state = digitalRead(BUTTON_CALIBRATE_PIN);
    
    // Simple state change detection with crude debounce
    if (kick_state != kick_last_state) {
        debounce_counter_kick++;
        if (debounce_counter_kick > 1000) { // Threshold for debounce
            kick_last_state = kick_state;
            debounce_counter_kick = 0;
            
            if (kick_state == 1) {
                handle_drum_command(DRUM_CODE_KICK);
            }
        }
    } else {
        debounce_counter_kick = 0;
    }
    
    if (calib_state != calib_last_state) {
        debounce_counter_calib++;
        if (debounce_counter_calib > 1000) {
            calib_last_state = calib_state;
            debounce_counter_calib = 0;
            
            if (calib_state == 1) {
                handle_drum_command(DRUM_CODE_HIHAT); 
            }
        }
    } else {
        debounce_counter_calib = 0;
    }
}

// Main function
int main(void) {
    // Initialize system
    configureFlash();
    configureClock();
    
    // Enable GPIO clocks (Lab07 style - line 60)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
    
    // Initialize SPI (Lab07 style - line 64)
    initSPI(1, 0, 0);
    
    // Load and done pins (Lab07 style - lines 68-69)
    pinMode(PA5, GPIO_OUTPUT);  // LOAD
    pinMode(PA6, GPIO_INPUT);   // DONE
    
    // CE pin (Lab07 style - lines 78-79)
    pinMode(PA11, GPIO_OUTPUT);
    digitalWrite(PA11, 1);  // CE high initially
    
    // Initialize Buttons
    pinMode(BUTTON_KICK_PIN, GPIO_INPUT);      // PB1
    pinMode(BUTTON_CALIBRATE_PIN, GPIO_INPUT); // PA8
    
    // Initialize DAC for audio output
    DAC_InitAudio(DAC_CHANNEL_1);
    
    // Startup Sound - Play Snare twice to indicate ready
    ms_delay(500);
    handle_drum_command(DRUM_CODE_SNARE);
    ms_delay(200);
    handle_drum_command(DRUM_CODE_SNARE);
    ms_delay(500);
    
    // TEST: Send SPI data to verify clock appears on PB3
    // In SPI Mode 0: Clock is idle LOW, goes HIGH during transmission
    // Each byte generates 8 clock pulses (4 HIGH, 4 LOW transitions)
    for(int i = 0; i < 20; i++) {
        digitalWrite(PA11, 1); // CE high (like Lab07 line 126)
        spiSendReceive(0xAA);  // Send byte - clock will toggle HIGH/LOW 8 times
        digitalWrite(PA11, 0); // CE low (like Lab07 line 128)
        ms_delay(10);  // Small delay between bytes
    }
    
    while(SPI1->SR & SPI_SR_BSY); // Wait for SPI to finish
    ms_delay(100);
    
    // Main loop
    while(1) {
        // Check for SPI data from FPGA (Lab07 style - blocking wait)
        if (digitalRead(PA6)) {  // DONE signal (Lab07 style)
            uint8_t packet[15];
            int16_t quat_w, quat_x, quat_y, quat_z;
            int16_t gyro_x, gyro_y, gyro_z;
            
            readSensorDataPacket15(packet);
            parseSensorDataPacket15(packet, &quat_w, &quat_x, &quat_y, &quat_z,
                                    &gyro_x, &gyro_y, &gyro_z);
            
            // TODO: Process sensor data for drum detection
        }
        
        check_buttons();
    }
    
    return 0;
}
