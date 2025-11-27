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

// Handle drum command from FPGA
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

// Main function
// NOTE: Only ONE main() function should be active at a time!
// This is the ACTIVE main() - receives drum commands from FPGA via SPI
// To use main_sensor_data.c instead:
//   1. Comment out this main() function
//   2. Uncomment main() in main_sensor_data.c
//   3. Rebuild the project
int main(void) {
    // Initialize system
    configureFlash();
    configureClock();
    
    // Initialize SPI for FPGA communication
    // br=3 (divide by 16 = 80MHz/16 = 5MHz), cpol=0, cpha=0 (SPI Mode 0)
    // Using slower baud rate for FPGA compatibility (FPGA runs at 3MHz)
    initSPI(3, 0, 0);
    initSPIControlPins();
    
    // Initialize DAC for audio output (using channel 1 on PA4)
    DAC_InitAudio(DAC_CHANNEL_1);
    
    // Main loop: wait for drum commands from FPGA
    // Following Lab07 pattern: blocking wait in readDrumCommand()
    while(1) {
        // Read drum command from FPGA via SPI (blocking - waits for DONE signal)
        uint8_t command = readDrumCommand();
        
        // Process command if valid (0x00-0x07 are all valid drum codes)
        if (command <= 0x07) {
            handle_drum_command(command);
        }
        
        // No delay needed - readDrumCommand() blocks until next command is ready
        // This matches Lab07's blocking approach
    }
    
    return 0;
}

