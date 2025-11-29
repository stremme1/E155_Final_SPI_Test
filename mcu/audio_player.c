#include "audio_player.h"
#include "STM32L432KC_DAC.h"
#include "wav_arrays/drum_samples.h"
#include <stddef.h>

// Drum sample rate (all samples converted to 22.05 kHz)
#define DRUM_SAMPLE_RATE 22050

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

// Handle drum command - plays corresponding WAV sample
// Based on Lab4-Final_Project/main_integrated.c handle_drum_command()
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

