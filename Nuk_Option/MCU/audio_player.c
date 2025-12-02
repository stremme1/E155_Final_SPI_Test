#include "audio_player.h"
#include "STM32L432KC_DAC.h"
#include "wav_arrays/drum_samples.h"
#include "debug_print.h"
#include <stddef.h>

// Drum sample rate (all samples converted to 22.05 kHz)
#define DRUM_SAMPLE_RATE 22050

// Flag to prevent overlapping playback
static volatile uint8_t is_playing = 0;

// Function to play a drum sample
static void play_drum_sample(const int16_t* data, uint32_t length, uint32_t sample_rate) {
    if (data == NULL || length == 0) {
        debug_print("[AUDIO] ERROR: Invalid sample data\r\n");
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
        debug_print("[AUDIO] Ignoring command - already playing\r\n");
        return;
    }
    
    // Ignore invalid commands
    if (command > 0x07) {
        debug_printf("[AUDIO] Invalid command: 0x%x (ignoring)\r\n", command);
        return;
    }
    
    debug_printf("[AUDIO] Playing drum command: 0x%x\r\n", command);
    
    switch(command) {
        case DRUM_CODE_SNARE:  // 0x00: Snare drum
            debug_print("[AUDIO] -> SNARE\r\n");
            play_drum_sample(snare_sample_data, snare_sample_length, snare_sample_sample_rate);
            break;
            
        case DRUM_CODE_HIHAT:  // 0x01: Hi-hat (use closed for now)
            debug_print("[AUDIO] -> HIHAT_CLOSED\r\n");
            play_drum_sample(hihat_closed_sample_data, hihat_closed_sample_length, hihat_closed_sample_sample_rate);
            break;
            
        case DRUM_CODE_KICK:  // 0x02: Kick drum
            debug_print("[AUDIO] -> KICK\r\n");
            play_drum_sample(kick_sample_data, kick_sample_length, kick_sample_sample_rate);
            break;
            
        case DRUM_CODE_HIGH_TOM:  // 0x03: High tom
            debug_print("[AUDIO] -> TOM_HIGH\r\n");
            play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
            break;
            
        case DRUM_CODE_MID_TOM:  // 0x04: Mid tom (use tom_high sample for now)
            // Note: Lab4 doesn't have a separate mid_tom sample, using tom_high
            debug_print("[AUDIO] -> TOM_MID (using TOM_HIGH sample)\r\n");
            play_drum_sample(tom_high_sample_data, tom_high_sample_length, tom_high_sample_sample_rate);
            break;
            
        case DRUM_CODE_CRASH:  // 0x05: Crash cymbal
            debug_print("[AUDIO] -> CRASH\r\n");
            play_drum_sample(crash_sample_data, crash_sample_length, crash_sample_sample_rate);
            break;
            
        case DRUM_CODE_RIDE:  // 0x06: Ride cymbal
            debug_print("[AUDIO] -> RIDE\r\n");
            play_drum_sample(ride_sample_data, ride_sample_length, ride_sample_sample_rate);
            break;
            
        case DRUM_CODE_FLOOR_TOM:  // 0x07: Floor tom
            debug_print("[AUDIO] -> TOM_LOW\r\n");
            play_drum_sample(tom_low_sample_data, tom_low_sample_length, tom_low_sample_sample_rate);
            break;
            
        default:
            // Invalid command, ignore
            debug_printf("[AUDIO] Unknown command: 0x%x\r\n", command);
            break;
    }
    
    debug_print("[AUDIO] Playback complete\r\n");
}

