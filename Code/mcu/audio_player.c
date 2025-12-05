// audio_player.c
// Multi-channel non-blocking audio playback system
//
// Architecture:
// - Uses TIM6 interrupt at 22.05kHz for precise sample timing
// - Multi-channel mixing (up to 8 simultaneous sounds)
// - Non-blocking: main loop never stops, can always read sensors
// - Professional implementation with proper mixing and clipping prevention
//
// Based on Approach 1 from BRAINSTORM_OVERLAPPING_AUDIO.md

#include "audio_player.h"
#include "STM32L432KC_DAC.h"
#include "STM32L432KC_TIMER.h"
#include "wav_arrays/drum_samples.h"
#include "debug_print.h"
#include <stddef.h>

// Audio configuration
#define DRUM_SAMPLE_RATE 22050  // All samples converted to 22.05 kHz
#define MAX_CHANNELS 8           // Maximum simultaneous sounds (like pygame.mixer)

// Audio channel structure (similar to pygame.mixer.Channel)
typedef struct {
    const int16_t* data;      // Pointer to sample data
    uint32_t length;          // Number of samples
    uint32_t position;        // Current playback position
    uint8_t active;           // 1 if channel is playing, 0 if free
} audio_channel_t;

// Channel array (one channel per simultaneous sound)
static audio_channel_t channels[MAX_CHANNELS];

// Audio system state
static volatile uint8_t audio_initialized = 0;

// ============================================================================
// Internal Functions
// ============================================================================

// Find a free channel for new playback
// Returns channel index (0 to MAX_CHANNELS-1) or -1 if all channels busy
static int find_free_channel(void) {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (!channels[i].active) {
            return i;
        }
    }
    return -1;  // All channels busy
}

// Start playback on a specific channel (non-blocking)
static void start_channel_playback(int channel_idx, const int16_t* data, uint32_t length) {
    if (channel_idx < 0 || channel_idx >= MAX_CHANNELS) {
        return;
    }
    
    if (data == NULL || length == 0) {
        return;
    }
    
    // Initialize channel
    channels[channel_idx].data = data;
    channels[channel_idx].length = length;
    channels[channel_idx].position = 0;
    channels[channel_idx].active = 1;
    
    debug_printf("[AUDIO] Started playback on channel %d (%d samples)\r\n", channel_idx, length);
}

// Get sample data pointer and length for a drum command
static void get_drum_sample_data(uint8_t command, const int16_t** data, uint32_t* length) {
    switch(command) {
        case DRUM_CODE_SNARE:
            *data = snare_sample_data;
            *length = snare_sample_length;
            break;
        case DRUM_CODE_HIHAT:
            *data = hihat_closed_sample_data;
            *length = hihat_closed_sample_length;
            break;
        case DRUM_CODE_KICK:
            *data = kick_sample_data;
            *length = kick_sample_length;
            break;
        case DRUM_CODE_HIGH_TOM:
            *data = tom_high_sample_data;
            *length = tom_high_sample_length;
            break;
        case DRUM_CODE_MID_TOM:
            *data = tom_high_sample_data;  // Use tom_high as fallback
            *length = tom_high_sample_length;
            break;
        case DRUM_CODE_CRASH:
            *data = crash_sample_data;
            *length = crash_sample_length;
            break;
        case DRUM_CODE_RIDE:
            *data = ride_sample_data;
            *length = ride_sample_length;
            break;
        case DRUM_CODE_FLOOR_TOM:
            *data = tom_low_sample_data;
            *length = tom_low_sample_length;
            break;
        default:
            *data = NULL;
            *length = 0;
            break;
    }
}

// Convert mixed sample to DAC value (12-bit, centered at 2048)
static uint16_t mix_to_dac(int32_t mixed_sample, uint8_t active_count) {
    if (active_count == 0) {
        return 2048;  // Silence (mid-point)
    }
    
    // Average the samples to prevent clipping (simple mixing)
    int32_t averaged = mixed_sample / active_count;
    
    // Scale: map -32768..32767 to 0..4095
    // Center at 2048: dac_value = 2048 + (sample >> 4)
    int32_t dac_value = 2048 + (averaged >> 4);
    
    // Clamp to 12-bit range
    if (dac_value < 0) dac_value = 0;
    if (dac_value > 4095) dac_value = 4095;
    
    return (uint16_t)dac_value;
}

// ============================================================================
// Public Functions
// ============================================================================

// Initialize audio system (call once at startup)
void audio_player_init(void) {
    if (audio_initialized) {
        return;  // Already initialized
    }
    
    // Initialize all channels to inactive
    for (int i = 0; i < MAX_CHANNELS; i++) {
        channels[i].active = 0;
        channels[i].data = NULL;
        channels[i].length = 0;
        channels[i].position = 0;
    }
    
    // Initialize TIM6 for audio interrupts
    TIM6_InitAudioInterrupt();
    
    // Start the audio interrupt timer
    TIM6_StartAudioInterrupt();
    
    audio_initialized = 1;
    
    debug_print("[AUDIO] Audio system initialized (multi-channel, non-blocking)\r\n");
}

// Handle drum command - starts playback on free channel (non-blocking)
// Based on Code_for_C_imp/PYTHON/play_sound.py behavior
// Each trigger starts immediately, can overlap with other sounds
void handle_drum_command(uint8_t command) {
    // Ensure audio system is initialized
    if (!audio_initialized) {
        audio_player_init();
    }
    
    // Ignore invalid commands
    if (command > 0x07) {
        debug_printf("[AUDIO] Invalid command: 0x%x (ignoring)\r\n", command);
        return;
    }
    
    // Find free channel
    int channel = find_free_channel();
    if (channel < 0) {
        // All channels busy - could queue or ignore
        // For now, ignore (like pygame when all channels busy)
        debug_print("[AUDIO] All channels busy, ignoring trigger\r\n");
        return;
    }
    
    // Get sample data
    const int16_t* sample_data;
    uint32_t sample_length;
    get_drum_sample_data(command, &sample_data, &sample_length);
    
    if (sample_data == NULL || sample_length == 0) {
        debug_printf("[AUDIO] Invalid sample data for command 0x%x\r\n", command);
        return;
    }
    
    // Start playback on free channel (non-blocking)
    start_channel_playback(channel, sample_data, sample_length);
    
    // Debug output
    const char* drum_names[] = {
        "SNARE", "HIHAT", "KICK", "TOM_HIGH",
        "TOM_MID", "CRASH", "RIDE", "TOM_LOW"
    };
    if (command < 8) {
        debug_printf("[AUDIO] Trigger: %s (channel %d)\r\n", drum_names[command], channel);
    }
}

// ============================================================================
// TIM6 Interrupt Handler (called at 22.05kHz)
// ============================================================================
// This function is called automatically by the hardware when TIM6 overflows
// It mixes all active channels and outputs one sample to the DAC

void TIM6_DAC_IRQHandler(void) {
    // Clear interrupt flag (UIF bit in SR register)
    TIM6->SR &= ~(1 << 0);
    
    // Mix all active channels
    int32_t mixed_sample = 0;
    uint8_t active_count = 0;
    
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (channels[i].active) {
            if (channels[i].position < channels[i].length) {
                // Get sample from this channel
                int32_t sample = (int32_t)channels[i].data[channels[i].position];
                mixed_sample += sample;  // Simple mixing (add samples)
                active_count++;
                channels[i].position++;  // Advance to next sample
            } else {
                // Channel finished - mark as inactive
                channels[i].active = 0;
                channels[i].data = NULL;
                channels[i].length = 0;
                channels[i].position = 0;
            }
        }
    }
    
    // Convert mixed sample to DAC value and output
    uint16_t dac_value = mix_to_dac(mixed_sample, active_count);
    DAC_SetValue(DAC_CHANNEL_1, dac_value);
}
