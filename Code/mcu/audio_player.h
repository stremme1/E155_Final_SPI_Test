#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include <stdint.h>

// Drum command codes (matches FPGA and drum_detector)
#define DRUM_CODE_SNARE      0x00
#define DRUM_CODE_HIHAT      0x01
#define DRUM_CODE_KICK       0x02
#define DRUM_CODE_HIGH_TOM   0x03
#define DRUM_CODE_MID_TOM    0x04
#define DRUM_CODE_CRASH      0x05
#define DRUM_CODE_RIDE       0x06
#define DRUM_CODE_FLOOR_TOM  0x07

// Initialize audio system (call once at startup)
// Sets up TIM6 interrupt for 22.05kHz audio output
void audio_player_init(void);

// Handle drum command - starts playback on free channel (non-blocking)
// Based on Code_for_C_imp/PYTHON/play_sound.py behavior
// Each trigger starts immediately, can overlap with other sounds
// Like pygame.mixer: sound.play() starts immediately, doesn't block
void handle_drum_command(uint8_t command);

// TIM6 interrupt handler (called automatically by hardware at 22.05kHz)
// Mixes all active channels and outputs to DAC
// This is defined in audio_player.c but declared here for reference
void TIM6_DAC_IRQHandler(void);

#endif // AUDIO_PLAYER_H

