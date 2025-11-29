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

// Handle drum command - plays corresponding WAV sample
// Based on Lab4-Final_Project/main_integrated.c handle_drum_command()
void handle_drum_command(uint8_t command);

#endif // AUDIO_PLAYER_H

