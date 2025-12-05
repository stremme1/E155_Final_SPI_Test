# Left and Right Hand System Setup

This project supports two separate systems - one for the left hand and one for the right hand. Each system has its own MCU, FPGA, and Arduino.

## File Structure

### Main Files
- **`main_left.c`** - MCU code for LEFT HAND system
- **`main_right.c`** - MCU code for RIGHT HAND system
- **`main.c`** - Original dual-hand system (kept for reference)

### Drum Detector Files
- **`drum_detector_left.c`** - LEFT HAND drum detection logic
- **`drum_detector_right.c`** - RIGHT HAND drum detection logic
- **`drum_detector.c`** - Original dual-hand detection (kept for reference)

## Building and Flashing

### For LEFT HAND System

1. **In your SEGGER Embedded Studio project:**
   - Remove `main.c` from the project (if it's there)
   - Add `main_left.c` to the project
   - Add `drum_detector_left.c` to the project
   - Remove `drum_detector.c` from the project (optional, but recommended to avoid confusion)

2. **Build the project:**
   - Build → Build Solution (or press F7)

3. **Flash to MCU:**
   - Target → Download and Debug (or press F5)
   - The debug output will show: `=== MCU Drum Trigger System - LEFT HAND ===`

### For RIGHT HAND System

1. **In your SEGGER Embedded Studio project:**
   - Remove `main.c` from the project (if it's there)
   - Add `main_right.c` to the project
   - Add `drum_detector_right.c` to the project
   - Remove `drum_detector.c` from the project (optional, but recommended to avoid confusion)

2. **Build the project:**
   - Build → Build Solution (or press F7)

3. **Flash to MCU:**
   - Target → Download and Debug (or press F5)
   - The debug output will show: `=== MCU Drum Trigger System - RIGHT HAND ===`

## Differences Between Systems

### LEFT HAND System (`main_left.c`)
- Processes only LEFT HAND sensor data
- Uses `detect_drum_trigger_left()` function
- Uses `update_yaw_offset_left()` for calibration
- Drum zones:
  - Yaw 350-100: Snare or Hi-Hat
  - Yaw 325-350: High Tom or Crash
  - Yaw 300-325: Mid Tom or Ride
  - Yaw 200-300: Floor Tom or Ride

### RIGHT HAND System (`main_right.c`)
- Processes only RIGHT HAND sensor data
- Uses `detect_drum_trigger_right()` function
- Uses `update_yaw_offset_right()` for calibration
- Drum zones:
  - Yaw 20-120: Snare
  - Yaw 340-20: High Tom or Crash
  - Yaw 305-340: Mid Tom or Ride
  - Yaw 200-305: Floor Tom or Ride

## Hardware Setup

Each system requires:
- **MCU (STM32L432KC)** - Flash the appropriate main file
- **FPGA** - Same FPGA code for both (handles single sensor)
- **Arduino** - Same Arduino code for both (sends single sensor data)

## Quick Reference

### To switch between left and right:
1. In SEGGER Embedded Studio, right-click on the project
2. Select "Properties" or "Options"
3. Go to "C/C++ Build" → "Settings" → "Tool Settings" → "C Compiler" → "Preprocessor"
4. Or simply add/remove the appropriate main file from the project

### Alternative: Use Preprocessor Defines

You could also use a single `main.c` with preprocessor defines:

```c
#ifdef LEFT_HAND
    // Left hand code
#elif defined(RIGHT_HAND)
    // Right hand code
#else
    // Dual hand code (default)
#endif
```

Then define `LEFT_HAND` or `RIGHT_HAND` in the project settings.

## Notes

- Both systems use the same SPI communication protocol
- Both systems use the same audio player and WAV samples
- Both systems have the same button configuration (PA8=Calibrate, PA10=Kick)
- The calibration button calibrates only the active hand's yaw offset
- The kick button works the same in both systems

## Troubleshooting

If you see errors about missing functions:
- Make sure you've added the correct drum detector file (`drum_detector_left.c` or `drum_detector_right.c`)
- Make sure you've removed the old `drum_detector.c` if it conflicts

If the wrong hand logic is being used:
- Check the debug output - it should say "LEFT HAND" or "RIGHT HAND" at startup
- Verify you've built and flashed the correct main file

