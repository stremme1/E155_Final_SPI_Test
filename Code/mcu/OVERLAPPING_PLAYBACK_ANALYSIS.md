# Overlapping Playback Analysis

## Current Behavior

**Detection**: ✅ YES - The drum detector continues to run and can detect multiple triggers while audio is playing.

**Playback**: ❌ NO - New triggers are **ignored** while audio is playing.

## Current Implementation

### Detection Flow
1. Main loop continuously reads sensor data
2. Drum detector processes sensor data and detects triggers
3. If trigger detected, calls `handle_drum_command()`

### Playback Flow
1. `handle_drum_command()` checks `is_playing` flag
2. If `is_playing == 1`, new command is **ignored** (returns early)
3. If `is_playing == 0`, starts playback:
   - Sets `is_playing = 1`
   - Calls `DAC_PlayWAV()` (blocking function)
   - Waits for entire sample to play
   - Sets `is_playing = 0`

### Code Location
- **Detection**: `drum_detector_left.c` / `drum_detector_right.c`
- **Playback Control**: `audio_player.c` lines 28-32
- **Audio Output**: `STM32L432KC_DAC.c` `DAC_PlayWAV()` function (blocking)

## Impact

### What Happens When Multiple Triggers Occur
1. **First trigger**: Detected → Plays sound → Blocks for ~100-500ms (sample length)
2. **Second trigger** (during playback): Detected → **Ignored** → Lost
3. **Third trigger** (after playback): Detected → Plays sound

### Example Timeline
```
Time:  0ms    50ms   100ms  150ms  200ms  250ms  300ms
       |      |      |      |      |      |      |
       Trigger1      Trigger2      Trigger3
       (plays)       (ignored)      (plays)
       |-------------|
       Playing...
```

## Options for Overlapping Playback

### Option 1: Simple Queue (Recommended)
- Add a small queue (2-4 commands)
- Queue new commands while playing
- Play next command when current finishes
- **Pros**: Simple, prevents lost triggers
- **Cons**: Still sequential, slight delay

### Option 2: Interrupt-Based Playback
- Use DMA + Timer interrupts
- Non-blocking playback
- Can queue multiple sounds
- **Pros**: True overlapping, no blocking
- **Cons**: More complex, requires DMA setup

### Option 3: Mixing (Advanced)
- Mix multiple samples in real-time
- Play combined audio
- **Pros**: True simultaneous playback
- **Cons**: Complex, requires more CPU/memory

## Recommendation

For drum triggers, **Option 1 (Simple Queue)** is recommended:
- Prevents lost triggers
- Simple to implement
- Sequential playback is acceptable for drums
- Minimal code changes

Would you like me to implement a queue system?

