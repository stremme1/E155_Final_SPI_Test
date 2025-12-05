# Brainstorm: Overlapping Audio Playback Solutions

## Problem
- Need to collect triggers while audio is playing
- Need to play multiple sounds simultaneously (overlapping)
- Current blocking `DAC_PlayWAV()` prevents main loop from running

## Solution Approaches

---

## Approach 1: Timer Interrupt + State Machine (RECOMMENDED)

### Concept
Use a hardware timer interrupt to output audio samples at exactly 22.05kHz, while main loop continues normally.

### How It Works
1. **Timer Setup**: Configure TIM6 or TIM7 to interrupt at 22.05kHz (every ~45μs)
2. **Interrupt Handler**: Outputs one sample per interrupt, advances playback position
3. **State Machine**: Tracks multiple playing sounds (channels)
4. **Mixing**: Mixes all active channels in interrupt handler
5. **Main Loop**: Continues reading sensors, new triggers just add to channel list

### Implementation
```c
// In timer interrupt (22.05kHz)
void TIM6_IRQHandler(void) {
    // Mix all active channels
    int32_t mixed = 0;
    uint8_t active_count = 0;
    
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (channels[i].active) {
            if (channels[i].pos < channels[i].length) {
                mixed += channels[i].data[channels[i].pos++];
                active_count++;
            } else {
                channels[i].active = 0;  // Channel finished
            }
        }
    }
    
    // Output mixed sample
    if (active_count > 0) {
        DAC_SetValue(DAC_CHANNEL_1, mix_to_dac(mixed / active_count));
    } else {
        DAC_SetValue(DAC_CHANNEL_1, 2048);  // Silence
    }
}

// In main loop - just add triggers (non-blocking)
void handle_drum_command(uint8_t command) {
    int channel = find_free_channel();
    if (channel >= 0) {
        start_channel(channel, get_sample_data(command));
    }
}
```

### Pros
✅ True non-blocking - main loop never stops  
✅ Precise timing (hardware timer)  
✅ Can mix multiple sounds  
✅ Professional audio quality  
✅ Relatively simple to implement  

### Cons
❌ Requires timer interrupt setup  
❌ Mixing math in interrupt (must be fast)  
❌ Need to manage multiple channels  

### Estimated Time: 2-3 hours

---

## Approach 2: DMA + Timer (Most Professional)

### Concept
Use DMA to automatically transfer samples to DAC, triggered by timer. CPU is completely free.

### How It Works
1. **DMA Setup**: Configure DMA channel to transfer from memory to DAC
2. **Timer Setup**: Timer triggers DMA transfers at sample rate
3. **Double Buffering**: While DMA plays buffer 1, CPU fills buffer 2
4. **Channel Management**: Track multiple sounds, mix into DMA buffers

### Implementation
```c
// DMA automatically transfers samples to DAC
// Timer triggers DMA at 22.05kHz
// CPU just mixes samples into buffers when ready
```

### Pros
✅ CPU completely free (DMA handles everything)  
✅ Most efficient  
✅ Professional audio systems use this  
✅ Can handle many channels  

### Cons
❌ Most complex to implement  
❌ Requires DMA + Timer configuration  
❌ Buffer management complexity  
❌ More memory usage  

### Estimated Time: 4-6 hours

---

## Approach 3: Simple Queue + Fast Polling

### Concept
Keep blocking playback but make it very fast, process queue between samples.

### How It Works
1. **Queue System**: Store triggers in queue
2. **Fast Playback**: Process one sample, check queue, process next sample
3. **Interrupt Current**: New trigger can interrupt by setting flag

### Implementation
```c
void play_with_queue(const int16_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        // Check for interrupt
        if (interrupt_flag) {
            return;  // New trigger wants to play
        }
        
        // Check queue
        if (queue_count > 0) {
            // Mix current + queued sounds
        }
        
        // Output sample
        DAC_SetValue(...);
        delay_sample_period();
    }
}
```

### Pros
✅ Simpler than interrupts  
✅ No timer setup needed  
✅ Can mix sounds  

### Cons
❌ Still somewhat blocking  
❌ Timing less precise  
❌ Mixing in main loop (slower)  

### Estimated Time: 1-2 hours

---

## Approach 4: Hybrid - Timer + Simple Mixing

### Concept
Timer interrupt outputs samples, but only one sound plays at a time. New triggers interrupt current.

### How It Works
1. **Timer Interrupt**: Outputs samples at 22.05kHz
2. **Single Channel**: Only one sound plays at a time
3. **Interrupt Flag**: New trigger sets flag, current sound stops
4. **Main Loop**: Continues normally, just sets interrupt flag

### Implementation
```c
// Timer interrupt
void TIM6_IRQHandler(void) {
    if (interrupt_flag) {
        // Stop current, start new
        current_sample = new_sample;
        current_pos = 0;
        interrupt_flag = 0;
    }
    
    if (current_pos < current_length) {
        DAC_SetValue(DAC_CHANNEL_1, current_sample[current_pos++]);
    }
}

// Main loop
void handle_drum_command(uint8_t command) {
    if (is_playing) {
        interrupt_flag = 1;  // Interrupt current
    }
    new_sample = get_sample_data(command);
    is_playing = 1;
}
```

### Pros
✅ Non-blocking main loop  
✅ Simple (no mixing)  
✅ Most recent trigger wins  
✅ Fast to implement  

### Cons
❌ No true overlapping (one sound at a time)  
❌ Still need timer setup  

### Estimated Time: 1-2 hours

---

## Approach 5: Background Task in Main Loop

### Concept
Process one audio sample per main loop iteration, mix all active channels.

### How It Works
1. **State Machine**: Track all playing sounds
2. **Per-Iteration**: Process one sample from each active channel
3. **Mixing**: Mix all samples, output to DAC
4. **Timing**: Use counter to maintain sample rate

### Implementation
```c
void audio_update(void) {
    static uint32_t sample_counter = 0;
    sample_counter++;
    
    // Only output every N iterations (to maintain 22.05kHz)
    if (sample_counter < samples_per_iteration) {
        return;
    }
    sample_counter = 0;
    
    // Mix all channels
    int32_t mixed = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (channels[i].active) {
            mixed += channels[i].data[channels[i].pos++];
            if (channels[i].pos >= channels[i].length) {
                channels[i].active = 0;
            }
        }
    }
    
    DAC_SetValue(DAC_CHANNEL_1, mix_to_dac(mixed));
}
```

### Pros
✅ No interrupts needed  
✅ Simple to understand  
✅ Can mix multiple sounds  
✅ Main loop continues  

### Cons
❌ Timing less precise (depends on loop speed)  
❌ May cause audio glitches if loop is slow  
❌ Mixing happens in main loop (slower)  

### Estimated Time: 1 hour

---

## Recommendation

### For Your Use Case: **Approach 1 (Timer Interrupt + State Machine)**

**Why:**
1. **Non-blocking**: Main loop never stops, can always read sensors
2. **True overlapping**: Can play multiple sounds simultaneously
3. **Precise timing**: Hardware timer ensures correct sample rate
4. **Manageable complexity**: Not too hard to implement
5. **Professional**: Similar to how real audio systems work

### Implementation Steps

1. **Setup Timer 6** (or Timer 7) for 22.05kHz interrupts
2. **Create channel structure** to track playing sounds
3. **Implement interrupt handler** that mixes and outputs samples
4. **Modify `handle_drum_command()`** to just add channels (non-blocking)
5. **Remove blocking `DAC_PlayWAV()` calls**

### Quick Start Code Structure

```c
// audio_player.c
#define MAX_CHANNELS 8
typedef struct {
    const int16_t* data;
    uint32_t length;
    uint32_t position;
    uint8_t active;
} audio_channel_t;

static audio_channel_t channels[MAX_CHANNELS];

// Timer interrupt handler
void TIM6_IRQHandler(void) {
    // Clear interrupt flag
    TIM6->SR &= ~TIM_SR_UIF;
    
    // Mix and output (see Approach 1 code above)
    // ...
}

// Non-blocking trigger handler
void handle_drum_command(uint8_t command) {
    int ch = find_free_channel();
    if (ch >= 0) {
        start_channel(ch, get_sample_data(command));
    }
}
```

---

## Alternative: Quick Win (Approach 4)

If you want something working **fast** (30 minutes):
- Use timer interrupt for single-channel playback
- New triggers interrupt current (most recent wins)
- No mixing, but main loop never blocks
- Can upgrade to Approach 1 later for true overlapping

---

## Questions to Consider

1. **Do you need true overlapping** (multiple sounds at once) or just **interrupting** (new sound replaces current)?
2. **How many simultaneous sounds** do you need? (2-4 is usually enough for drums)
3. **Are you comfortable with interrupts?** (Timer interrupt is needed for best results)
4. **Timeline?** (Approach 4 = fast, Approach 1 = better but takes longer)

Let me know which approach you prefer and I can implement it!

