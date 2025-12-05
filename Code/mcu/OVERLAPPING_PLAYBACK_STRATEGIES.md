# Strategies for Handling Multiple Triggers

## Strategy Comparison

| Strategy | Complexity | Overlap | Lost Triggers | Implementation Time |
|----------|-----------|---------|---------------|---------------------|
| **1. Simple Queue** | Low | No | None | 30 min |
| **2. Interrupt Queue** | Medium | No | None | 1-2 hours |
| **3. DMA + Timer** | High | Yes | None | 4-6 hours |
| **4. Audio Mixing** | Very High | Yes | None | 8+ hours |

---

## Strategy 1: Simple Queue (RECOMMENDED)

### Overview
Add a small FIFO queue to store drum commands. When a trigger is detected while playing, add it to the queue. When current playback finishes, automatically play the next queued command.

### Implementation
- Add a circular buffer (queue) for 4-8 commands
- Modify `handle_drum_command()` to enqueue instead of ignoring
- Add queue processing after playback completes
- Keep blocking playback (simple)

### Pros
✅ Simple to implement (~50 lines of code)  
✅ Prevents lost triggers  
✅ No hardware changes needed  
✅ Sequential playback (acceptable for drums)  
✅ Low memory usage  

### Cons
❌ Still sequential (no true overlap)  
❌ Slight delay for queued sounds  

### Code Structure
```c
#define QUEUE_SIZE 8
static uint8_t command_queue[QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;

void handle_drum_command(uint8_t command) {
    if (is_playing) {
        // Add to queue instead of ignoring
        if (queue_count < QUEUE_SIZE) {
            command_queue[queue_tail] = command;
            queue_tail = (queue_tail + 1) % QUEUE_SIZE;
            queue_count++;
        }
        return;
    }
    
    // Play immediately
    play_command(command);
    
    // Process queue after playback
    process_queue();
}
```

### Estimated Time: 30 minutes

---

## Strategy 2: Interrupt-Based Queue

### Overview
Similar to Strategy 1, but use a timer interrupt to check the queue periodically. This allows the main loop to continue processing sensor data without blocking.

### Implementation
- Same queue structure as Strategy 1
- Use SysTick or TIM interrupt
- Interrupt handler checks queue and starts next playback
- Non-blocking main loop

### Pros
✅ Non-blocking main loop  
✅ Better sensor data processing  
✅ Prevents lost triggers  
✅ Still relatively simple  

### Cons
❌ More complex than Strategy 1  
❌ Requires interrupt setup  
❌ Still sequential playback  

### Code Structure
```c
void SysTick_Handler(void) {
    if (!is_playing && queue_count > 0) {
        uint8_t next_cmd = dequeue();
        play_command(next_cmd);
    }
}
```

### Estimated Time: 1-2 hours

---

## Strategy 3: DMA + Timer (True Overlapping)

### Overview
Use DMA to transfer audio samples to DAC automatically, triggered by a timer. This allows the CPU to be free while audio plays, enabling true overlapping or rapid sequential playback.

### Implementation
- Configure DMA channel for DAC
- Configure timer to trigger DMA at sample rate
- Use double-buffering for smooth playback
- Queue system for multiple sounds
- Mix or switch between samples

### Pros
✅ True overlapping possible  
✅ Non-blocking CPU  
✅ Professional audio quality  
✅ Can mix multiple sounds  

### Cons
❌ Complex implementation  
❌ Requires DMA knowledge  
❌ More memory usage  
❌ Hardware-specific setup  

### Hardware Requirements
- STM32L432KC has DMA1 channels available
- DAC can use DMA (DAC_DHR12R1 register)
- Timer (TIM6/TIM7) for sample rate timing

### Estimated Time: 4-6 hours

---

## Strategy 4: Audio Mixing (Advanced)

### Overview
Mix multiple audio samples in real-time and output the combined signal. Allows true simultaneous playback of multiple drum sounds.

### Implementation
- Sample-level mixing (add samples together)
- Clipping prevention
- Volume control per sound
- Complex queue/mixer system

### Pros
✅ True simultaneous playback  
✅ Professional drum kit behavior  
✅ No lost triggers  

### Cons
❌ Very complex  
❌ High CPU usage  
❌ Memory intensive  
❌ Requires careful mixing math  

### Estimated Time: 8+ hours

---

## Recommendation

### For Your Use Case: **Strategy 1 (Simple Queue)**

**Why:**
1. **Quick to implement** - You can have it working in 30 minutes
2. **Prevents lost triggers** - Most important goal
3. **Sequential playback is fine** - Drums typically don't need true overlap
4. **Low risk** - Simple code, easy to debug
5. **Can upgrade later** - Easy to move to Strategy 2 or 3 if needed

### Implementation Plan for Strategy 1

1. **Add queue data structures** to `audio_player.c`
2. **Modify `handle_drum_command()`** to enqueue instead of ignore
3. **Add `process_queue()`** function to play next queued command
4. **Call `process_queue()`** after each playback completes
5. **Test** with rapid triggers

### Next Steps

Would you like me to:
1. **Implement Strategy 1** (Simple Queue) - Recommended
2. **Show detailed code** for Strategy 2 (Interrupt-based)
3. **Provide DMA setup code** for Strategy 3 (if you want true overlap)

---

## Quick Decision Guide

**Choose Strategy 1 if:**
- You want it working quickly
- Sequential playback is acceptable
- You want simple, maintainable code

**Choose Strategy 2 if:**
- You need non-blocking main loop
- You want better sensor processing
- You're comfortable with interrupts

**Choose Strategy 3 if:**
- You need true overlapping sounds
- You have time for complex implementation
- You want professional audio quality

**Choose Strategy 4 if:**
- You need simultaneous multiple sounds
- You have significant development time
- You want the most advanced solution

