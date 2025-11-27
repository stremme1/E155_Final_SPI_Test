# FPGA Resource Optimization Plan

## Problem
Design synthesizes but doesn't map to device - too large for available resources.

## Root Cause Analysis

### Current Resource Usage:
1. **Dual Sensor Processing (2x everything)**:
   - 2x `quaternion_to_euler_dsp` - ~9 DSP blocks each = 18 DSP blocks
   - 2x `yaw_normalizer` - Multiple pipeline stages
   - 2x `drum_zone_detector` - Comparison logic
   - 2x `strike_detector` - Simple
   - 2x `drum_selector` - Simple
   - 2x `bno085_controller` - State machines
   - 2x `spi_master` - Simple

2. **Complex Operations**:
   - Quaternion to Euler: 5 pipeline stages, 9 multiplications per sensor
   - Yaw normalization: 3 pipeline stages
   - Total: ~8 pipeline stages per sensor = 16 stages total

## Solution: Time-Multiplexed Processing

### Approach
Process sensors **sequentially** instead of in parallel:
- Use a single processing pipeline
- Alternate between sensor 1 and sensor 2
- Store intermediate results for each sensor
- Combine outputs at the end

### Resource Savings:
- **~50% reduction** in DSP blocks (18 → 9)
- **~50% reduction** in pipeline registers
- **~50% reduction** in comparison logic
- **Minimal overhead**: Small mux logic and state storage

### Implementation Strategy:
1. Add sensor select signal (alternates each cycle)
2. Mux sensor inputs to single processing pipeline
3. Store pipeline outputs for each sensor separately
4. Process sensor 1, then sensor 2, alternating
5. Combine final outputs (prioritize sensor 1 if both trigger)

### Testability:
- Can test mux logic independently
- Can test each sensor path independently  
- Full system testbench still works
- Can verify alternating behavior

## Implementation Steps

1. **Create time-multiplexed drum_trigger_processor**:
   - Add sensor_select signal
   - Mux quaternion/gyro inputs
   - Single instance of each processing module
   - Store results per sensor
   - Output mux

2. **Update top-level module**:
   - Keep dual BNO085 controllers (they can run in parallel)
   - Use time-multiplexed processor
   - All other logic stays the same

3. **Benefits**:
   - Same functionality
   - ~50% resource reduction
   - Easier to test (can isolate each sensor)
   - Still supports both sensors
