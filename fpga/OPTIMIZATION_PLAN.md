# FPGA Resource Optimization Plan

## Problem
- **Available DSP blocks**: 8
- **Current usage**: ~24 DSP blocks (2x quaternion_to_euler_dsp with ~12 DSP each)
- **Design doesn't fit**: Need to reduce to ≤8 DSP blocks

## Solution: Time-Multiplexed + Pipelined DSP

### Strategy
1. **Time-multiplex quaternion processing**: Use 1 shared module instead of 2 (saves 12 DSP)
2. **Pipeline multiplications**: Process 9 multiplications in 3 cycles using 3 DSP blocks (saves 6 DSP)
3. **Total DSP usage**: 6 blocks ✅ (fits in 8 limit!)

### Implementation

#### 1. Optimized Quaternion Module (`quaternion_to_euler_dsp_optimized.sv`)
- **Stage 1**: 9 multiplications pipelined into 3 cycles using 3 DSP blocks
  - Cycle 1: w_x, y_z, w_y (3 DSP)
  - Cycle 2: z_x, w_z, x_y (3 DSP, reused)
  - Cycle 3: x_sq, y_sq, z_sq (3 DSP, reused)
- **Stage 5**: 3 multiplications in 1 cycle using 3 DSP blocks
- **Total**: 3 DSP (stage 1) + 3 DSP (stage 5) = **6 DSP blocks**

#### 2. Time-Multiplexed Processor (`drum_trigger_processor_optimized.sv`)
- Single shared `quaternion_to_euler_dsp_optimized` instance
- Time-multiplexes between sensor 1 and sensor 2
- Buffers outputs for each sensor
- Rest of pipeline unchanged (yaw_normalizer, zone_detector, etc.)

### DSP Block Allocation
- **quaternion_to_euler_dsp_optimized**: 6 DSP blocks (pipelined)
- **Other modules**: 0 DSP blocks
- **Total**: **6 DSP blocks** ✅ (fits in 8!)

### Files Created
1. `quaternion_to_euler_dsp_optimized.sv` - Pipelined version (6 DSP blocks)
2. `drum_trigger_processor_optimized.sv` - Time-multiplexed version
3. Testbench (to be created)

### Testing Strategy
1. **Unit test**: `quaternion_to_euler_dsp_optimized` with pipelined multiplications
2. **Integration test**: Time-multiplexed dual sensor processing
3. **System test**: Full drum trigger system with optimized DSP usage

### Usage Instructions
1. Replace `quaternion_to_euler_dsp` with `quaternion_to_euler_dsp_optimized` in top module
2. Replace `drum_trigger_processor` with `drum_trigger_processor_optimized` in top module
3. Update top module instantiation

### Expected Results
- **DSP blocks used**: 6 (within 8 limit) ✅
- **Latency**: Slightly increased (3 cycles for multiplications instead of 1, but shared)
- **Functionality**: Same (both sensors still processed)
- **Testability**: Maintained (can test with testbench)
