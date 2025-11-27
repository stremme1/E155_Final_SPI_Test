# Plan: Integrated Drum Trigger System with DSP Blocks and BRAM

## Overview
Integrate the C drum trigger logic into the existing FPGA SystemVerilog codebase (`Old_SPI_test_xa`), adding button inputs for calibration and kick drum, and optimizing for resource usage with DSP blocks and BRAM.

## Current System Architecture

### Existing Modules:
- `spi_test_top.sv` - Top-level module with clock, reset, SPI interface
- `bno085_controller.sv` - BNO085 SPI communication, outputs quaternion and gyroscope data
- `spi_master.sv` - SPI master interface

### Current Data Outputs from `bno085_controller.sv`:
- `quat_valid`, `quat_w`, `quat_x`, `quat_y`, `quat_z` (16-bit signed)
- `gyro_valid`, `gyro_x`, `gyro_y`, `gyro_z` (16-bit signed)

## Integration Plan

### Phase 1: Update Top-Level Module

#### 1.1 Modify `spi_test_top.sv`

**New Inputs**:
```systemverilog
input  logic        calibrate_btn_n,  // Calibration button (active low, with pull-up)
input  logic        kick_btn_n,       // Kick drum button (active low, with pull-up)
```

**New Outputs**:
```systemverilog
output logic        drum_trigger_valid,  // Valid drum trigger
output logic [3:0]  drum_code,          // 0-7 drum code
output logic        drum_hand,           // 0=right hand, 1=left hand
output logic [7:0]  debug_euler_yaw     // Debug: yaw angle (for testing)
```

**Internal Signals**:
- Button debounced signals
- Drum trigger processor outputs
- Multiple BNO085 sensor support (if needed)

**Module Structure**:
```systemverilog
module spi_test_top (
    input  logic        fpga_rst_n,
    
    // SPI Interface (BNO085)
    output logic        sclk1,
    output logic        mosi1,
    input  logic        miso1,
    output logic        cs_n1,
    output logic        ps0_1,
    output logic        bno085_rst_n1,
    input  logic        int1,
    
    // Button Inputs
    input  logic        calibrate_btn_n,  // Calibration button
    input  logic        kick_btn_n,       // Kick drum button
    
    // Drum Trigger Outputs
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,
    output logic        drum_hand,
    
    // Debug/Status LEDs
    output logic        led_initialized,
    output logic        led_error,
    output logic        led_heartbeat
);
```

### Phase 2: Button Input Module with Debouncing

#### 2.1 Button Debouncer Module
**File**: `button_debouncer.sv`

**Function**: Debounce button inputs (50ms debounce time @ 3MHz = 150,000 cycles)

**Implementation**:
- Use BRAM or registers for debounce counter
- Edge detection for button press/release
- Output clean button signal

**Module Interface**:
```systemverilog
module button_debouncer #(
    parameter DEBOUNCE_CYCLES = 150000  // 50ms @ 3MHz
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        btn_n,          // Active low button input
    output logic        btn_pressed,    // Single-cycle pulse on press
    output logic        btn_released,   // Single-cycle pulse on release
    output logic        btn_state       // Current debounced state
);
```

**Resource Usage**:
- Registers: ~20 bits (counter + state)
- LUTs: ~50 (logic)

### Phase 3: Quaternion to Euler Conversion (DSP + BRAM Optimized)

#### 3.1 Quaternion to Euler Converter
**File**: `quaternion_to_euler_dsp.sv`

**Algorithm** (from C code):
```
roll  = atan2(2*(w*x + y*z), 1 - 2*(x² + y²)) * 180/π
pitch = asin(2*(w*y - z*x)) * 180/π
yaw   = atan2(2*(w*z + x*y), 1 - 2*(y² + z²)) * 180/π
```

**DSP Block Usage**:
- **Multiplications**: Use DSP blocks for all multiplications
  - `w*x`, `y*z`, `w*y`, `z*x`, `w*z`, `x*y` (6 multiplications)
  - `x²`, `y²`, `z²` (3 multiplications)
  - Total: 9 multiplications → 9 DSP blocks
- **Additions**: Use DSP blocks' adders where possible
  - `w*x + y*z`, `w*y - z*x`, `w*z + x*y` (3 additions in DSP)
  - `1 - 2*(x² + y²)`, `1 - 2*(y² + z²)` (2 additions in DSP)

**BRAM Usage**:
- **CORDIC Lookup Tables**: Store CORDIC rotation angles and scaling factors
  - atan2 LUT: 512 entries × 32 bits = 2KB BRAM
  - asin LUT: 512 entries × 32 bits = 2KB BRAM
  - Total: ~4KB BRAM (1 BRAM block on iCE40UP5k)

**Fixed-Point Format**:
- Input quaternion: 16-bit signed (from BNO085, already scaled)
- Internal: Q16.15 (32-bit signed, 15 fractional bits)
- Output Euler: Q16.15 (32-bit signed, degrees)

**Module Interface**:
```systemverilog
module quaternion_to_euler_dsp (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [15:0] quat_w, quat_x, quat_y, quat_z,
    output logic        valid_out,
    output logic signed [31:0] roll,   // Q16.15 format, degrees
    output logic signed [31:0] pitch,   // Q16.15 format, degrees
    output logic signed [31:0] yaw     // Q16.15 format, degrees
);
```

**Pipeline Stages** (for DSP block usage):
1. **Stage 1**: Multiplications (DSP blocks, 1 cycle)
   - Compute all 9 multiplications in parallel
2. **Stage 2**: Additions (DSP blocks, 1 cycle)
   - Compute sums/differences
3. **Stage 3**: CORDIC atan2/asin (BRAM LUT, ~16 cycles)
   - Use CORDIC algorithm with BRAM lookup
4. **Stage 4**: Scale to degrees (multiply by 180/π, DSP block, 1 cycle)

**Total Latency**: ~19 cycles

**Resource Usage**:
- DSP Blocks: 10-12 (multiplications + scaling)
- BRAM: 1 block (4KB for CORDIC LUTs)
- LUTs: ~500 (control logic, CORDIC state machine)
- Registers: ~400 (pipeline registers)

### Phase 4: Yaw Normalization with Offset

#### 4.1 Yaw Normalizer Module
**File**: `yaw_normalizer.sv`

**Function**: 
- Subtract yaw offset (calibration)
- Normalize to 0-360 degrees (EXACT from C code's `normalizeYaw`)

**Algorithm** (EXACT from C code):
```c
float normalizeYaw(float yaw) {
    yaw = fmod(yaw, 360.0);  // Modulo 360
    if (yaw < 0) {
        yaw += 360.0;  // Ensure positive
    }
    return yaw;
}
```

**In SystemVerilog**:
```
yaw_adjusted = yaw - yaw_offset  // First subtract offset
yaw_normalized = yaw_adjusted mod 360  // Then normalize
if (yaw_normalized < 0) yaw_normalized += 360
```

**DSP Block Usage**:
- Subtraction: Use DSP block adder in subtract mode
- Modulo: Use DSP block for division/modulo (or combinational logic)

**Calibration Logic** (EXACT from C code):
- When calibrate button pressed (button2):
  1. Read current quaternion values
  2. Convert to Euler angles (roll, pitch, yaw)
  3. Set `yawOffset1 = yaw1` and `yawOffset2 = yaw2`
  4. Store offsets in registers (persists until next calibration)
- Yaw normalization: `yaw_normalized = normalizeYaw(yaw - yawOffset)`
  - `normalizeYaw`: `yaw = fmod(yaw, 360.0)`, if `yaw < 0` then `yaw += 360.0`

**Module Interface**:
```systemverilog
module yaw_normalizer (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [31:0] yaw,        // Q16.15
    input  logic        calibrate_pulse,   // Calibration button pulse
    output logic        valid_out,
    output logic [31:0] yaw_normalized,    // 0-360 degrees, Q16.15
    output logic signed [31:0] yaw_offset   // Current offset (for debug)
);
```

**Resource Usage**:
- DSP Blocks: 1 (for subtraction/modulo)
- LUTs: ~200 (normalization logic)
- Registers: ~100 (offset storage, pipeline)

### Phase 5: Zone Detection (Combinational Logic)

#### 5.1 Drum Zone Detector
**File**: `drum_zone_detector.sv`

**Function**: Determine drum zone from yaw angle (combinational, no DSP/BRAM needed)

**Zone Ranges** (Right Hand - EXACT from C code):
- Snare: `yaw >= 20 && yaw <= 120`
- High Tom: `yaw >= 340 || yaw <= 20` (wraps around)
- Mid Tom: `yaw >= 305 && yaw <= 340`
- Floor Tom: `yaw >= 200 && yaw <= 305`

**Zone Ranges** (Left Hand - EXACT from C code):
- Snare/Hi-hat: `yaw >= 350 || yaw <= 100` (wraps around)
- High Tom: `yaw >= 325 && yaw <= 350`
- Mid Tom: `yaw >= 300 && yaw <= 325`
- Floor Tom: `yaw >= 200 && yaw <= 300`

**Implementation**:
- Pure combinational logic (comparators)
- Handle wrap-around with OR logic
- Output zone enum

**Module Interface**:
```systemverilog
module drum_zone_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [31:0] yaw,           // 0-360, Q16.15
    input  logic        is_left_hand,  // 0=right, 1=left
    output logic        valid_out,
    output logic [2:0]  zone_id        // 0=snare, 1=high_tom, 2=mid_tom, 3=floor_tom
);
```

**Resource Usage**:
- LUTs: ~150 (comparators and logic)
- Registers: ~10 (pipeline)

### Phase 6: Strike Detection

#### 6.1 Strike Detector Module
**File**: `strike_detector.sv`

**Function**: Detect strikes using gyroscope Y-axis threshold (EXACT from C code)

**Logic**:
- Threshold: `gyro_y < -2500` (16-bit signed comparison)
- **Debounce flag**: `printedForGyro` prevents multiple triggers
  - When `gyro_y < -2500` AND `!printedForGyro`: Trigger strike, set `printedForGyro = true`
  - When `gyro_y >= -2500` AND `printedForGyro`: Reset `printedForGyro = false`
- This creates edge detection: only triggers once per strike until gyro returns above threshold

**Implementation**:
- Simple state machine
- No DSP/BRAM needed (just comparison)

**Module Interface**:
```systemverilog
module strike_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [15:0] gyro_y,
    output logic        strike_detected,  // Single-cycle pulse when strike occurs
    output logic        printed_flag      // High while printed (prevents retrigger)
);
```

**State Machine**:
- State: `printed_flag` (equivalent to C's `printedForGyro`)
- When `gyro_y < -2500` AND `!printed_flag`: Set `strike_detected = 1`, `printed_flag = 1`
- When `gyro_y >= -2500` AND `printed_flag`: Set `printed_flag = 0`

**Resource Usage**:
- LUTs: ~50 (state machine, comparators)
- Registers: ~20 (state, previous value)

### Phase 7: Drum Selection Logic

#### 7.1 Drum Selector Module
**File**: `drum_selector.sv`

**Function**: Combine zone, pitch, and gyro to select drum/cymbal

**Logic** (EXACT from C code):

**Right Hand**:
- **Snare zone** (20-120): Always code "0" (snare)
- **High Tom zone** (340-20):
  - If `pitch > 50`: code "5" (crash cymbal)
  - Else: code "3" (high tom)
- **Mid Tom zone** (305-340):
  - If `pitch > 50`: code "6" (ride cymbal)
  - Else: code "4" (mid tom)
- **Floor Tom zone** (200-305):
  - If `pitch > 30`: code "6" (ride cymbal)
  - Else: code "7" (floor tom)

**Left Hand**:
- **Snare/Hi-hat zone** (350-100):
  - If `pitch > 30` AND `gyro_z > -2000`: code "1" (hi-hat)
  - Else: code "0" (snare)
- **High Tom zone** (325-350):
  - If `pitch > 50`: code "5" (crash cymbal)
  - Else: code "3" (high tom)
- **Mid Tom zone** (300-325):
  - If `pitch > 50`: code "6" (ride cymbal)
  - Else: code "4" (mid tom)
- **Floor Tom zone** (200-300):
  - If `pitch > 30`: code "6" (ride cymbal)
  - Else: code "7" (floor tom)

**DSP Block Usage**:
- Pitch comparisons: Use DSP block comparators (if available) or LUTs

**Module Interface**:
```systemverilog
module drum_selector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [2:0]  zone_id,        // 0=snare, 1=high_tom, 2=mid_tom, 3=floor_tom
    input  logic signed [31:0] pitch,   // Q16.15, degrees
    input  logic signed [15:0] gyro_z,   // For left hand hi-hat detection
    input  logic        is_left_hand,    // 0=right, 1=left
    output logic        valid_out,
    output logic [3:0]  drum_code       // 0-7 (matches C code exactly)
);
```

**Drum Code Mapping** (EXACT from C code):
- 0: Snare drum
- 1: Hi-hat (left hand only, special condition)
- 2: Kick drum (from button, handled separately)
- 3: High tom
- 4: Mid tom
- 5: Crash cymbal
- 6: Ride cymbal
- 7: Floor tom

**Resource Usage**:
- DSP Blocks: 0-1 (for pitch comparisons, optional)
- LUTs: ~200 (selection logic)
- Registers: ~10 (pipeline)

### Phase 8: Top-Level Drum Trigger Processor

#### 8.1 Drum Trigger Processor Module
**File**: `drum_trigger_processor.sv`

**Function**: Integrate all components, handle kick button, output drum codes

**Data Flow** (EXACT from C code):
```
BNO085 Data (quat, gyro) 
    ↓
Quaternion to Euler (DSP + BRAM)
    ↓
Yaw Normalizer (yaw - yawOffset, then normalize to 0-360)
    ↓
Zone Detector (determines zone based on yaw ranges)
    ↓
Strike Detector (gyro_y < -2500 with printedForGyro flag)
    ↓
Drum Selector (zone + pitch + gyro_z → drum code)
    ↓
Drum Code Output (0-7)

Parallel:
Kick Button → Drum Code "2" (bypasses all processing)
```

**Kick Button Handling** (EXACT from C code):
- **Button 1** (kick_btn): When pressed (reading == 1), output drum code "2" (kick)
  - Debounce: 50ms delay, flag `buttonprinted1` prevents multiple triggers
  - When button released (reading == 0), reset flag
- **Button 2** (calibrate_btn): When pressed, capture current yaw values and set as offset
  - Reads quaternion → Euler → sets `yawOffset1` and `yawOffset2`
  - Debounce: 50ms delay, flag `buttonprinted2` prevents multiple triggers

**Module Interface**:
```systemverilog
module drum_trigger_processor (
    input  logic        clk,
    input  logic        rst_n,
    
    // BNO085 Sensor Inputs (Right Hand - Sensor 1)
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z,
    
    // Button Inputs (EXACT from C code)
    input  logic        calibrate_btn_pulse,  // Button 2: Calibration (sets yaw offset)
    input  logic        kick_btn_pulse,       // Button 1: Kick drum (outputs code "2")
    
    // Outputs
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,          // 0-7
    output logic        drum_hand            // 0=right, 1=left (currently always 0)
);
```

**Note**: 
- Currently designed for single sensor (right hand - sensor 1)
- C code supports two sensors (address1=0x28 for right, address2=0x29 for left)
- Can be extended for left hand (sensor 2) later
- Each sensor has independent `printedForGyro` flag (gyro1_y and gyro2_y)

**Resource Usage**:
- Total DSP Blocks: ~12-13
- Total BRAM: 1 block (4KB)
- Total LUTs: ~1200
- Total Registers: ~600

### Phase 9: Complete Integration

#### 9.1 Updated `spi_test_top.sv` Structure

```systemverilog
module spi_test_top (
    // ... existing ports ...
    input  logic        calibrate_btn_n,
    input  logic        kick_btn_n,
    output logic        drum_trigger_valid,
    output logic [3:0]  drum_code,
    output logic        drum_hand
);

    // Existing signals
    logic clk, rst_n;
    logic quat_valid, gyro_valid;
    logic signed [15:0] quat_w, quat_x, quat_y, quat_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    
    // New signals
    logic calibrate_btn_pulse, kick_btn_pulse;
    logic drum_trigger_valid_int;
    logic [3:0] drum_code_int;
    
    // Existing modules
    HSOSC hf_osc(...);
    spi_master spi_master_inst(...);
    bno085_controller bno085_ctrl_inst(...);
    
    // New modules
    button_debouncer #(.DEBOUNCE_CYCLES(150000)) calibrate_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(calibrate_btn_n),
        .btn_pressed(calibrate_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    button_debouncer #(.DEBOUNCE_CYCLES(150000)) kick_debouncer (
        .clk(clk),
        .rst_n(rst_n),
        .btn_n(kick_btn_n),
        .btn_pressed(kick_btn_pulse),
        .btn_released(),
        .btn_state()
    );
    
    drum_trigger_processor drum_processor (
        .clk(clk),
        .rst_n(rst_n),
        .quat1_valid(quat_valid),
        .quat1_w(quat_w),
        .quat1_x(quat_x),
        .quat1_y(quat_y),
        .quat1_z(quat_z),
        .gyro1_valid(gyro_valid),
        .gyro1_x(gyro_x),
        .gyro1_y(gyro_y),
        .gyro1_z(gyro_z),
        .calibrate_btn_pulse(calibrate_btn_pulse),
        .kick_btn_pulse(kick_btn_pulse),
        .drum_trigger_valid(drum_trigger_valid_int),
        .drum_code(drum_code_int),
        .drum_hand()
    );
    
    // Output assignments
    assign drum_trigger_valid = drum_trigger_valid_int;
    assign drum_code = drum_code_int;
    assign drum_hand = 1'b0; // Right hand for now
    
endmodule
```

## Resource Optimization Strategy

### DSP Block Usage Summary:
1. **Quaternion to Euler**: 10-12 DSP blocks
   - 9 multiplications (quaternion products)
   - 1-2 for scaling (180/π multiplication)
2. **Yaw Normalizer**: 1 DSP block
   - Subtraction and modulo
3. **Drum Selector**: 0-1 DSP blocks (optional)
   - Pitch comparisons
**Total: ~12-14 DSP blocks** (iCE40UP5k has 8 DSP blocks, may need to pipeline or share)

### BRAM Usage Summary:
1. **CORDIC LUTs**: 1 BRAM block (4KB)
   - atan2 lookup table
   - asin lookup table
**Total: 1 BRAM block** (iCE40UP5k has multiple BRAM blocks, plenty available)

### Optimization Techniques:
1. **Time-multiplex DSP blocks**: Share DSP blocks across pipeline stages
2. **Reduce precision**: Use Q8.7 instead of Q16.15 if acceptable
3. **Pipeline sharing**: Reuse CORDIC unit for atan2 and asin (sequential)
4. **LUT-based small multiplications**: Use LUTs for small constant multiplications

## Implementation Order

### Week 1: Core Infrastructure
1. Button debouncer module
2. Basic quaternion to Euler (proof of concept)
3. Integration test with existing BNO085 controller

### Week 2: Math Functions with DSP/BRAM
1. CORDIC implementation with BRAM LUTs
2. DSP block integration for multiplications
3. Yaw normalizer with calibration

### Week 3: Detection Logic
1. Zone detector
2. Strike detector
3. Drum selector

### Week 4: Integration & Optimization
1. Top-level drum trigger processor
2. Integration with spi_test_top
3. Resource optimization (DSP/BRAM sharing)
4. Testing and debugging

## File Structure

```
Old_SPI_test_xa/
├── spi_test_top.sv              (updated - add buttons, drum outputs)
├── bno085_controller.sv         (existing - no changes)
├── spi_master.sv                 (existing - no changes)
├── button_debouncer.sv          (new)
├── drum_trigger_processor.sv    (new - top-level integration)
├── quaternion_to_euler_dsp.sv  (new - with DSP blocks)
├── yaw_normalizer.sv             (new)
├── drum_zone_detector.sv         (new)
├── strike_detector.sv             (new)
├── drum_selector.sv              (new)
└── cordic_atan2.sv               (new - BRAM-based CORDIC)
```

## Testing Strategy

### Unit Tests:
1. Button debouncer: Test debounce timing
2. Quaternion to Euler: Test with known quaternion values
3. Yaw normalizer: Test calibration and wrap-around
4. Zone detector: Test all zone boundaries
5. Strike detector: Test threshold detection
6. Drum selector: Test all drum/cymbal combinations

### Integration Tests:
1. End-to-end: BNO085 → Drum Code
2. Button integration: Calibration and kick
3. Timing: Verify pipeline latency (~25 cycles)
4. Resource usage: Verify DSP/BRAM usage

## Exact C Code Logic Summary

### Quaternion to Euler Conversion (EXACT from bno055.c):
```c
roll  = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / M_PI;
pitch = asin(2.0 * (w * y - z * x)) * 180.0 / M_PI;
yaw   = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / M_PI;
```

### Yaw Normalization (EXACT from main.c):
```c
float normalizeYaw(float yaw) {
    yaw = fmod(yaw, 360.0);
    if (yaw < 0) {
        yaw += 360.0;
    }
    return yaw;
}
// Usage: yaw1 = normalizeYaw(yaw1 - yawOffset1);
```

### Strike Detection (EXACT from main.c):
- Threshold: `gyro_y < -2500` (16-bit signed)
- Flag: `printedForGyro` prevents multiple triggers
- Reset: When `gyro_y >= -2500`, reset flag

### Zone Ranges (EXACT from main.c):
**Right Hand (yaw1)**:
- Snare: `yaw1 >= 20 && yaw1 <= 120`
- High Tom: `yaw1 >= 340 || yaw1 <= 20`
- Mid Tom: `yaw1 >= 305 && yaw1 <= 340`
- Floor Tom: `yaw1 >= 200 && yaw1 <= 305`

**Left Hand (yaw2)**:
- Snare/Hi-hat: `yaw2 >= 350 || yaw2 <= 100`
- High Tom: `yaw2 >= 325 && yaw2 <= 350`
- Mid Tom: `yaw2 >= 300 && yaw2 <= 325`
- Floor Tom: `yaw2 >= 200 && yaw2 <= 300`

### Drum Selection (EXACT from main.c):
**Right Hand**:
- Snare zone: Always code "0"
- High Tom zone: `pitch1 > 50` → "5" (crash), else → "3" (high tom)
- Mid Tom zone: `pitch1 > 50` → "6" (ride), else → "4" (mid tom)
- Floor Tom zone: `pitch1 > 30` → "6" (ride), else → "7" (floor tom)

**Left Hand**:
- Snare/Hi-hat zone: `pitch2 > 30 && gyro2_z > -2000` → "1" (hi-hat), else → "0" (snare)
- High Tom zone: `pitch2 > 50` → "5" (crash), else → "3" (high tom)
- Mid Tom zone: `pitch2 > 50` → "6" (ride), else → "4" (mid tom)
- Floor Tom zone: `pitch2 > 30` → "6" (ride), else → "7" (floor tom)

### Button Logic (EXACT from main.c):
- **Button 1** (button1=pin 6): Kick drum → Output code "2"
  - Debounce: 50ms, flag `buttonprinted1`
- **Button 2** (button2=pin 7): Calibration → Set yaw offsets
  - Reads quaternion → Euler → Sets `yawOffset1 = yaw1`, `yawOffset2 = yaw2`
  - Debounce: 50ms, flag `buttonprinted2`

### Drum Code Mapping (EXACT from main.c):
- "0": Snare drum
- "1": Hi-hat (left hand only)
- "2": Kick drum (button or piezo)
- "3": High tom
- "4": Mid tom
- "5": Crash cymbal
- "6": Ride cymbal
- "7": Floor tom

## Notes

- **iCE40UP5k DSP Blocks**: 8 available, may need time-multiplexing
- **iCE40UP5k BRAM**: Multiple blocks available (30 blocks × 4KB = 120KB total)
- **Clock Domain**: All modules at 3MHz system clock
- **Button Pull-ups**: External pull-ups required (or use FPGA internal pull-ups)
- **BNO085 Data Rate**: 50Hz (20ms period), plenty of time for processing
- **Latency**: ~25 cycles @ 3MHz = ~8.3μs (negligible compared to 20ms data period)
- **C Code Reference**: All logic matches `Code_for_C_imp/main.c` exactly

## Future Enhancements

- Support for second sensor (left hand)
- Configurable thresholds via registers
- Velocity-sensitive triggering
- MIDI output interface
- Calibration persistence (store in BRAM)

