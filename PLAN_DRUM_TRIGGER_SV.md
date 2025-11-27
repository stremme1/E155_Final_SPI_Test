# Plan: Convert C Drum Trigger Code to SystemVerilog for FPGA

## Overview
Convert the C-based drum trigger system (`Code_for_C_imp/main.c`) to SystemVerilog to run on FPGA, processing quaternion and gyroscope data from BNO085 sensors.

## Current C Code Functionality

### Input Data (from BNO085):
- **Quaternion**: w, x, y, z (normalized to -1.0 to 1.0, scaled by 16384)
- **Gyroscope**: x, y, z (16-bit signed integers, rad/s)
- **Buttons**: Digital inputs for kick drum and calibration
- **Piezo**: Analog input for kick drum (optional)

### Processing Steps:
1. **Quaternion to Euler Conversion**: Converts quaternion to roll, pitch, yaw (degrees)
2. **Yaw Normalization**: Wraps yaw to 0-360 degrees
3. **Zone Detection**: Uses yaw angle to determine drum zone (snare, tom, cymbal areas)
4. **Strike Detection**: Uses gyroscope Y-axis threshold (< -2500) to detect strikes
5. **Drum Selection**: Combines zone + pitch to select drum/cymbal type
6. **Output**: Serial codes 0-7 representing different drums/cymbals

### Drum Mapping:
- **0**: Snare drum
- **1**: Hi-hat
- **2**: Kick drum
- **3**: High tom
- **4**: Mid tom
- **5**: Crash cymbal
- **6**: Ride cymbal
- **7**: Floor tom

## SystemVerilog Implementation Plan

### Phase 1: Core Math Functions (Fixed-Point Arithmetic)

#### 1.1 Quaternion to Euler Converter Module
**File**: `quaternion_to_euler.sv`

**Requirements**:
- Fixed-point arithmetic (avoid floating point)
- Input: 16-bit signed quaternion components (from BNO085)
- Output: Roll, Pitch, Yaw in degrees (fixed-point format)

**Algorithm** (from `bno055_quaternion_to_euler`):
```
roll = atan2(2*(w*x + y*z), 1 - 2*(x² + y²)) * 180/π
pitch = asin(2*(w*y - z*x)) * 180/π
yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²)) * 180/π
```

**Implementation Strategy**:
- Use CORDIC algorithm for atan2 and asin (standard FPGA approach)
- Or use lookup tables (LUT) for trigonometric functions
- Fixed-point format: Q16.15 (16 integer bits, 15 fractional bits) for angles
- Scale quaternion inputs: divide by 16384 (right shift 14 bits)

**Module Interface**:
```systemverilog
module quaternion_to_euler (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [15:0] quat_w, quat_x, quat_y, quat_z,
    output logic        valid_out,
    output logic signed [31:0] roll,   // Q16.15 format
    output logic signed [31:0] pitch, // Q16.15 format
    output logic signed [31:0] yaw    // Q16.15 format
);
```

#### 1.2 Yaw Normalization Module
**File**: `yaw_normalizer.sv`

**Function**: Normalize yaw to 0-360 degrees range
```
yaw_normalized = (yaw - offset) mod 360
if (yaw_normalized < 0) yaw_normalized += 360
```

**Implementation**:
- Use modulo operation (remainder after division)
- Fixed-point arithmetic
- Handle wrap-around (340-20 range)

**Module Interface**:
```systemverilog
module yaw_normalizer (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [31:0] yaw,        // Q16.15
    input  logic signed [31:0] yaw_offset, // Q16.15
    output logic        valid_out,
    output logic [31:0] yaw_normalized     // 0-360 degrees, Q16.15
);
```

### Phase 2: Zone Detection Logic

#### 2.1 Drum Zone Detector Module
**File**: `drum_zone_detector.sv`

**Function**: Determine drum zone based on yaw angle

**Zone Mapping** (Right Hand):
- Snare: 20° - 120°
- High Tom: 340° - 20° (wraps around)
- Mid Tom: 305° - 340°
- Floor Tom: 200° - 305°

**Zone Mapping** (Left Hand):
- Snare/Hi-hat: 350° - 100° (wraps around)
- High Tom: 325° - 350°
- Mid Tom: 300° - 325°
- Floor Tom: 200° - 300°

**Implementation**:
- Combinational logic with range comparisons
- Handle wrap-around cases (e.g., 340-20)
- Output zone ID (enum or one-hot encoding)

**Module Interface**:
```systemverilog
module drum_zone_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [31:0] yaw,      // Normalized yaw (0-360)
    input  logic        is_left_hand, // 0=right, 1=left
    output logic        valid_out,
    output logic [2:0]  zone_id   // 0=snare, 1=high_tom, 2=mid_tom, 3=floor_tom
);
```

### Phase 3: Strike Detection

#### 3.1 Strike Detector Module
**File**: `strike_detector.sv`

**Function**: Detect drum strikes using gyroscope threshold

**Logic**:
- Monitor gyro_y (Y-axis gyroscope)
- Trigger when gyro_y < -2500 (threshold)
- Debounce: prevent multiple triggers until gyro_y >= -2500
- Edge detection: trigger on falling edge below threshold

**Implementation**:
- Compare gyro_y with threshold (-2500)
- State machine: IDLE → STRIKE_DETECTED → WAIT_RELEASE
- Output strike_valid pulse when strike detected

**Module Interface**:
```systemverilog
module strike_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic signed [15:0] gyro_y,
    output logic        strike_detected, // Pulse when strike occurs
    output logic        strike_active   // High while strike is active
);
```

### Phase 4: Drum Selection Logic

#### 4.1 Drum Selector Module
**File**: `drum_selector.sv`

**Function**: Combine zone, pitch, and gyro to select drum/cymbal

**Logic**:
- Input: zone_id, pitch, gyro_z, is_left_hand
- Use pitch to differentiate drums from cymbals:
  - Pitch > 50°: Cymbal (crash/ride)
  - Pitch > 30°: Ride cymbal (for floor tom zone)
  - Otherwise: Drum
- Special cases:
  - Left hand: pitch > 30° AND gyro_z > -2000 → Hi-hat
  - Otherwise: Snare

**Output Codes**:
- 0: Snare
- 1: Hi-hat
- 2: Kick (from button/piezo, separate module)
- 3: High tom
- 4: Mid tom
- 5: Crash cymbal
- 6: Ride cymbal
- 7: Floor tom

**Module Interface**:
```systemverilog
module drum_selector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [2:0]  zone_id,
    input  logic signed [31:0] pitch,    // Q16.15
    input  logic signed [15:0] gyro_z,
    input  logic        is_left_hand,
    output logic        valid_out,
    output logic [3:0]  drum_code       // 0-7
);
```

### Phase 5: Integration Module

#### 5.1 Drum Trigger Processor Module
**File**: `drum_trigger_processor.sv`

**Function**: Top-level module that integrates all components

**Data Flow**:
```
BNO085 Data → Quaternion to Euler → Yaw Normalizer → Zone Detector
                                                      ↓
Gyroscope Data → Strike Detector ← → Drum Selector → Output
```

**Module Interface**:
```systemverilog
module drum_trigger_processor (
    input  logic        clk,
    input  logic        rst_n,
    
    // BNO085 Sensor 1 (Right Hand) Inputs
    input  logic        quat1_valid,
    input  logic signed [15:0] quat1_w, quat1_x, quat1_y, quat1_z,
    input  logic        gyro1_valid,
    input  logic signed [15:0] gyro1_x, gyro1_y, gyro1_z,
    
    // BNO085 Sensor 2 (Left Hand) Inputs
    input  logic        quat2_valid,
    input  logic signed [15:0] quat2_w, quat2_x, quat2_y, quat2_z,
    input  logic        gyro2_valid,
    input  logic signed [15:0] gyro2_x, gyro2_y, gyro2_z,
    
    // Calibration Inputs
    input  logic        calibrate_btn,  // Button to set yaw offset
    
    // Outputs
    output logic        drum_valid,
    output logic [3:0]  drum_code,      // 0-7
    output logic        is_left_hand     // 0=right, 1=left
);
```

### Phase 6: Top-Level Integration

#### 6.1 Update `spi_test_top.sv`

**Changes**:
1. Instantiate `drum_trigger_processor` module
2. Connect BNO085 controller outputs to drum trigger processor
3. Add button inputs for kick drum and calibration
4. Add drum code outputs (could drive LEDs or external interface)

**New Interface**:
```systemverilog
module spi_test_top (
    // ... existing ports ...
    
    // Drum trigger inputs
    input  logic        kick_btn,        // Kick drum button
    input  logic        calibrate_btn,  // Calibration button
    
    // Drum trigger outputs
    output logic        drum_valid,
    output logic [3:0]  drum_code,
    output logic        drum_hand        // 0=right, 1=left
);
```

## Implementation Details

### Fixed-Point Number Format
- **Q16.15**: 16 integer bits, 15 fractional bits
- Range: -32768.0 to 32767.999...
- Precision: ~0.00003 (1/32768)
- Suitable for angles in degrees (-180 to 180, or 0 to 360)

### CORDIC Algorithm for Trigonometry
- Standard FPGA approach for atan2, asin, cos, sin
- Iterative algorithm, ~16-20 cycles per calculation
- No floating-point units needed
- Can use existing CORDIC IP cores if available

### Pipeline Design
- Stage 1: Quaternion to Euler (pipelined CORDIC, ~20 cycles)
- Stage 2: Yaw normalization (1 cycle)
- Stage 3: Zone detection (1 cycle, combinational)
- Stage 4: Strike detection (1 cycle, combinational)
- Stage 5: Drum selection (1 cycle, combinational)
- Total latency: ~22-25 clock cycles @ 3MHz = ~7-8 microseconds

### Resource Estimation
- **CORDIC units**: 3 (one for each Euler angle calculation)
- **Multipliers**: ~10-15 (for quaternion math)
- **LUTs**: ~2000-3000 (for logic and small LUTs)
- **Registers**: ~500-800 (for pipeline stages and state)

## Testing Strategy

### Unit Tests
1. **Quaternion to Euler**: Test with known quaternion values
2. **Yaw Normalizer**: Test wrap-around cases (340-20 range)
3. **Zone Detector**: Test all zone boundaries
4. **Strike Detector**: Test threshold detection and debouncing
5. **Drum Selector**: Test all drum/cymbal combinations

### Integration Tests
1. End-to-end: Quaternion → Drum Code
2. Timing: Verify pipeline latency
3. Edge cases: Boundary conditions, wrap-around

## File Structure

```
Old_SPI_test_xa/
├── spi_test_top.sv              (updated)
├── bno085_controller.sv         (existing)
├── spi_master.sv                 (existing)
├── drum_trigger_processor.sv    (new)
├── quaternion_to_euler.sv       (new)
├── yaw_normalizer.sv             (new)
├── drum_zone_detector.sv         (new)
├── strike_detector.sv             (new)
├── drum_selector.sv              (new)
└── cordic_atan2.sv               (new, if needed)
```

## Implementation Order

1. **Week 1**: Core math functions
   - CORDIC modules (atan2, asin)
   - Quaternion to Euler converter
   - Yaw normalizer

2. **Week 2**: Detection logic
   - Zone detector
   - Strike detector
   - Drum selector

3. **Week 3**: Integration
   - Drum trigger processor
   - Top-level integration
   - Testing and debugging

4. **Week 4**: Optimization
   - Pipeline optimization
   - Resource optimization
   - Performance tuning

## Notes

- **BNO085 vs BNO055**: The FPGA code uses BNO085 (SPI), while C code uses BNO055 (I2C). Data formats are similar but may need scaling adjustments.
- **Quaternion Scaling**: BNO085 outputs quaternion as 16-bit signed integers. Need to verify scaling factor (may be different from BNO055's 16384).
- **Gyroscope Units**: Verify BNO085 gyroscope units (rad/s vs deg/s) and scaling.
- **Clock Domain**: All modules run at 3MHz system clock.
- **Button Debouncing**: May need separate debounce module for buttons.

## Future Enhancements

- Support for multiple sensors (currently designed for 2)
- Configurable thresholds (via registers)
- Velocity-sensitive triggering (based on gyro magnitude)
- MIDI output interface
- Calibration persistence (store yaw offsets)

