# Dual BNO085 Sensor Analysis

## Current Implementation Status

### ✅ What's Implemented

1. **Calibration Button** ✅ FULLY IMPLEMENTED
   - **Location**: `drum_trigger_top_integrated.sv` lines 34, 124-131
   - **Function**: Sets yaw offset when pressed
   - **Implementation**: 
     - Button input: `calibrate_btn_n` (active low)
     - Debounced: `calibrate_btn_pulse` (50ms debounce @ 3MHz)
     - Connected to: `yaw_normalizer` module (line 64 in `drum_trigger_processor.sv`)
   - **Behavior**: When pressed, captures current yaw as offset (matches C code: `yawOffset1 = yaw1`)
   - **Status**: ✅ Working correctly

2. **Kick Button** ✅ FULLY IMPLEMENTED
   - **Location**: `drum_trigger_top_integrated.sv` lines 35, 133-140
   - **Function**: Bypasses processing, outputs kick drum (code 2) directly
   - **Implementation**:
     - Button input: `kick_btn_n` (active low)
     - Debounced: `kick_btn_pulse` (50ms debounce @ 3MHz)
     - Connected to: `drum_trigger_processor` (line 208)
   - **Behavior**: When pressed, outputs `drum_code = 2` immediately (matches C code line 217)
   - **Status**: ✅ Working correctly

### ❌ What's Missing

1. **Second BNO085 Sensor** ❌ NOT IMPLEMENTED
   - **Original C Code**: Uses TWO sensors
     - `address1 = 0x28` (right hand sensor)
     - `address2 = 0x29` (left hand sensor)
     - Both on same I2C bus (shared SDA/SCL, different addresses)
   - **Current FPGA**: Only ONE BNO085 controller
     - Single instance: `bno085_controller bno085_ctrl_inst` (line 161)
     - Only processes sensor 1 (right hand)
     - No support for sensor 2 (left hand)
   - **Impact**: 
     - Left hand drum triggers (hi-hat, etc.) won't work
     - Only right hand triggers work (snare, toms, cymbals)

## Signal Sharing Analysis

### Original C Implementation (I2C)
- **Shared Signals**: SDA, SCL (I2C bus)
- **Different**: I2C addresses (0x28 vs 0x29)
- **Method**: I2C addressing allows multiple devices on same bus

### Current FPGA Implementation (SPI)
- **Current**: Single SPI bus for one BNO085
- **To Support Two Sensors**: Would need:
  1. **Option A**: Two separate SPI buses (different CS pins)
     - `cs_n1` for sensor 1
     - `cs_n2` for sensor 2
     - Can share: SCK, MOSI, MISO (SPI supports multiple slaves)
  2. **Option B**: Time-multiplexed single SPI bus
     - Switch CS between sensors
     - More complex, requires state machine

### Recommended Approach: Option A (Two CS Pins)
- **Shared Signals** (can be shared):
  - `sclk` (SPI clock) - same for both sensors
  - `mosi` (SPI data out) - same for both sensors  
  - `miso` (SPI data in) - can be shared if CS ensures only one active
  - `ps0_wake` - can be shared (wake both sensors)
  - `bno085_rst_n` - can be shared (reset both sensors)
  
- **Separate Signals** (must be separate):
  - `cs_n1` and `cs_n2` (chip select - one per sensor)
  - `int1` and `int2` (interrupt - one per sensor)

## Implementation Requirements for Dual Sensors

### 1. Add Second BNO085 Controller
```systemverilog
// Second BNO085 Controller
bno085_controller bno085_ctrl_inst2 (
    .clk(clk),
    .rst_n(rst_n),
    .sclk(sclk2),      // Can share with sclk1
    .mosi(mosi2),      // Can share with mosi1
    .miso(miso2),      // Can share with miso1 (with proper CS)
    .cs_n(cs_n2),      // MUST be separate
    .ps0_wake(ps0_2),  // Can share with ps0_1
    .int_n(int2),      // MUST be separate
    // ... rest of signals
    .quat_valid(quat2_valid),
    .quat_w(quat2_w), .quat_x(quat2_x), .quat_y(quat2_y), .quat_z(quat2_z),
    .gyro_valid(gyro2_valid),
    .gyro_x(gyro2_x), .gyro_y(gyro2_y), .gyro_z(gyro2_z),
    .initialized(initialized2),
    .error(error2)
);
```

### 2. Update Drum Trigger Processor
```systemverilog
drum_trigger_processor drum_processor (
    // ... existing sensor 1 inputs
    .quat1_valid(quat1_valid),
    .quat1_w(quat1_w), .quat1_x(quat1_x), .quat1_y(quat1_y), .quat1_z(quat1_z),
    .gyro1_valid(gyro1_valid),
    .gyro1_x(gyro1_x), .gyro1_y(gyro1_y), .gyro1_z(gyro1_z),
    
    // NEW: Sensor 2 inputs
    .quat2_valid(quat2_valid),
    .quat2_w(quat2_w), .quat2_x(quat2_x), .quat2_y(quat2_y), .quat2_z(quat2_z),
    .gyro2_valid(gyro2_valid),
    .gyro2_x(gyro2_x), .gyro2_y(gyro2_y), .gyro2_z(gyro2_z),
    
    // ... rest
);
```

### 3. Update Top-Level Module
```systemverilog
module drum_trigger_top_integrated (
    // ... existing
    
    // BNO085 Sensor 1 SPI Interface
    output logic        sclk1, mosi1,
    input  logic        miso1,
    output logic        cs_n1, ps0_1, bno085_rst_n1,
    input  logic        int1,
    
    // NEW: BNO085 Sensor 2 SPI Interface
    output logic        sclk2, mosi2,  // Can share with sclk1/mosi1
    input  logic        miso2,          // Can share with miso1
    output logic        cs_n2, ps0_2,   // MUST be separate
    input  logic        int2,           // MUST be separate
    // bno085_rst_n can be shared
);
```

## Summary

| Feature | Original C | Current FPGA | Status |
|---------|-----------|--------------|--------|
| **Calibration Button** | ✅ Button 2 sets yaw offset | ✅ Implemented | ✅ **COMPLETE** |
| **Kick Button** | ✅ Button 1 outputs code 2 | ✅ Implemented | ✅ **COMPLETE** |
| **Sensor 1 (Right)** | ✅ address1=0x28 | ✅ Implemented | ✅ **COMPLETE** |
| **Sensor 2 (Left)** | ✅ address2=0x29 | ❌ Not implemented | ❌ **MISSING** |
| **Dual Sensor Logic** | ✅ Both sensors processed | ❌ Only sensor 1 | ❌ **MISSING** |

## Recommendation

**For Initial Deployment**: Current single-sensor implementation is sufficient for right-hand drum triggers. Left-hand triggers (hi-hat) won't work, but all right-hand drums (snare, toms, cymbals) will work correctly.

**For Full Functionality**: Need to add second BNO085 controller and update `drum_trigger_processor` to accept both sensor inputs and implement left-hand logic (hi-hat detection).

