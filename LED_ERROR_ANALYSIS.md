# LED Error Analysis - BNO085 Initialization Issue

## Problem
The `led_error` signal is asserting, indicating the BNO085 controller is entering `ERROR_STATE` during initialization.

## Root Cause Analysis

### Key Difference: PS0/WAKE Pin Handling

**Old Working Code (`Old_SPI_test_xa/bno085_controller.sv`):**
1. Has `INIT_WAKE` state that drives PS0/WAKE LOW to wake the sensor
2. Sequence: `INIT_WAIT_RESET` → `INIT_WAKE` (PS0 LOW) → `INIT_WAIT_INT` → `INIT_CS_SETUP` (PS0 HIGH)
3. PS0/WAKE is controlled as an output pin

**Current Code (`fpga/bno085_controller.sv`):**
1. **Removed** `INIT_WAKE` state entirely
2. Sequence: `INIT_WAIT_RESET` → `INIT_WAIT_INT` (skips wake sequence)
3. PS0/WAKE port removed (assumed hardwired to 3.3V)

### The Problem

The BNO085 sensor **requires a PS0/WAKE LOW-to-HIGH transition** to properly wake up and assert INT, even if the pin is hardwired to 3.3V. Without this transition:

1. Sensor may not wake up properly
2. INT signal may never assert
3. Controller times out in `INIT_WAIT_INT` state (600 cycles = 200µs timeout)
4. Controller enters `ERROR_STATE` → `led_error` asserts

### Error State Entry Point

```systemverilog
INIT_WAIT_INT: begin
    if (!int_n_sync) begin
        // INT asserted, sensor is ready
        state <= INIT_CS_SETUP;
    end else if (delay_counter >= 19'd600) begin
        // Timeout: 600 cycles @ 3MHz = 200 µs
        state <= ERROR_STATE;  // <-- LED ERROR HERE
    end
end
```

## Possible Solutions

### Option 1: Restore PS0/WAKE Control (If Pin is Actually Controllable)
- Add PS0/WAKE port back to `bno085_controller.sv`
- Restore `INIT_WAKE` state
- Drive PS0 LOW, wait for INT, then drive HIGH
- **Check**: Is PS0/WAKE actually hardwired, or can it be controlled?

### Option 2: Increase Timeout (If Sensor is Just Slow)
- Increase `INIT_WAIT_INT` timeout from 600 to larger value (e.g., 3000 cycles = 1ms)
- Sensor might need more time to assert INT if PS0/WAKE is always high
- **Risk**: May mask other issues

### Option 3: Remove INT Wait Requirement (If Sensor Always Ready)
- Skip `INIT_WAIT_INT` state if PS0/WAKE is hardwired high
- Assume sensor is always ready after reset delay
- Go directly to `INIT_CS_SETUP`
- **Risk**: May not work if sensor actually needs INT assertion

### Option 4: Hardware Check
- Verify if PS0/WAKE pin is actually hardwired to 3.3V
- If it's connected to an FPGA pin, we can control it
- If it's truly hardwired, we need a different approach

## Recommended Investigation Steps

1. **Check Hardware**: Verify PS0/WAKE pin connection
   - Is it connected to an FPGA pin or hardwired to 3.3V?
   - If FPGA pin, restore PS0/WAKE control

2. **Check INT Signal**: Verify INT is actually asserting
   - Add debug output or scope INT pin
   - Check if sensor is responding at all

3. **Check Timing**: Verify reset delays are correct
   - Controller waits 300k cycles (100ms) in `INIT_WAIT_RESET`
   - Then waits 600 cycles (200µs) for INT in `INIT_WAIT_INT`
   - Sensor might need more time

4. **Compare with Old Code**: Restore exact initialization sequence
   - Copy `INIT_WAKE` state from old code
   - Restore PS0/WAKE port and control logic

## Files to Check/Modify

1. `fpga/bno085_controller.sv` - Add back PS0/WAKE control
2. `fpga/drum_trigger_top.sv` - Add PS0/WAKE port connection
3. Verify hardware pin assignment for PS0/WAKE

## Next Steps

1. Determine if PS0/WAKE can be controlled or is truly hardwired
2. If controllable: Restore PS0/WAKE control logic from old code
3. If hardwired: Increase timeout or modify initialization sequence
4. Test on hardware to verify fix

