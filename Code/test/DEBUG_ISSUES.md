# Testbench Debug Issues

## Current Status
**Unit Test**: FAILING - Controller stuck in ST_RESET state
- Tests: 1 PASS, 4 FAIL
- Controller never progresses past ST_RESET
- `ps0_wake=1` throughout (should go low in ST_WAKE state)
- `int_n=1` throughout (INT never asserted or not seen)

## Root Cause Analysis

### Issue 1: Controller Stuck in ST_RESET
The controller should:
1. Wait 300,000 cycles in ST_RESET (incrementing delay_counter)
2. Transition to ST_WAKE (drive ps0_wake low for 450 cycles)
3. Transition to ST_WAIT_INT (wait for INT to go low)
4. Transition to ST_SEND_CMD when INT is detected

**Problem**: Controller never leaves ST_RESET state. `delay_counter` may not be incrementing, or state transition logic has an issue.

### Issue 2: INT Timing
- Mock sensor asserts INT at cycle 301,000
- Controller enters ST_WAIT_INT at cycle 300,450 (300k + 450)
- INT should be visible, but controller doesn't see it

**Possible causes**:
- INT synchronization issue (`int_n_sync` not updating)
- INT being deasserted before controller checks it
- Timing mismatch

## Debug Steps

1. **Check delay_counter incrementing**:
   - Add debug output to show delay_counter value
   - Verify it increments from 0 to 300,000

2. **Check state transitions**:
   - Add state name to debug output
   - Verify ST_RESET -> ST_WAKE -> ST_WAIT_INT transitions

3. **Check INT signal**:
   - Verify INT goes low at correct time
   - Verify `int_n_sync` updates correctly
   - Check for race conditions

4. **Use VCD waveform**:
   - Run test with VCD dump enabled
   - Open in GTKWave to see actual signal behavior
   - Check state machine, counters, and INT timing

## Files to Check

- `fpga/bno085_controller.sv`: State machine logic, delay_counter increment
- `test/mocks/mock_bno085.sv`: INT assertion timing, CS deassertion logic
- `test/fpga/tb_bno085_controller_unit.sv`: Test timing, debug output

## Next Steps

1. Add more detailed debug output to controller
2. Simplify test to verify basic state machine operation
3. Check if delay_counter width is sufficient (19 bits for 300k)
4. Verify clock is running and state machine is active

