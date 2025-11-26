# Debug Checklist for No Signals on FPGA

## Critical Issues to Check:

### 1. Clock Generation (HSOSC)
- [ ] Verify HSOSC primitive is recognized by synthesis tool
- [ ] Check synthesis log for HSOSC warnings/errors
- [ ] Verify CLKHFPU and CLKHFEN are both set to 1'b1
- [ ] Check if clock constraint is being applied (see constraints.pdc)
- [ ] Verify clock net is being routed (check implementation report)

### 2. Clock Constraint
- [ ] Clock constraint added to constraints.pdc (line 5)
- [ ] Period = 333.333ns for 3MHz clock
- [ ] Check if synthesis tool recognizes the constraint

### 3. Reset Signal
- [ ] Verify rst_n pin is connected and not stuck low
- [ ] Check if reset button/pin is working
- [ ] Verify reset is released (goes high) after power-on

### 4. Output Pins
- [ ] Verify all output pins are correctly assigned in constraints.pdc
- [ ] Check if pins are being used by other modules
- [ ] Verify LED pins are correct (39, 40, 41)

### 5. Synthesis/Implementation
- [ ] Check synthesis log for errors/warnings
- [ ] Verify no signals are optimized away
- [ ] Check timing constraints are met
- [ ] Verify bitstream was generated successfully

### 6. Hardware Connections
- [ ] Verify FPGA is powered
- [ ] Check if programming was successful
- [ ] Verify BNO085 sensor is connected correctly
- [ ] Check if reset button works

## Quick Test:

To verify clock is working, the heartbeat LED (led_heartbeat on pin 40) should blink.
If this doesn't blink, the clock is not working.

## Common Issues:

1. **HSOSC not recognized**: Some tools need explicit primitive library
2. **Clock constraint syntax**: May need different syntax for your tool
3. **Reset stuck**: Reset pin might be inverted or stuck
4. **Optimization**: Signals might be optimized away if unused

## Next Steps:

1. Check synthesis log for HSOSC-related errors
2. Try a minimal design with just HSOSC and LED to verify clock works
3. Verify reset signal with oscilloscope/logic analyzer
4. Check if clock net appears in implementation report

