# Dual BNO085 Sensor Implementation

## ✅ Implementation Complete

### Pin Assignments

#### Shared Signals (Both Sensors)
- `sclk` - SPI clock (shared)
- `mosi` - SPI MOSI (shared)
- `bno085_rst_n` - Reset signal (shared)

#### Separate Signals (Per Sensor)
- **Sensor 1 (Right Hand)**:
  - `miso1` - SPI MISO
  - `cs_n1` - Chip select
  - `int1` - Interrupt
  
- **Sensor 2 (Left Hand)**:
  - `miso2` - SPI MISO
  - `cs_n2` - Chip select
  - `int2` - Interrupt

#### Removed Pins
- ❌ `ps0_1` - Removed (now internal signal `ps0_wake1` and `ps0_wake2`)
- ❌ `ps0_2` - Not needed

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│              drum_trigger_top_integrated                │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐         ┌──────────────┐            │
│  │ BNO085 Ctrl 1│         │ BNO085 Ctrl 2│            │
│  │ (Right Hand) │         │ (Left Hand)  │            │
│  └──────┬───────┘         └──────┬───────┘            │
│         │                        │                     │
│  ┌──────▼────────────────────────▼──────┐             │
│  │   drum_trigger_processor             │             │
│  │   - Processes both sensors           │             │
│  │   - Combines outputs                 │             │
│  └──────┬───────────────────────────────┘             │
│         │                                              │
│  ┌──────▼───────┐                                      │
│  │ drum_spi_    │                                      │
│  │ slave        │                                      │
│  └──────────────┘                                      │
└─────────────────────────────────────────────────────────┘
```

### Processing Pipeline

Each sensor has its own complete processing pipeline:

1. **Quaternion to Euler** - Converts quaternion to roll/pitch/yaw
2. **Yaw Normalization** - Applies calibration offset (both respond to same calibrate button)
3. **Zone Detection** - Detects drum zone based on yaw (different zones for left/right)
4. **Strike Detection** - Detects strike when gyro_y < -2500
5. **Drum Selection** - Selects drum code based on zone, pitch, and gyro_z

### Output Priority

1. **Kick Button** (highest priority) → Code 2
2. **Sensor 1 (Right Hand)** → Codes 0, 3-7 (snare, toms, cymbals)
3. **Sensor 2 (Left Hand)** → Codes 0, 1, 3-7 (snare, hi-hat, toms, cymbals)

### Calibration

- **Calibrate Button**: Sets yaw offset for **both sensors simultaneously**
- When pressed, both `yaw_normalizer` instances capture their current yaw values
- Matches C code behavior: `yawOffset1 = yaw1; yawOffset2 = yaw2;`

### Drum Codes by Sensor

| Code | Sensor 1 (Right) | Sensor 2 (Left) |
|------|------------------|-----------------|
| 0    | Snare            | Snare           |
| 1    | -                | Hi-hat          |
| 2    | Kick (button)     | Kick (button)   |
| 3    | High tom         | High tom        |
| 4    | Mid tom          | Mid tom         |
| 5    | Crash            | Crash           |
| 6    | Ride             | Ride            |
| 7    | Floor tom        | Floor tom       |

### Status LEDs

- `led_initialized`: HIGH when **both** sensors are initialized
- `led_error`: HIGH if **either** sensor has an error

## Files Modified

1. ✅ `drum_trigger_top_integrated.sv` - Added second BNO085 controller, removed ps0 pins
2. ✅ `drum_trigger_processor.sv` - Added dual sensor processing pipelines

## Testing Checklist

- [ ] Verify both sensors initialize correctly
- [ ] Test right hand triggers (sensor 1)
- [ ] Test left hand triggers (sensor 2) - especially hi-hat
- [ ] Test calibration button (should affect both sensors)
- [ ] Test kick button (should work regardless of sensor)
- [ ] Verify no conflicts when both sensors trigger simultaneously

