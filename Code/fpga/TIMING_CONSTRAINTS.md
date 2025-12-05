# Timing Constraints and CDC Analysis

## Clock Domains

### Domain 1: FPGA System Clock (clk)
- **Frequency**: 3MHz (48MHz HSOSC / 16)
- **Period**: 333.33ns
- **Source**: HSOSC primitive in `drum_trigger_top.sv`
- **Used by**: All internal FPGA logic, CDC synchronizers

### Domain 2: Arduino SCK (arduino_sck)
- **Frequency**: 100kHz (from Arduino SPI settings)
- **Period**: 10us
- **Source**: External (Arduino master)
- **Used by**: `arduino_spi_slave.sv` receive logic
- **Relationship to clk**: Asynchronous (30:1 ratio)

### Domain 3: MCU SCK (mcu_sck)
- **Frequency**: Variable (MCU controlled)
- **Source**: External (MCU master)
- **Used by**: `spi_slave_mcu.sv` transmit logic
- **Relationship to clk**: Asynchronous

## Clock Domain Crossings (CDC)

### CDC 1: Arduino SCK → FPGA clk
**Location**: `arduino_spi_slave.sv` lines 118-178

**Path**: `packet_buffer` (SCK domain) → `packet_snapshot` (clk domain)

**CDC Strategy**: CS-based safe read
- When CS goes high, SCK is guaranteed idle (SPI Mode 0: CPOL=0)
- Wait 3 clk cycles (1us) after CS high before reading
- Atomic read of all 16 bytes in one clock cycle

**Timing Analysis**:
- Worst case: Last SCK edge to CS high: < 1 SCK period (10us)
- 3 clk cycles at 3MHz: 3 × 333ns = 1us
- Margin: 10:1 (very safe)

**Metastability Risk**: LOW
- Data is guaranteed stable (CS high = SCK idle)
- 3-cycle delay provides ample settling time
- Atomic read prevents partial updates

### CDC 2: FPGA clk → MCU SCK
**Location**: `spi_slave_mcu.sv` lines 133-161

**Path**: Sensor data (clk domain) → `packet_buffer` (MCU SCK domain)

**CDC Strategy**: Continuous snapshot update
- Snapshot updated continuously when CS is high
- Frozen when CS is low (during transaction)
- Data captured from registered clk-domain signals

**Timing Analysis**:
- Data is registered in clk domain before snapshot
- Snapshot only updates when CS is high (MCU not reading)
- Safe because MCU only reads when CS is low (snapshot frozen)

**Metastability Risk**: LOW
- Snapshot updates only when transaction is not active
- Data is registered before snapshot

## Recommended Timing Constraints

### For Synthesis Tools

```tcl
# Clock definitions
create_clock -name clk -period 333.33 [get_ports clk]
create_clock -name arduino_sck -period 10000 [get_ports arduino_sck]
create_clock -name mcu_sck -period 1000 [get_ports mcu_sck]  # Example: 1MHz

# Clock groups (asynchronous)
set_clock_groups -asynchronous \
    -group [get_clocks clk] \
    -group [get_clocks arduino_sck] \
    -group [get_clocks mcu_sck]

# CDC paths - set false paths for safe CDC reads
# arduino_spi_slave: packet_buffer to packet_snapshot (safe when CS high)
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *packet_buffer*}] \
               -to [get_cells -hierarchical -filter {NAME =~ *packet_snapshot*}] \
               -setup -hold

# spi_slave_mcu: snapshot updates (safe when CS high)
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *snap*}] \
               -to [get_cells -hierarchical -filter {NAME =~ *packet_buffer*}] \
               -setup -hold
```

### For Static Timing Analysis

- **Setup Time**: Standard cell library defaults (typically 0.5-1ns)
- **Hold Time**: Standard cell library defaults (typically 0.1-0.3ns)
- **CDC Timing**: False paths set for safe CDC reads (see above)

## Timing Margins

### Arduino SPI (100kHz)
- SPI bit period: 10us
- FPGA clk period: 333ns
- Clocks per SPI bit: 30
- **Margin**: Excellent (30:1 ratio)

### CDC Delays
- CS high to read: 3 cycles = 1us
- Worst-case SPI timing: 10us
- **Margin**: 10:1 (very safe)

### MCU SPI
- Variable frequency (MCU controlled)
- Snapshot updates only when CS high (transaction inactive)
- **Margin**: Adequate (data stable before transaction)

## Verification

### Simulation
- Testbench uses realistic clock ratios
- CDC delays verified in simulation
- All test cases pass

### Hardware Testing
- ✅ Verify no data corruption over extended periods
- ✅ Test with various SPI clock rates
- ✅ Stress test with rapid packet transmission
- ✅ Monitor for metastability (LED indicators)

## Notes

- Current CDC implementation is pragmatic and safe for this application
- For higher reliability requirements, consider FIFO-based CDC
- Current approach balances simplicity and correctness
- All timing margins are conservative (10:1 or better)

