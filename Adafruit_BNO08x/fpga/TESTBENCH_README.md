# Testbench Documentation

This directory contains comprehensive testbenches for verifying the BNO08X FPGA implementation.

## Testbench Files

### 1. `tb_bno08x_spi_master.sv`
Tests the SPI master interface module.

**Tests:**
- Single byte write
- Multi-byte write (SHTP header)
- Single byte read
- Multi-byte read (SHTP response)
- Back-to-back transfers

**Usage:**
```bash
# Compile and run with your simulator
# Example for ModelSim/QuestaSim:
vlog tb_bno08x_spi_master.sv bno08x_spi_master.sv
vsim tb_bno08x_spi_master
run -all
```

### 2. `tb_bno08x_controller.sv`
Tests the BNO08X controller module with simulated BNO08X responses.

**Tests:**
- Initialization sequence
- Product ID request/response
- Sensor enable
- Sensor data reading

**Features:**
- Simulates BNO08X MISO responses
- Simulates H_INTN interrupt behavior
- Tests Product ID response
- Tests rotation vector report

**Usage:**
```bash
vlog tb_bno08x_controller.sv bno08x_controller.sv bno08x_spi_master.sv
vsim tb_bno08x_controller
run -all
```

### 3. `tb_sensor_processor.sv`
Tests sensor data processing and drum trigger logic.

**Tests:**
- Quaternion parsing
- Gyroscope parsing
- Drum trigger logic (right hand)
- Drum trigger logic (left hand)
- Yaw offset functionality

**Usage:**
```bash
vlog tb_sensor_processor.sv sensor_processor.sv
vsim tb_sensor_processor
run -all
```

### 4. `tb_bno08x_drum_system.sv`
Integration testbench for the complete drum system.

**Tests:**
- System initialization
- Sensor enable
- Drum trigger detection
- Yaw offset configuration

**Features:**
- Full system integration test
- Simulates complete BNO08X communication
- Tests end-to-end drum trigger functionality

**Usage:**
```bash
vlog tb_bno08x_drum_system.sv bno08x_drum_system.sv bno08x_controller.sv bno08x_spi_master.sv sensor_processor.sv
vsim tb_bno08x_drum_system
run -all
```

## Running All Testbenches

### For ModelSim/QuestaSim:
```bash
# Create a script to run all tests
vlog -work work *.sv
vsim -c work.tb_bno08x_spi_master -do "run -all; quit"
vsim -c work.tb_bno08x_controller -do "run -all; quit"
vsim -c work.tb_sensor_processor -do "run -all; quit"
vsim -c work.tb_bno08x_drum_system -do "run -all; quit"
```

### For Verilator:
```bash
verilator --cc --exe --build tb_bno08x_spi_master.sv bno08x_spi_master.sv
./obj_dir/Vtb_bno08x_spi_master
```

### For Icarus Verilog:
```bash
iverilog -o tb_bno08x_spi_master tb_bno08x_spi_master.sv bno08x_spi_master.sv
vvp tb_bno08x_spi_master
```

## Expected Results

### tb_bno08x_spi_master
- All SPI transfers should complete successfully
- CS, SCK, MOSI, MISO signals should show proper timing
- `transfer_done` should assert after each transfer

### tb_bno08x_controller
- `initialized` should go high after Product ID response
- `data_ready` should assert when sensor data is received
- No `error` signals should occur

### tb_sensor_processor
- Quaternion and gyroscope data should be parsed correctly
- Drum triggers should fire for appropriate orientations
- Yaw offset should be applied correctly

### tb_bno08x_drum_system
- Complete initialization sequence should succeed
- Drum triggers should be detected for test orientations
- System should remain stable throughout test

## Debugging Tips

1. **Check Clock Domain**: All modules use 3MHz clock (333ns period)

2. **SPI Timing**: Verify CS setup/hold times meet BNO08X requirements:
   - CS setup to CLK: 0.1μs min
   - CS hold: 16.83ns min

3. **Interrupt Handling**: H_INTN should be asserted before reading data

4. **Packet Format**: Verify SHTP header format matches datasheet:
   - Byte 0: Length LSB
   - Byte 1: Length MSB
   - Byte 2: Channel
   - Byte 3: Sequence

5. **Watch for Timeouts**: Some testbenches have timeout checks - adjust if needed

## Adding Custom Tests

To add custom test cases:

1. Create a new test task in the testbench
2. Set up input conditions
3. Wait for expected outputs
4. Use `$display` to report results
5. Use `$assert` or manual checks for verification

Example:
```systemverilog
task test_custom_scenario();
    $display("[%0t] Testing custom scenario", $time);
    // Set inputs
    input_signal = 1'b1;
    #(100 * CLK_PERIOD);
    // Check outputs
    if (output_signal == expected_value) begin
        $display("[%0t] ✓ Test passed", $time);
    end else begin
        $display("[%0t] ✗ Test failed", $time);
    end
endtask
```

## Notes

- All testbenches use `timescale 1ns / 1ps`
- Clock period is 333ns (3MHz)
- Reset is active low
- Some testbenches simulate BNO08X responses - adjust timing if needed for your simulator

