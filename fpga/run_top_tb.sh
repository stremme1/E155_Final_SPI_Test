#!/bin/bash

# Testbench Runner for Top-Level Module
# Compiles and runs tb_drum_trigger_top_integrated.sv

echo "=========================================="
echo "Top-Level Module Testbench"
echo "=========================================="
echo ""

# Check if iverilog is available
if ! command -v iverilog &> /dev/null; then
    echo "ERROR: iverilog not found. Please install iverilog."
    echo "On macOS: brew install icarus-verilog"
    exit 1
fi

# Compile all required modules
echo "Compiling testbench and modules..."
iverilog -g2012 -o tb_top_sim \
    hsosc_mock.sv \
    tb_drum_trigger_top_integrated.sv \
    drum_trigger_top_integrated.sv \
    drum_spi_slave.sv \
    drum_trigger_processor.sv \
    bno085_controller.sv \
    spi_master.sv \
    quaternion_to_euler_dsp.sv \
    yaw_normalizer.sv \
    drum_zone_detector.sv \
    strike_detector.sv \
    drum_selector.sv \
    button_debouncer.sv \
    2>&1 | tee compile.log

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Compilation failed. Check compile.log for details."
    exit 1
fi

echo "Compilation successful!"
echo ""
echo "Running simulation..."
echo ""

# Run simulation
vvp tb_top_sim 2>&1 | tee sim.log

# Check exit code
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Simulation completed successfully!"
    echo "=========================================="
else
    echo ""
    echo "=========================================="
    echo "Simulation had errors. Check sim.log"
    echo "=========================================="
fi

# Cleanup
rm -f tb_top_sim

