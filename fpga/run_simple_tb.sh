#!/bin/bash

# Simplified Testbench Runner - Tests MCU SPI and buttons only
# Avoids BNO085 multiple driver issues

echo "=========================================="
echo "Simplified Top-Level Testbench"
echo "Testing: MCU SPI + Button Functionality"
echo "=========================================="
echo ""

if ! command -v iverilog &> /dev/null; then
    echo "ERROR: iverilog not found."
    exit 1
fi

echo "Compiling testbench..."
iverilog -g2012 -o tb_simple_sim \
    hsosc_mock.sv \
    tb_top_mcu_spi_only.sv \
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
    2>&1 | tee compile_simple.log

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Compilation failed. Check compile_simple.log"
    exit 1
fi

echo "Compilation successful!"
echo ""
echo "Running simulation..."
echo ""

vvp tb_simple_sim 2>&1 | tee sim_simple.log

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ Simulation completed!"
    echo "=========================================="
    echo ""
    echo "Checking test results..."
    if grep -q "PASSED" sim_simple.log; then
        echo "✅ Tests found in log"
    fi
    if grep -q "ERROR" sim_simple.log; then
        echo "⚠️  Errors found in log"
    fi
else
    echo ""
    echo "=========================================="
    echo "❌ Simulation had errors"
    echo "=========================================="
fi

rm -f tb_simple_sim
