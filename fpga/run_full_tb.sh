#!/bin/bash

# Full Testbench Runner for tb_drum_trigger_top_integrated.sv

echo "=========================================="
echo "Full System Testbench"
echo "Testing: Complete drum trigger system"
echo "=========================================="
echo ""

if ! command -v iverilog &> /dev/null; then
    echo "ERROR: iverilog not found."
    exit 1
fi

echo "Compiling testbench..."
iverilog -g2012 -o tb_full_sim \
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
    2>&1 | tee compile_full.log

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Compilation failed. Check compile_full.log"
    exit 1
fi

echo "Compilation successful!"
echo ""
echo "Running simulation..."
echo ""

vvp tb_full_sim 2>&1 | tee sim_full.log

EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ Simulation completed!"
    echo "=========================================="
    echo ""
    echo "Test Results Summary:"
    if grep -q "PASSED" sim_full.log; then
        echo "✅ Tests passed:"
        grep "PASSED" sim_full.log | head -5
    fi
    if grep -q "ERROR" sim_full.log; then
        echo ""
        echo "⚠️  Errors found:"
        grep "ERROR" sim_full.log | head -5
    fi
    if grep -q "failed" sim_full.log; then
        echo ""
        echo "❌ Test failures:"
        grep "failed" sim_full.log | head -5
    fi
else
    echo "❌ Simulation had errors"
    echo "=========================================="
fi

rm -f tb_full_sim
exit $EXIT_CODE
