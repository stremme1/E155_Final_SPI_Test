#!/bin/bash
# Final testbench runner - ensures all tests pass

echo "=========================================="
echo "Full System Testbench - Final Run"
echo "=========================================="
echo ""

if ! command -v iverilog &> /dev/null; then
    echo "ERROR: iverilog not found."
    exit 1
fi

echo "Step 1: Compiling testbench..."
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
    2>&1 | tee compile_final.log

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ ERROR: Compilation failed. Check compile_final.log"
    exit 1
fi

echo "✅ Compilation successful!"
echo ""
echo "Step 2: Running simulation..."
echo "Note: This will take time due to 6M cycle reset delay (expected)"
echo ""

vvp tb_full_sim 2>&1 | tee sim_final.log

EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ Simulation completed!"
    echo "=========================================="
    echo ""
    echo "Test Results:"
    if grep -q "ALL TESTS PASSED" sim_final.log; then
        echo "✅ ALL TESTS PASSED!"
        echo "✅ System is ready for FPGA synthesis!"
    elif grep -q "PASSED" sim_final.log; then
        PASSED=$(grep -c "PASSED" sim_final.log)
        echo "✅ Tests passed: $PASSED"
        if grep -q "failed" sim_final.log; then
            FAILED=$(grep -c "failed" sim_final.log)
            echo "❌ Tests failed: $FAILED"
        fi
    fi
    echo ""
    echo "Key outputs:"
    grep -E "(Test.*PASSED|Received drum|ALL TESTS)" sim_final.log | tail -10
else
    echo "❌ Simulation had errors"
    echo "=========================================="
fi

rm -f tb_full_sim
exit $EXIT_CODE
