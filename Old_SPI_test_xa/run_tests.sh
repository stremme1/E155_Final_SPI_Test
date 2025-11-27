#!/bin/bash

# Testbench Runner Script
# Runs all testbenches to verify drum trigger system logic

echo "=========================================="
echo "Drum Trigger System Test Suite"
echo "=========================================="
echo ""

# Check if iverilog is available
if ! command -v iverilog &> /dev/null; then
    echo "ERROR: iverilog not found. Please install iverilog."
    echo "On macOS: brew install icarus-verilog"
    echo "On Linux: sudo apt-get install iverilog"
    exit 1
fi

# Test 1: Strike Detector
echo "=== Test 1: Strike Detector ==="
iverilog -g2012 -o tb_strike_detector.vvp \
    tb_strike_detector.sv strike_detector.sv 2>&1 | grep -v "warning" || true
if [ -f "tb_strike_detector.vvp" ]; then
    vvp tb_strike_detector.vvp
    echo ""
fi

# Test 2: Zone Detector
echo "=== Test 2: Zone Detector ==="
iverilog -g2012 -o tb_drum_zone_detector.vvp \
    tb_drum_zone_detector.sv drum_zone_detector.sv 2>&1 | grep -v "warning" || true
if [ -f "tb_drum_zone_detector.vvp" ]; then
    vvp tb_drum_zone_detector.vvp
    echo ""
fi

# Test 3: Drum Selector
echo "=== Test 3: Drum Selector ==="
iverilog -g2012 -o tb_drum_selector.vvp \
    tb_drum_selector.sv drum_selector.sv 2>&1 | grep -v "warning" || true
if [ -f "tb_drum_selector.vvp" ]; then
    vvp tb_drum_selector.vvp
    echo ""
fi

# Test 4: Yaw Normalizer
echo "=== Test 4: Yaw Normalizer ==="
iverilog -g2012 -o tb_yaw_normalizer.vvp \
    tb_yaw_normalizer.sv yaw_normalizer.sv 2>&1 | grep -v "warning" || true
if [ -f "tb_yaw_normalizer.vvp" ]; then
    vvp tb_yaw_normalizer.vvp
    echo ""
fi

# Test 5: Complete System
echo "=== Test 5: Complete System ==="
iverilog -g2012 -o tb_drum_trigger_system.vvp \
    tb_drum_trigger_system.sv \
    drum_trigger_processor.sv \
    quaternion_to_euler_dsp.sv \
    yaw_normalizer.sv \
    drum_zone_detector.sv \
    strike_detector.sv \
    drum_selector.sv \
    button_debouncer.sv 2>&1 | grep -v "warning" || true
if [ -f "tb_drum_trigger_system.vvp" ]; then
    vvp tb_drum_trigger_system.vvp
    echo ""
fi

echo "=========================================="
echo "All Tests Complete"
echo "=========================================="

# Cleanup
rm -f *.vvp
