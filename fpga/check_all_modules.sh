#!/bin/bash
# Comprehensive compilation check for all FPGA modules

echo "=========================================="
echo "FPGA Module Compilation Check"
echo "=========================================="
echo ""

ERRORS=0
PASSED=0

check_module() {
    local file=$1
    local name=$2
    echo -n "Checking $name... "
    
    if iverilog -g2012 -s $name $file -o /dev/null 2>&1 | grep -q "error"; then
        echo "❌ ERROR"
        iverilog -g2012 -s $name $file -o /dev/null 2>&1 | grep "error" | head -3
        ERRORS=$((ERRORS + 1))
    else
        echo "✅ OK"
        PASSED=$((PASSED + 1))
    fi
}

echo "1. Core Modules:"
check_module "button_debouncer.sv" "button_debouncer"
check_module "strike_detector.sv" "strike_detector"
check_module "drum_zone_detector.sv" "drum_zone_detector"
check_module "drum_selector.sv" "drum_selector"

echo ""
echo "2. Processing Modules:"
check_module "yaw_normalizer.sv" "yaw_normalizer"
check_module "quaternion_to_euler_dsp.sv" "quaternion_to_euler_dsp"

echo ""
echo "3. SPI Modules:"
check_module "spi_master.sv" "spi_master"
check_module "drum_spi_slave.sv" "drum_spi_slave"

echo ""
echo "4. Complex Modules:"
check_module "bno085_controller.sv" "bno085_controller"
check_module "drum_trigger_processor.sv" "drum_trigger_processor"

echo ""
echo "=========================================="
echo "Results: $PASSED passed, $ERRORS errors"
echo "=========================================="

if [ $ERRORS -eq 0 ]; then
    exit 0
else
    exit 1
fi
