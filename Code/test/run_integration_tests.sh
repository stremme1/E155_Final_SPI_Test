#!/bin/bash

# Integration Tests Runner
# Runs FPGA-MCU integration tests, stopping on first failure

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TEST_DIR="$SCRIPT_DIR/integration"

cd "$TEST_DIR"

echo "=========================================="
echo "Integration Tests"
echo "=========================================="
echo ""

# Check if iverilog is available
if ! command -v iverilog &> /dev/null; then
    echo "[ERROR] iverilog not found. Please install Icarus Verilog."
    exit 1
fi

# Test 3.1: FPGA-MCU SPI Communication
echo "Test 3.1: FPGA-MCU SPI Communication (tb_fpga_mcu_spi.sv)"
echo "---------------------------------------------------"
if iverilog -g2012 -o tb_fpga_mcu_spi.vvp \
    tb_fpga_mcu_spi.sv \
    ../../fpga/spi_slave_mcu.sv \
    ../mocks/mock_mcu_spi_master.sv 2>&1; then
    if vvp tb_fpga_mcu_spi.vvp 2>&1; then
        echo "[PASS] Test 3.1: FPGA-MCU SPI Communication"
        rm -f tb_fpga_mcu_spi.vvp
    else
        echo "[FAIL] Test 3.1: FPGA-MCU SPI Communication - Simulation failed"
        rm -f tb_fpga_mcu_spi.vvp
        exit 1
    fi
else
    echo "[FAIL] Test 3.1: FPGA-MCU SPI Communication - Compilation failed"
    exit 1
fi
echo ""

# Test 3.2: System Integration
echo "Test 3.2: System Integration (tb_system_integration.sv)"
echo "---------------------------------------------------"
if iverilog -g2012 -o tb_system_integration.vvp \
    tb_system_integration.sv \
    ../../fpga/drum_trigger_top.sv \
    ../../fpga/bno085_controller.sv \
    ../../fpga/spi_master.sv \
    ../../fpga/spi_slave_mcu.sv \
    ../mocks/mock_bno085.sv \
    ../mocks/mock_mcu_spi_master.sv \
    ../mocks/hsosc_mock.sv 2>&1; then
    if vvp tb_system_integration.vvp 2>&1; then
        echo "[PASS] Test 3.2: System Integration"
        rm -f tb_system_integration.vvp
    else
        echo "[FAIL] Test 3.2: System Integration - Simulation failed"
        rm -f tb_system_integration.vvp
        exit 1
    fi
else
    echo "[FAIL] Test 3.2: System Integration - Compilation failed"
    exit 1
fi
echo ""

echo "=========================================="
echo "All Integration Tests PASSED"
echo "=========================================="

