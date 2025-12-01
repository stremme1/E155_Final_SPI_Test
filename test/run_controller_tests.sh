#!/bin/bash

# Test runner for BNO085 controller tests
# Runs unit test and full system test with timeout protection

set -e

cd "$(dirname "$0")/.."
TEST_DIR="test/fpga"
INTEGRATION_DIR="test/integration"

echo "=========================================="
echo "BNO085 Controller Test Suite"
echo "=========================================="
echo ""

# Unit Test
echo "Running Unit Test..."
echo "----------------------------------------"
cd "$(dirname "$0")/.."
iverilog -g2012 -I fpga -I test/mocks -o $TEST_DIR/tb_bno085_controller_unit \
  $TEST_DIR/tb_bno085_controller_unit.sv \
  fpga/bno085_controller.sv \
  fpga/spi_master.sv \
  test/mocks/mock_bno085.sv 2>&1 | grep -v "warning:" || true

echo ""
echo "Executing unit test..."
vvp $TEST_DIR/tb_bno085_controller_unit 2>&1 | tee $TEST_DIR/unit_test_output.txt
echo ""

# Full System Test (if we have all dependencies)
if [ -f "fpga/drum_trigger_top.sv" ]; then
    echo "Running Full System Integration Test..."
    echo "----------------------------------------"
    iverilog -g2012 -I fpga -I test/mocks -o $INTEGRATION_DIR/tb_system_full \
      $INTEGRATION_DIR/tb_system_full.sv \
      fpga/drum_trigger_top.sv \
      fpga/bno085_controller.sv \
      fpga/spi_master.sv \
      fpga/spi_slave_mcu.sv \
      test/mocks/mock_bno085.sv 2>&1 | grep -v "warning:" || true
    
    echo ""
    echo "Executing system test..."
    vvp $INTEGRATION_DIR/tb_system_full 2>&1 | tee $INTEGRATION_DIR/system_test_output.txt
    echo ""
else
    echo "Skipping full system test (drum_trigger_top.sv not found)"
fi

echo "=========================================="
echo "Test Suite Complete"
echo "=========================================="

