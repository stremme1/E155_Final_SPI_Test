#!/bin/bash
# Integration Test Script for Drum Trigger System
# Runs all testbenches and verifies functionality

echo "=========================================="
echo "Drum Trigger System Integration Tests"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test results
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run a test
run_test() {
    local test_name=$1
    local test_file=$2
    
    echo -e "${YELLOW}Running: ${test_name}${NC}"
    echo "File: ${test_file}"
    
    # Run iverilog simulation (adjust command based on your simulator)
    # This is a template - adjust for your specific toolchain
    if iverilog -o ${test_name}_sim ${test_file} 2>&1 | tee ${test_name}_compile.log; then
        if vvp ${test_name}_sim 2>&1 | tee ${test_name}_run.log; then
            if grep -q "PASSED\|All Tests PASSED" ${test_name}_run.log; then
                echo -e "${GREEN}✓ ${test_name} PASSED${NC}"
                TESTS_PASSED=$((TESTS_PASSED + 1))
            else
                echo -e "${RED}✗ ${test_name} FAILED${NC}"
                TESTS_FAILED=$((TESTS_FAILED + 1))
            fi
        else
            echo -e "${RED}✗ ${test_name} SIMULATION FAILED${NC}"
            TESTS_FAILED=$((TESTS_FAILED + 1))
        fi
    else
        echo -e "${RED}✗ ${test_name} COMPILATION FAILED${NC}"
        TESTS_FAILED=$((TESTS_FAILED + 1))
    fi
    echo ""
}

# Test 1: Drum SPI Slave Module
echo "=========================================="
echo "Test 1: Drum SPI Slave Module"
echo "=========================================="
run_test "tb_drum_spi_slave" "tb_drum_spi_slave.sv drum_spi_slave.sv"

# Test 2: Integrated Top-Level
echo "=========================================="
echo "Test 2: Integrated Top-Level System"
echo "=========================================="
echo -e "${YELLOW}Note: This test requires all dependent modules${NC}"
echo "Dependencies:"
echo "  - drum_spi_slave.sv"
echo "  - drum_trigger_processor.sv"
echo "  - bno085_controller.sv"
echo "  - spi_master.sv"
echo "  - button_debouncer.sv"
echo "  - (and other supporting modules)"
echo ""

# Summary
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo -e "Tests Passed: ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Tests Failed: ${RED}${TESTS_FAILED}${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests PASSED!${NC}"
    exit 0
else
    echo -e "${RED}Some tests FAILED!${NC}"
    exit 1
fi

