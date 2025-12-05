#!/bin/bash

# Master Test Runner
# Runs all tests incrementally in order: MCU Unit → FPGA Unit → Integration
# Stops on first failure and reports status

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track test results
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run a test script and track results
run_test_phase() {
    local phase_name="$1"
    local script_path="$2"
    
    echo ""
    echo "=========================================="
    echo "Phase: $phase_name"
    echo "=========================================="
    
    if [ -f "$script_path" ]; then
        if bash "$script_path"; then
            echo -e "${GREEN}[PASS]${NC} $phase_name"
            ((TESTS_PASSED++))
            return 0
        else
            echo -e "${RED}[FAIL]${NC} $phase_name"
            ((TESTS_FAILED++))
            return 1
        fi
    else
        echo -e "${RED}[ERROR]${NC} Test script not found: $script_path"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up temporary files..."
    find "$SCRIPT_DIR" -name "*.vvp" -type f -delete 2>/dev/null || true
    find "$SCRIPT_DIR" -name "test_*" -type f -executable -delete 2>/dev/null || true
    find "$SCRIPT_DIR" -name "*.vcd" -type f -delete 2>/dev/null || true
}

# Trap to cleanup on exit
trap cleanup EXIT

echo "=========================================="
echo "Incremental Test Execution"
echo "=========================================="
echo "Starting test execution..."
echo ""

# Phase 1: MCU Unit Tests
if ! run_test_phase "MCU Unit Tests" "$SCRIPT_DIR/run_mcu_tests.sh"; then
    echo ""
    echo -e "${RED}=========================================="
    echo "TEST EXECUTION FAILED"
    echo "==========================================${NC}"
    echo "Failed at: MCU Unit Tests"
    echo "Total Passed: $TESTS_PASSED"
    echo "Total Failed: $TESTS_FAILED"
    exit 1
fi

# Phase 2: FPGA Unit Tests
if ! run_test_phase "FPGA Unit Tests" "$SCRIPT_DIR/run_fpga_tests.sh"; then
    echo ""
    echo -e "${RED}=========================================="
    echo "TEST EXECUTION FAILED"
    echo "==========================================${NC}"
    echo "Failed at: FPGA Unit Tests"
    echo "Total Passed: $TESTS_PASSED"
    echo "Total Failed: $TESTS_FAILED"
    exit 1
fi

# Phase 3: Integration Tests
if ! run_test_phase "Integration Tests" "$SCRIPT_DIR/run_integration_tests.sh"; then
    echo ""
    echo -e "${RED}=========================================="
    echo "TEST EXECUTION FAILED"
    echo "==========================================${NC}"
    echo "Failed at: Integration Tests"
    echo "Total Passed: $TESTS_PASSED"
    echo "Total Failed: $TESTS_FAILED"
    exit 1
fi

# All tests passed
echo ""
echo -e "${GREEN}=========================================="
echo "ALL TESTS PASSED"
echo "==========================================${NC}"
echo "Total Phases Passed: $TESTS_PASSED"
echo "Total Phases Failed: $TESTS_FAILED"
echo ""
echo "Code is ready for hardware deployment!"

exit 0

