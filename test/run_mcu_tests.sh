#!/bin/bash

# MCU Unit Tests Runner
# Runs all MCU unit tests in order, stopping on first failure

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TEST_DIR="$SCRIPT_DIR/mcu"

cd "$TEST_DIR"

echo "=========================================="
echo "MCU Unit Tests"
echo "=========================================="
echo ""

# Test 1.1: Quaternion Decoder
echo "Test 1.1: Quaternion Decoder (test_bno085_decoder.c)"
echo "---------------------------------------------------"
if gcc test_bno085_decoder.c ../../mcu/bno085_decoder.c mock_debug_print.c -I../../mcu -I../../mcu/CMSIS_5/CMSIS/Core/Include -o test_decoder -lm; then
    if ./test_decoder; then
        echo "[PASS] Test 1.1: Quaternion Decoder"
        rm -f test_decoder
    else
        echo "[FAIL] Test 1.1: Quaternion Decoder - Test execution failed"
        rm -f test_decoder
        exit 1
    fi
else
    echo "[FAIL] Test 1.1: Quaternion Decoder - Compilation failed"
    exit 1
fi
echo ""

# Test 1.2: Drum Detector
echo "Test 1.2: Drum Detector (test_drum_detector.c)"
echo "---------------------------------------------------"
if gcc test_drum_detector.c ../../mcu/drum_detector.c ../../mcu/bno085_decoder.c mock_debug_print.c -I../../mcu -I../../mcu/CMSIS_5/CMSIS/Core/Include -o test_detector -lm; then
    if ./test_detector; then
        echo "[PASS] Test 1.2: Drum Detector"
        rm -f test_detector
    else
        echo "[FAIL] Test 1.2: Drum Detector - Test execution failed"
        rm -f test_detector
        exit 1
    fi
else
    echo "[FAIL] Test 1.2: Drum Detector - Compilation failed"
    exit 1
fi
echo ""

# Test 1.3: SPI Parser
echo "Test 1.3: SPI Parser (test_spi_parser.c)"
echo "---------------------------------------------------"
if gcc test_spi_parser.c -o test_parser; then
    if ./test_parser; then
        echo "[PASS] Test 1.3: SPI Parser"
        rm -f test_parser
    else
        echo "[FAIL] Test 1.3: SPI Parser - Test execution failed"
        rm -f test_parser
        exit 1
    fi
else
    echo "[FAIL] Test 1.3: SPI Parser - Compilation failed"
    exit 1
fi
echo ""

# Test 1.4: SPI Communication
echo "Test 1.4: SPI Communication (test_spi_communication.c)"
echo "---------------------------------------------------"
if gcc test_spi_communication.c -o test_spi_comm; then
    if ./test_spi_comm; then
        echo "[PASS] Test 1.4: SPI Communication"
        rm -f test_spi_comm
    else
        echo "[FAIL] Test 1.4: SPI Communication - Test execution failed"
        rm -f test_spi_comm
        exit 1
    fi
else
    echo "[FAIL] Test 1.4: SPI Communication - Compilation failed"
    exit 1
fi
echo ""

# Test 1.5: Arduino Packet Parser (Nuk_Option)
echo "Test 1.5: Arduino Packet Parser (test_arduino_packet_parser.c)"
echo "---------------------------------------------------"
if gcc test_arduino_packet_parser.c -o test_arduino_parser -lm; then
    if ./test_arduino_parser; then
        echo "[PASS] Test 1.5: Arduino Packet Parser"
        rm -f test_arduino_parser
    else
        echo "[FAIL] Test 1.5: Arduino Packet Parser - Test execution failed"
        rm -f test_arduino_parser
        exit 1
    fi
else
    echo "[FAIL] Test 1.5: Arduino Packet Parser - Compilation failed"
    exit 1
fi
echo ""

echo "=========================================="
echo "All MCU Unit Tests PASSED"
echo "=========================================="

