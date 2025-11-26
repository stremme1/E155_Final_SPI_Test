#!/bin/bash

# Testbench Runner Script
# This script compiles and runs the testbenches

echo "========================================="
echo "BNO08X FPGA Testbench Runner"
echo "========================================="
echo ""

# Check for available simulators
if command -v verilator &> /dev/null; then
    echo "✓ Verilator found"
    VERILATOR_AVAILABLE=1
else
    echo "✗ Verilator not found"
    VERILATOR_AVAILABLE=0
fi

if command -v iverilog &> /dev/null; then
    echo "✓ Icarus Verilog found"
    IVERILOG_AVAILABLE=1
else
    echo "✗ Icarus Verilog not found"
    IVERILOG_AVAILABLE=0
fi

if command -v vlog &> /dev/null; then
    echo "✓ ModelSim/QuestaSim found"
    MODELSIM_AVAILABLE=1
else
    echo "✗ ModelSim/QuestaSim not found"
    MODELSIM_AVAILABLE=0
fi

echo ""
echo "========================================="
echo "Testbench Summary"
echo "========================================="
echo ""
echo "1. tb_bno08x_spi_master.sv"
echo "   Tests: SPI master interface"
echo "   - Single/multi-byte writes"
echo "   - Single/multi-byte reads"
echo "   - SPI timing verification"
echo ""
echo "2. tb_bno08x_controller.sv"
echo "   Tests: BNO08X controller"
echo "   - Initialization sequence"
echo "   - Product ID request/response"
echo "   - Sensor enable/configuration"
echo "   - Interrupt-driven data reading"
echo ""
echo "3. tb_sensor_processor.sv"
echo "   Tests: Sensor data processing"
echo "   - Quaternion parsing"
echo "   - Gyroscope parsing"
echo "   - Drum trigger logic"
echo "   - Yaw offset functionality"
echo ""
echo "4. tb_bno08x_drum_system.sv"
echo "   Tests: Complete system integration"
echo "   - Full initialization"
echo "   - End-to-end data flow"
echo "   - Drum trigger detection"
echo ""
echo "========================================="
echo "Compilation Check"
echo "========================================="
echo ""

# Try to compile with Verilator (syntax check only)
if [ $VERILATOR_AVAILABLE -eq 1 ]; then
    echo "Checking syntax with Verilator..."
    
    # Check SPI master
    echo -n "  tb_bno08x_spi_master.sv: "
    verilator --lint-only -Wno-WIDTH -Wno-CASEINCOMPLETE tb_bno08x_spi_master.sv bno08x_spi_master.sv 2>&1 | grep -q "Error" && echo "✗ Errors found" || echo "✓ Syntax OK"
    
    # Check controller
    echo -n "  tb_bno08x_controller.sv: "
    verilator --lint-only -Wno-WIDTH -Wno-CASEINCOMPLETE tb_bno08x_controller.sv bno08x_controller.sv bno08x_spi_master.sv 2>&1 | grep -q "Error" && echo "✗ Errors found" || echo "✓ Syntax OK"
    
    # Check sensor processor
    echo -n "  tb_sensor_processor.sv: "
    verilator --lint-only -Wno-WIDTH -Wno-CASEINCOMPLETE tb_sensor_processor.sv sensor_processor.sv 2>&1 | grep -q "Error" && echo "✗ Errors found" || echo "✓ Syntax OK"
    
    # Check drum system
    echo -n "  tb_bno08x_drum_system.sv: "
    verilator --lint-only -Wno-WIDTH -Wno-CASEINCOMPLETE tb_bno08x_drum_system.sv bno08x_drum_system.sv bno08x_controller.sv bno08x_spi_master.sv sensor_processor.sv 2>&1 | grep -q "Error" && echo "✗ Errors found" || echo "✓ Syntax OK"
fi

echo ""
echo "========================================="
echo "Note: Full simulation requires a simulator"
echo "with SystemVerilog and timing support."
echo ""
echo "Recommended tools:"
echo "  - ModelSim/QuestaSim (commercial)"
echo "  - VCS (commercial)"
echo "  - Xcelium (commercial)"
echo ""
echo "For open-source options, consider:"
echo "  - Verilator (needs C++ wrapper for timing)"
echo "  - Icarus Verilog (limited SV support)"
echo "========================================="

