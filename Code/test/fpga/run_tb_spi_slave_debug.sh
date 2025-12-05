#!/bin/bash

# Run SPI Slave Debug Testbench
# Compiles and runs the testbench to verify shift register behavior

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$SCRIPT_DIR"

echo "=========================================="
echo "Running SPI Slave Debug Testbench"
echo "=========================================="
echo ""

# Check if iverilog is available
if ! command -v iverilog &> /dev/null; then
    echo "[ERROR] iverilog not found. Please install Icarus Verilog."
    exit 1
fi

# Compile testbench
echo "Compiling testbench..."
if iverilog -g2012 -o tb_spi_slave_mcu_debug.vvp \
    tb_spi_slave_mcu_debug.sv \
    ../../fpga/spi_slave_mcu.sv 2>&1; then
    echo "[OK] Compilation successful"
    echo ""
    
    # Run simulation
    echo "Running simulation..."
    echo "----------------------------------------"
    if vvp tb_spi_slave_mcu_debug.vvp 2>&1; then
        echo ""
        echo "----------------------------------------"
        echo "[OK] Simulation completed"
        
        # Check if VCD file was created
        if [ -f "tb_spi_slave_mcu_debug.vcd" ]; then
            echo "[OK] Waveform file created: tb_spi_slave_mcu_debug.vcd"
        fi
        
        # Clean up
        rm -f tb_spi_slave_mcu_debug.vvp
        echo ""
        echo "=========================================="
        echo "Testbench run complete"
        echo "=========================================="
    else
        echo ""
        echo "[ERROR] Simulation failed"
        rm -f tb_spi_slave_mcu_debug.vvp
        exit 1
    fi
else
    echo "[ERROR] Compilation failed"
    exit 1
fi

