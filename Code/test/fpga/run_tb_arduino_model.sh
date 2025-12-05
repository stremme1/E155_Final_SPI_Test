#!/bin/bash

# Run Arduino SPI Slave Testbench with Arduino Model
# Compiles and runs the testbench using exact Arduino SPI master model

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$SCRIPT_DIR"

echo "=========================================="
echo "Running Arduino SPI Slave Testbench with Arduino Model"
echo "=========================================="
echo ""

# Check if iverilog is available
if ! command -v iverilog &> /dev/null; then
    echo "[ERROR] iverilog not found. Please install Icarus Verilog."
    exit 1
fi

# Compile testbench
echo "Compiling testbench with Arduino model..."
if iverilog -g2012 -o tb_arduino_model.vvp \
    -I "$PROJECT_ROOT/fpga" \
    tb_arduino_spi_slave_with_arduino_model.sv \
    arduino_spi_master_model.sv \
    "$PROJECT_ROOT/fpga/arduino_spi_slave.sv" 2>&1; then
    echo "[OK] Compilation successful"
    echo ""
    
    # Run simulation
    echo "Running simulation..."
    echo "----------------------------------------"
    if vvp tb_arduino_model.vvp 2>&1; then
        echo ""
        echo "----------------------------------------"
        echo "[OK] Simulation completed"
        
        # Check if VCD file was created
        if [ -f "tb_arduino_spi_slave_with_arduino_model.vcd" ]; then
            echo "[OK] Waveform file created: tb_arduino_spi_slave_with_arduino_model.vcd"
            echo "    View with: gtkwave tb_arduino_spi_slave_with_arduino_model.vcd"
        fi
        
        echo ""
        echo "=========================================="
        echo "Testbench run complete"
        echo "=========================================="
    else
        echo ""
        echo "[ERROR] Simulation failed"
        rm -f tb_arduino_model.vvp
        exit 1
    fi
else
    echo "[ERROR] Compilation failed"
    exit 1
fi

