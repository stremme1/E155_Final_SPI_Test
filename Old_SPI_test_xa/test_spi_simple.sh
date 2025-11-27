#!/bin/bash
# Simple test script to run SPI slave testbench

cd "/Users/emmettstralka/Desktop/E155_SPI_Final Project/Old_SPI_test_xa"

echo "Compiling testbench..."
iverilog -g2012 -o tb_drum_spi_slave_sim tb_drum_spi_slave.sv drum_spi_slave.sv 2>&1

if [ $? -eq 0 ]; then
    echo "Compilation successful!"
    echo "Running simulation (will timeout after 30 seconds)..."
    (vvp tb_drum_spi_slave_sim 2>&1 & PID=$!; sleep 30; kill $PID 2>/dev/null) || true
    echo "Simulation check complete"
else
    echo "Compilation failed!"
    exit 1
fi

