# Testing Directory

This directory contains test files for the BNO085 FPGA-MCU drum trigger system.

## Directory Structure

```
test/
├── fpga/              # FPGA testbenches (SystemVerilog)
├── mcu/               # MCU unit tests (C code)
├── integration/       # FPGA-MCU integration tests
├── mocks/             # Mock models for simulation
└── hardware/          # Hardware test files
```

## Test Files

- **FPGA tests**: `fpga/tb_*.sv` - SystemVerilog testbenches
- **MCU tests**: `mcu/test_*.c` - C unit tests
- **Integration tests**: `integration/tb_*.sv` - Integration testbenches
- **Mocks**: `mocks/mock_*.sv` or `mocks/mock_*.c` - Simulation models

