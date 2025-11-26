// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table internal header
//
// Internal details; most calling programs do not need this header,
// unless using verilator public meta comments.

#ifndef VERILATED_VTB_BNO08X_SPI_MASTER__SYMS_H_
#define VERILATED_VTB_BNO08X_SPI_MASTER__SYMS_H_  // guard

#include "verilated.h"

// INCLUDE MODEL CLASS

#include "Vtb_bno08x_spi_master.h"

// INCLUDE MODULE CLASSES
#include "Vtb_bno08x_spi_master___024root.h"
#include "Vtb_bno08x_spi_master___024unit.h"

// SYMS CLASS (contains all model state)
class alignas(VL_CACHE_LINE_BYTES)Vtb_bno08x_spi_master__Syms final : public VerilatedSyms {
  public:
    // INTERNAL STATE
    Vtb_bno08x_spi_master* const __Vm_modelp;
    VlDeleter __Vm_deleter;
    bool __Vm_didInit = false;

    // MODULE INSTANCE STATE
    Vtb_bno08x_spi_master___024root TOP;
    Vtb_bno08x_spi_master___024unit TOP____024unit;

    // CONSTRUCTORS
    Vtb_bno08x_spi_master__Syms(VerilatedContext* contextp, const char* namep, Vtb_bno08x_spi_master* modelp);
    ~Vtb_bno08x_spi_master__Syms();

    // METHODS
    const char* name() { return TOP.name(); }
};

#endif  // guard
