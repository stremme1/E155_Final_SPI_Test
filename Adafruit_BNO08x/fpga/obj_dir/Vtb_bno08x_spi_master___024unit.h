// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vtb_bno08x_spi_master.h for the primary calling header

#ifndef VERILATED_VTB_BNO08X_SPI_MASTER___024UNIT_H_
#define VERILATED_VTB_BNO08X_SPI_MASTER___024UNIT_H_  // guard

#include "verilated.h"
#include "verilated_timing.h"


class Vtb_bno08x_spi_master__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vtb_bno08x_spi_master___024unit final : public VerilatedModule {
  public:

    // DESIGN SPECIFIC STATE
    CData/*0:0*/ __VmonitorOff;

    // INTERNAL VARIABLES
    Vtb_bno08x_spi_master__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vtb_bno08x_spi_master___024unit(Vtb_bno08x_spi_master__Syms* symsp, const char* v__name);
    ~Vtb_bno08x_spi_master___024unit();
    VL_UNCOPYABLE(Vtb_bno08x_spi_master___024unit);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
