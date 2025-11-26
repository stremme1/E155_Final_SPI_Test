// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vtb_bno08x_spi_master.h for the primary calling header

#ifndef VERILATED_VTB_BNO08X_SPI_MASTER___024ROOT_H_
#define VERILATED_VTB_BNO08X_SPI_MASTER___024ROOT_H_  // guard

#include "verilated.h"
#include "verilated_timing.h"
class Vtb_bno08x_spi_master___024unit;


class Vtb_bno08x_spi_master__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vtb_bno08x_spi_master___024root final : public VerilatedModule {
  public:
    // CELLS
    Vtb_bno08x_spi_master___024unit* __PVT____024unit;

    // DESIGN SPECIFIC STATE
    CData/*0:0*/ tb_bno08x_spi_master__DOT__clk;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__rst_n;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__spi_cs_n;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__spi_sck;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__spi_mosi;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__spi_miso;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__transfer_done;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__transfer_busy;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__start_transfer;
    CData/*7:0*/ tb_bno08x_spi_master__DOT__tx_data;
    CData/*2:0*/ tb_bno08x_spi_master__DOT__dut__DOT__state;
    CData/*2:0*/ tb_bno08x_spi_master__DOT__dut__DOT__next_state;
    CData/*3:0*/ tb_bno08x_spi_master__DOT__dut__DOT__bit_counter;
    CData/*7:0*/ tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg;
    CData/*7:0*/ tb_bno08x_spi_master__DOT__dut__DOT__rx_shift_reg;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__dut__DOT__sck_phase;
    CData/*0:0*/ tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div;
    CData/*0:0*/ __Vdly__tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div;
    CData/*0:0*/ __Vdly__tb_bno08x_spi_master__DOT__spi_cs_n;
    CData/*3:0*/ __Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter;
    CData/*0:0*/ __VstlFirstIteration;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__rst_n__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_sck__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_cs_n__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_miso__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_mosi__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_busy__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_done__0;
    CData/*0:0*/ __Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__clk__0;
    CData/*0:0*/ __VactDidInit;
    CData/*0:0*/ __VactContinue;
    SData/*15:0*/ tb_bno08x_spi_master__DOT__data_length;
    SData/*15:0*/ tb_bno08x_spi_master__DOT__dut__DOT__byte_counter;
    IData/*31:0*/ tb_bno08x_spi_master__DOT__rx_index;
    IData/*31:0*/ __VactIterCount;
    VlUnpacked<CData/*7:0*/, 16> tb_bno08x_spi_master__DOT__tx_buffer;
    VlUnpacked<CData/*7:0*/, 16> tb_bno08x_spi_master__DOT__rx_buffer;
    VlUnpacked<CData/*7:0*/, 1> tb_bno08x_spi_master__DOT__unnamedblk1__DOT__test_data;
    VlUnpacked<CData/*7:0*/, 4> tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data;
    VlUnpacked<CData/*7:0*/, 4> tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1;
    VlDelayScheduler __VdlySched;
    VlTriggerScheduler __VtrigSched_hc35b8c84__0;
    VlTriggerScheduler __VtrigSched_ha629cbb0__0;
    VlTriggerVec<1> __VstlTriggered;
    VlTriggerVec<10> __VactTriggered;
    VlTriggerVec<10> __VnbaTriggered;

    // INTERNAL VARIABLES
    Vtb_bno08x_spi_master__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vtb_bno08x_spi_master___024root(Vtb_bno08x_spi_master__Syms* symsp, const char* v__name);
    ~Vtb_bno08x_spi_master___024root();
    VL_UNCOPYABLE(Vtb_bno08x_spi_master___024root);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
