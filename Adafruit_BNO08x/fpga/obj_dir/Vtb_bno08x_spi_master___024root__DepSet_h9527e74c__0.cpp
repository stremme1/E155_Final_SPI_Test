// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vtb_bno08x_spi_master.h for the primary calling header

#include "Vtb_bno08x_spi_master__pch.h"
#include "Vtb_bno08x_spi_master__Syms.h"
#include "Vtb_bno08x_spi_master___024root.h"

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__act(Vtb_bno08x_spi_master___024root* vlSelf);
#endif  // VL_DEBUG

void Vtb_bno08x_spi_master___024root___eval_triggers__act(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_triggers__act\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__VactTriggered.setBit(0U, ((~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__rst_n)) 
                                          & (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__rst_n__0)));
    vlSelfRef.__VactTriggered.setBit(1U, ((~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck)) 
                                          & (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_sck__0)));
    vlSelfRef.__VactTriggered.setBit(2U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_cs_n) 
                                          != (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_cs_n__0)));
    vlSelfRef.__VactTriggered.setBit(3U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_miso) 
                                          != (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_miso__0)));
    vlSelfRef.__VactTriggered.setBit(4U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_mosi) 
                                          != (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_mosi__0)));
    vlSelfRef.__VactTriggered.setBit(5U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck) 
                                          != (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_sck__0)));
    vlSelfRef.__VactTriggered.setBit(6U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_busy) 
                                          != (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_busy__0)));
    vlSelfRef.__VactTriggered.setBit(7U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done) 
                                          != (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_done__0)));
    vlSelfRef.__VactTriggered.setBit(8U, ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__clk) 
                                          & (~ (IData)(vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__clk__0))));
    vlSelfRef.__VactTriggered.setBit(9U, vlSelfRef.__VdlySched.awaitingCurrentTime());
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__rst_n__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__rst_n;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_sck__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_cs_n__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__spi_cs_n;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_miso__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__spi_miso;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_mosi__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__spi_mosi;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_busy__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__transfer_busy;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_done__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done;
    vlSelfRef.__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__clk__0 
        = vlSelfRef.tb_bno08x_spi_master__DOT__clk;
    if (VL_UNLIKELY(((1U & (~ (IData)(vlSelfRef.__VactDidInit)))))) {
        vlSelfRef.__VactDidInit = 1U;
        vlSelfRef.__VactTriggered.setBit(2U, 1U);
        vlSelfRef.__VactTriggered.setBit(3U, 1U);
        vlSelfRef.__VactTriggered.setBit(4U, 1U);
        vlSelfRef.__VactTriggered.setBit(5U, 1U);
        vlSelfRef.__VactTriggered.setBit(6U, 1U);
        vlSelfRef.__VactTriggered.setBit(7U, 1U);
    }
#ifdef VL_DEBUG
    if (VL_UNLIKELY(vlSymsp->_vm_contextp__->debug())) {
        Vtb_bno08x_spi_master___024root___dump_triggers__act(vlSelf);
    }
#endif
}

VL_INLINE_OPT void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__1(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___nba_sequent__TOP__1\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if (VL_UNLIKELY(((1U & (~ (IData)(vlSymsp->TOP____024unit.__VmonitorOff)))))) {
        VL_WRITEF_NX("[%0t] CS=%b SCK=%b MOSI=%b MISO=%b BUSY=%b DONE=%b\n",0,
                     64,VL_TIME_UNITED_Q(1000),-9,1,
                     (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_cs_n),
                     1,vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck,
                     1,(IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_mosi),
                     1,vlSelfRef.tb_bno08x_spi_master__DOT__spi_miso,
                     1,(IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_busy),
                     1,vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done);
    }
}
