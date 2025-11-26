// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vtb_bno08x_spi_master.h for the primary calling header

#include "Vtb_bno08x_spi_master__pch.h"
#include "Vtb_bno08x_spi_master___024root.h"

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___eval_static(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_static\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
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
}

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___eval_final(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_final\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__stl(Vtb_bno08x_spi_master___024root* vlSelf);
#endif  // VL_DEBUG
VL_ATTR_COLD bool Vtb_bno08x_spi_master___024root___eval_phase__stl(Vtb_bno08x_spi_master___024root* vlSelf);

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___eval_settle(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_settle\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    IData/*31:0*/ __VstlIterCount;
    CData/*0:0*/ __VstlContinue;
    // Body
    __VstlIterCount = 0U;
    vlSelfRef.__VstlFirstIteration = 1U;
    __VstlContinue = 1U;
    while (__VstlContinue) {
        if (VL_UNLIKELY(((0x64U < __VstlIterCount)))) {
#ifdef VL_DEBUG
            Vtb_bno08x_spi_master___024root___dump_triggers__stl(vlSelf);
#endif
            VL_FATAL_MT("tb_bno08x_spi_master.sv", 9, "", "Settle region did not converge.");
        }
        __VstlIterCount = ((IData)(1U) + __VstlIterCount);
        __VstlContinue = 0U;
        if (Vtb_bno08x_spi_master___024root___eval_phase__stl(vlSelf)) {
            __VstlContinue = 1U;
        }
        vlSelfRef.__VstlFirstIteration = 0U;
    }
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__stl(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___dump_triggers__stl\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1U & (~ vlSelfRef.__VstlTriggered.any()))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelfRef.__VstlTriggered.word(0U))) {
        VL_DBG_MSGF("         'stl' region trigger index 0 is active: Internal 'stl' trigger - first iteration\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___stl_sequent__TOP__0(Vtb_bno08x_spi_master___024root* vlSelf);

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___eval_stl(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_stl\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1ULL & vlSelfRef.__VstlTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___stl_sequent__TOP__0(vlSelf);
    }
}

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___stl_sequent__TOP__0(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___stl_sequent__TOP__0\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done 
        = (4U == (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state));
    vlSelfRef.tb_bno08x_spi_master__DOT__transfer_busy 
        = ((0U != (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state)) 
           & (4U != (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state)));
    vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state 
        = vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state;
    if ((4U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state = 0U;
    } else if ((2U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
        if ((1U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
            vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state = 4U;
        } else if ((((0U == (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__byte_counter)) 
                     & (0U == (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter))) 
                    & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__sck_phase)))) {
            vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state = 3U;
        }
    } else if ((1U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state = 2U;
    } else if (((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer) 
                & (0U < (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__data_length)))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state = 1U;
    }
}

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___eval_triggers__stl(Vtb_bno08x_spi_master___024root* vlSelf);

VL_ATTR_COLD bool Vtb_bno08x_spi_master___024root___eval_phase__stl(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_phase__stl\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    CData/*0:0*/ __VstlExecute;
    // Body
    Vtb_bno08x_spi_master___024root___eval_triggers__stl(vlSelf);
    __VstlExecute = vlSelfRef.__VstlTriggered.any();
    if (__VstlExecute) {
        Vtb_bno08x_spi_master___024root___eval_stl(vlSelf);
    }
    return (__VstlExecute);
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__act(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___dump_triggers__act\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1U & (~ vlSelfRef.__VactTriggered.any()))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 0 is active: @(negedge tb_bno08x_spi_master.rst_n)\n");
    }
    if ((2ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 1 is active: @(negedge tb_bno08x_spi_master.spi_sck)\n");
    }
    if ((4ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 2 is active: @( tb_bno08x_spi_master.spi_cs_n)\n");
    }
    if ((8ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 3 is active: @( tb_bno08x_spi_master.spi_miso)\n");
    }
    if ((0x10ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 4 is active: @( tb_bno08x_spi_master.spi_mosi)\n");
    }
    if ((0x20ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 5 is active: @( tb_bno08x_spi_master.spi_sck)\n");
    }
    if ((0x40ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 6 is active: @( tb_bno08x_spi_master.transfer_busy)\n");
    }
    if ((0x80ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 7 is active: @( tb_bno08x_spi_master.transfer_done)\n");
    }
    if ((0x100ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 8 is active: @(posedge tb_bno08x_spi_master.clk)\n");
    }
    if ((0x200ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 9 is active: @([true] __VdlySched.awaitingCurrentTime())\n");
    }
}
#endif  // VL_DEBUG

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__nba(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___dump_triggers__nba\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1U & (~ vlSelfRef.__VnbaTriggered.any()))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 0 is active: @(negedge tb_bno08x_spi_master.rst_n)\n");
    }
    if ((2ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 1 is active: @(negedge tb_bno08x_spi_master.spi_sck)\n");
    }
    if ((4ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 2 is active: @( tb_bno08x_spi_master.spi_cs_n)\n");
    }
    if ((8ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 3 is active: @( tb_bno08x_spi_master.spi_miso)\n");
    }
    if ((0x10ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 4 is active: @( tb_bno08x_spi_master.spi_mosi)\n");
    }
    if ((0x20ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 5 is active: @( tb_bno08x_spi_master.spi_sck)\n");
    }
    if ((0x40ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 6 is active: @( tb_bno08x_spi_master.transfer_busy)\n");
    }
    if ((0x80ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 7 is active: @( tb_bno08x_spi_master.transfer_done)\n");
    }
    if ((0x100ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 8 is active: @(posedge tb_bno08x_spi_master.clk)\n");
    }
    if ((0x200ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 9 is active: @([true] __VdlySched.awaitingCurrentTime())\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___ctor_var_reset(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___ctor_var_reset\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    const uint64_t __VscopeHash = VL_MURMUR64_HASH(vlSelf->name());
    vlSelf->tb_bno08x_spi_master__DOT__clk = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 4010481309307262114ull);
    vlSelf->tb_bno08x_spi_master__DOT__rst_n = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 1573484335881152761ull);
    vlSelf->tb_bno08x_spi_master__DOT__spi_cs_n = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 12519779141885320282ull);
    vlSelf->tb_bno08x_spi_master__DOT__spi_sck = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 1550709571536485870ull);
    vlSelf->tb_bno08x_spi_master__DOT__spi_mosi = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 13770201945916277759ull);
    vlSelf->tb_bno08x_spi_master__DOT__spi_miso = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 17071898457725753403ull);
    vlSelf->tb_bno08x_spi_master__DOT__start_transfer = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 3328740562995195955ull);
    vlSelf->tb_bno08x_spi_master__DOT__data_length = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 18206670668529246911ull);
    vlSelf->tb_bno08x_spi_master__DOT__tx_data = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 9048693847789424246ull);
    vlSelf->tb_bno08x_spi_master__DOT__transfer_done = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5276537227669416064ull);
    vlSelf->tb_bno08x_spi_master__DOT__transfer_busy = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9782703448508448973ull);
    for (int __Vi0 = 0; __Vi0 < 16; ++__Vi0) {
        vlSelf->tb_bno08x_spi_master__DOT__tx_buffer[__Vi0] = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 7184149429489024548ull);
    }
    for (int __Vi0 = 0; __Vi0 < 16; ++__Vi0) {
        vlSelf->tb_bno08x_spi_master__DOT__rx_buffer[__Vi0] = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 9106397044026570728ull);
    }
    vlSelf->tb_bno08x_spi_master__DOT__rx_index = 0;
    for (int __Vi0 = 0; __Vi0 < 1; ++__Vi0) {
        vlSelf->tb_bno08x_spi_master__DOT__unnamedblk1__DOT__test_data[__Vi0] = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 7125961516946189305ull);
    }
    for (int __Vi0 = 0; __Vi0 < 4; ++__Vi0) {
        vlSelf->tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data[__Vi0] = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 15847772658174254484ull);
    }
    for (int __Vi0 = 0; __Vi0 < 4; ++__Vi0) {
        vlSelf->tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1[__Vi0] = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 14120117378788022855ull);
    }
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__state = VL_SCOPED_RAND_RESET_I(3, __VscopeHash, 1263654263276928690ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__next_state = VL_SCOPED_RAND_RESET_I(3, __VscopeHash, 5031803162120011467ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__byte_counter = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 6898888206229919894ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__bit_counter = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 15443669672723428716ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 335856384674576775ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__rx_shift_reg = VL_SCOPED_RAND_RESET_I(8, __VscopeHash, 15622537758362379636ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__sck_phase = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8955718109153781592ull);
    vlSelf->tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 3406948860842575975ull);
    vlSelf->__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9592611344557011048ull);
    vlSelf->__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 6747923428086856842ull);
    vlSelf->__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 1904064004904501917ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__rst_n__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8829452104957998828ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_sck__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9490812986286540086ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_cs_n__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5576735611271555835ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_miso__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 18367344684406820664ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__spi_mosi__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5609068843439336588ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_busy__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 17603696705868505932ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__transfer_done__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 6042144184011437279ull);
    vlSelf->__Vtrigprevexpr___TOP__tb_bno08x_spi_master__DOT__clk__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 16926957866965645887ull);
    vlSelf->__VactDidInit = 0;
}
