// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vtb_bno08x_spi_master.h for the primary calling header

#include "Vtb_bno08x_spi_master__pch.h"
#include "Vtb_bno08x_spi_master___024root.h"

VlCoroutine Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__0(Vtb_bno08x_spi_master___024root* vlSelf);
VlCoroutine Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__1(Vtb_bno08x_spi_master___024root* vlSelf);

void Vtb_bno08x_spi_master___024root___eval_initial(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_initial\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__0(vlSelf);
    Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__1(vlSelf);
}

VL_INLINE_OPT VlCoroutine Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__0(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__0\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.tb_bno08x_spi_master__DOT__clk = 0U;
    while (1U) {
        co_await vlSelfRef.__VdlySched.delay(0x28870ULL, 
                                             nullptr, 
                                             "tb_bno08x_spi_master.sv", 
                                             57);
        vlSelfRef.tb_bno08x_spi_master__DOT__clk = 
            (1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__clk)));
    }
}

VL_INLINE_OPT VlCoroutine Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__1(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_initial__TOP__Vtiming__1\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    IData/*31:0*/ tb_bno08x_spi_master__DOT__spi_write__Vstatic__i;
    tb_bno08x_spi_master__DOT__spi_write__Vstatic__i = 0;
    IData/*31:0*/ tb_bno08x_spi_master__DOT__spi_read__Vstatic__i;
    tb_bno08x_spi_master__DOT__spi_read__Vstatic__i = 0;
    IData/*31:0*/ __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__length;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__length = 0;
    VlQueue<CData/*7:0*/> __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__data;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__data.atDefault() = 0;
    IData/*31:0*/ __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__length;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__length = 0;
    VlQueue<CData/*7:0*/> __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__data;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__data.atDefault() = 0;
    IData/*31:0*/ __Vtask_tb_bno08x_spi_master__DOT__spi_read__3__length;
    __Vtask_tb_bno08x_spi_master__DOT__spi_read__3__length = 0;
    IData/*31:0*/ __Vtask_tb_bno08x_spi_master__DOT__spi_read__4__length;
    __Vtask_tb_bno08x_spi_master__DOT__spi_read__4__length = 0;
    IData/*31:0*/ __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__length;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__length = 0;
    VlQueue<CData/*7:0*/> __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__data;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__data.atDefault() = 0;
    IData/*31:0*/ __Vtask_tb_bno08x_spi_master__DOT__spi_read__6__length;
    __Vtask_tb_bno08x_spi_master__DOT__spi_read__6__length = 0;
    // Body
    VL_WRITEF_NX("=========================================\nBNO08X SPI Master Testbench\n=========================================\n",0);
    vlSelfRef.tb_bno08x_spi_master__DOT__rst_n = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__rx_index = 0U;
    co_await vlSelfRef.__VdlySched.delay(0x32cfd0ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         82);
    vlSelfRef.tb_bno08x_spi_master__DOT__rst_n = 1U;
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         84);
    VL_WRITEF_NX("\n--- Test 1: Single Byte Write ---\n",0);
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk1__DOT__test_data[0U] = 0xf9U;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__data 
        = VL_CVT_UNPACK_TO_Q(vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk1__DOT__test_data);
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__length = 1U;
    VL_WRITEF_NX("[%0t] Starting SPI Write: %0d bytes\n",0,
                 64,VL_TIME_UNITED_Q(1000),-9,32,__Vtask_tb_bno08x_spi_master__DOT__spi_write__1__length);
    tb_bno08x_spi_master__DOT__spi_write__Vstatic__i = 0U;
    while ((VL_LTS_III(32, tb_bno08x_spi_master__DOT__spi_write__Vstatic__i, __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__length) 
            & VL_GTS_III(32, 0x10U, tb_bno08x_spi_master__DOT__spi_write__Vstatic__i))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__tx_buffer[(0xfU 
                                                        & tb_bno08x_spi_master__DOT__spi_write__Vstatic__i)] 
            = __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__data.at(tb_bno08x_spi_master__DOT__spi_write__Vstatic__i);
        tb_bno08x_spi_master__DOT__spi_write__Vstatic__i 
            = ((IData)(1U) + tb_bno08x_spi_master__DOT__spi_write__Vstatic__i);
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length 
        = (0xffffU & __Vtask_tb_bno08x_spi_master__DOT__spi_write__1__length);
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 
        vlSelfRef.tb_bno08x_spi_master__DOT__tx_buffer
        [0U];
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         101);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 1U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         103);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    while ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done)))) {
        co_await vlSelfRef.__VtrigSched_ha629cbb0__0.trigger(1U, 
                                                             nullptr, 
                                                             "@( tb_bno08x_spi_master.transfer_done)", 
                                                             "tb_bno08x_spi_master.sv", 
                                                             107);
    }
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         108);
    VL_WRITEF_NX("[%0t] SPI Write Complete\n",0,64,
                 VL_TIME_UNITED_Q(1000),-9);
    co_await vlSelfRef.__VdlySched.delay(0x32cfd0ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         155);
    VL_WRITEF_NX("\n--- Test 2: Multi-Byte Write (SHTP Header) ---\n",0);
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data[0U] = 5U;
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data[1U] = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data[2U] = 2U;
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data[3U] = 1U;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__data 
        = VL_CVT_UNPACK_TO_Q(vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk2__DOT__test_data);
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__length = 4U;
    VL_WRITEF_NX("[%0t] Starting SPI Write: %0d bytes\n",0,
                 64,VL_TIME_UNITED_Q(1000),-9,32,__Vtask_tb_bno08x_spi_master__DOT__spi_write__2__length);
    tb_bno08x_spi_master__DOT__spi_write__Vstatic__i = 0U;
    while ((VL_LTS_III(32, tb_bno08x_spi_master__DOT__spi_write__Vstatic__i, __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__length) 
            & VL_GTS_III(32, 0x10U, tb_bno08x_spi_master__DOT__spi_write__Vstatic__i))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__tx_buffer[(0xfU 
                                                        & tb_bno08x_spi_master__DOT__spi_write__Vstatic__i)] 
            = __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__data.at(tb_bno08x_spi_master__DOT__spi_write__Vstatic__i);
        tb_bno08x_spi_master__DOT__spi_write__Vstatic__i 
            = ((IData)(1U) + tb_bno08x_spi_master__DOT__spi_write__Vstatic__i);
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length 
        = (0xffffU & __Vtask_tb_bno08x_spi_master__DOT__spi_write__2__length);
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 
        vlSelfRef.tb_bno08x_spi_master__DOT__tx_buffer
        [0U];
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         101);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 1U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         103);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    while ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done)))) {
        co_await vlSelfRef.__VtrigSched_ha629cbb0__0.trigger(1U, 
                                                             nullptr, 
                                                             "@( tb_bno08x_spi_master.transfer_done)", 
                                                             "tb_bno08x_spi_master.sv", 
                                                             107);
    }
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         108);
    VL_WRITEF_NX("[%0t] SPI Write Complete\n",0,64,
                 VL_TIME_UNITED_Q(1000),-9);
    co_await vlSelfRef.__VdlySched.delay(0x32cfd0ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         167);
    VL_WRITEF_NX("\n--- Test 3: Single Byte Read ---\n",0);
    __Vtask_tb_bno08x_spi_master__DOT__spi_read__3__length = 1U;
    VL_WRITEF_NX("[%0t] Starting SPI Read: %0d bytes\n",0,
                 64,VL_TIME_UNITED_Q(1000),-9,32,__Vtask_tb_bno08x_spi_master__DOT__spi_read__3__length);
    tb_bno08x_spi_master__DOT__spi_read__Vstatic__i = 0U;
    while ((VL_LTS_III(32, tb_bno08x_spi_master__DOT__spi_read__Vstatic__i, __Vtask_tb_bno08x_spi_master__DOT__spi_read__3__length) 
            & VL_GTS_III(32, 0x10U, tb_bno08x_spi_master__DOT__spi_read__Vstatic__i))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__rx_buffer[(0xfU 
                                                        & tb_bno08x_spi_master__DOT__spi_read__Vstatic__i)] 
            = (0xffU & ((IData)(0xa0U) + tb_bno08x_spi_master__DOT__spi_read__Vstatic__i));
        tb_bno08x_spi_master__DOT__spi_read__Vstatic__i 
            = ((IData)(1U) + tb_bno08x_spi_master__DOT__spi_read__Vstatic__i);
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__rx_index = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length 
        = (0xffffU & __Vtask_tb_bno08x_spi_master__DOT__spi_read__3__length);
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 0U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         127);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 1U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         129);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    while ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done)))) {
        co_await vlSelfRef.__VtrigSched_ha629cbb0__0.trigger(1U, 
                                                             nullptr, 
                                                             "@( tb_bno08x_spi_master.transfer_done)", 
                                                             "tb_bno08x_spi_master.sv", 
                                                             133);
    }
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         134);
    VL_WRITEF_NX("[%0t] SPI Read Complete\n",0,64,VL_TIME_UNITED_Q(1000),
                 -9);
    co_await vlSelfRef.__VdlySched.delay(0x32cfd0ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         172);
    VL_WRITEF_NX("\n--- Test 4: Multi-Byte Read (SHTP Response) ---\n",0);
    __Vtask_tb_bno08x_spi_master__DOT__spi_read__4__length = 0x10U;
    VL_WRITEF_NX("[%0t] Starting SPI Read: %0d bytes\n",0,
                 64,VL_TIME_UNITED_Q(1000),-9,32,__Vtask_tb_bno08x_spi_master__DOT__spi_read__4__length);
    tb_bno08x_spi_master__DOT__spi_read__Vstatic__i = 0U;
    while ((VL_LTS_III(32, tb_bno08x_spi_master__DOT__spi_read__Vstatic__i, __Vtask_tb_bno08x_spi_master__DOT__spi_read__4__length) 
            & VL_GTS_III(32, 0x10U, tb_bno08x_spi_master__DOT__spi_read__Vstatic__i))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__rx_buffer[(0xfU 
                                                        & tb_bno08x_spi_master__DOT__spi_read__Vstatic__i)] 
            = (0xffU & ((IData)(0xa0U) + tb_bno08x_spi_master__DOT__spi_read__Vstatic__i));
        tb_bno08x_spi_master__DOT__spi_read__Vstatic__i 
            = ((IData)(1U) + tb_bno08x_spi_master__DOT__spi_read__Vstatic__i);
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__rx_index = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length 
        = (0xffffU & __Vtask_tb_bno08x_spi_master__DOT__spi_read__4__length);
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 0U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         127);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 1U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         129);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    while ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done)))) {
        co_await vlSelfRef.__VtrigSched_ha629cbb0__0.trigger(1U, 
                                                             nullptr, 
                                                             "@( tb_bno08x_spi_master.transfer_done)", 
                                                             "tb_bno08x_spi_master.sv", 
                                                             133);
    }
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         134);
    VL_WRITEF_NX("[%0t] SPI Read Complete\n",0,64,VL_TIME_UNITED_Q(1000),
                 -9);
    co_await vlSelfRef.__VdlySched.delay(0x32cfd0ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         177);
    VL_WRITEF_NX("\n--- Test 5: Back-to-Back Transfers ---\n",0);
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1[0U] = 5U;
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1[1U] = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1[2U] = 2U;
    vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1[3U] = 2U;
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__data 
        = VL_CVT_UNPACK_TO_Q(vlSelfRef.tb_bno08x_spi_master__DOT__unnamedblk3__DOT__test_data1);
    __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__length = 4U;
    VL_WRITEF_NX("[%0t] Starting SPI Write: %0d bytes\n",0,
                 64,VL_TIME_UNITED_Q(1000),-9,32,__Vtask_tb_bno08x_spi_master__DOT__spi_write__5__length);
    tb_bno08x_spi_master__DOT__spi_write__Vstatic__i = 0U;
    while ((VL_LTS_III(32, tb_bno08x_spi_master__DOT__spi_write__Vstatic__i, __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__length) 
            & VL_GTS_III(32, 0x10U, tb_bno08x_spi_master__DOT__spi_write__Vstatic__i))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__tx_buffer[(0xfU 
                                                        & tb_bno08x_spi_master__DOT__spi_write__Vstatic__i)] 
            = __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__data.at(tb_bno08x_spi_master__DOT__spi_write__Vstatic__i);
        tb_bno08x_spi_master__DOT__spi_write__Vstatic__i 
            = ((IData)(1U) + tb_bno08x_spi_master__DOT__spi_write__Vstatic__i);
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length 
        = (0xffffU & __Vtask_tb_bno08x_spi_master__DOT__spi_write__5__length);
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 
        vlSelfRef.tb_bno08x_spi_master__DOT__tx_buffer
        [0U];
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         101);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 1U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         103);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    while ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done)))) {
        co_await vlSelfRef.__VtrigSched_ha629cbb0__0.trigger(1U, 
                                                             nullptr, 
                                                             "@( tb_bno08x_spi_master.transfer_done)", 
                                                             "tb_bno08x_spi_master.sv", 
                                                             107);
    }
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         108);
    VL_WRITEF_NX("[%0t] SPI Write Complete\n",0,64,
                 VL_TIME_UNITED_Q(1000),-9);
    co_await vlSelfRef.__VdlySched.delay(0xa2990ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         189);
    __Vtask_tb_bno08x_spi_master__DOT__spi_read__6__length = 4U;
    VL_WRITEF_NX("[%0t] Starting SPI Read: %0d bytes\n",0,
                 64,VL_TIME_UNITED_Q(1000),-9,32,__Vtask_tb_bno08x_spi_master__DOT__spi_read__6__length);
    tb_bno08x_spi_master__DOT__spi_read__Vstatic__i = 0U;
    while ((VL_LTS_III(32, tb_bno08x_spi_master__DOT__spi_read__Vstatic__i, __Vtask_tb_bno08x_spi_master__DOT__spi_read__6__length) 
            & VL_GTS_III(32, 0x10U, tb_bno08x_spi_master__DOT__spi_read__Vstatic__i))) {
        vlSelfRef.tb_bno08x_spi_master__DOT__rx_buffer[(0xfU 
                                                        & tb_bno08x_spi_master__DOT__spi_read__Vstatic__i)] 
            = (0xffU & ((IData)(0xa0U) + tb_bno08x_spi_master__DOT__spi_read__Vstatic__i));
        tb_bno08x_spi_master__DOT__spi_read__Vstatic__i 
            = ((IData)(1U) + tb_bno08x_spi_master__DOT__spi_read__Vstatic__i);
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__rx_index = 0U;
    vlSelfRef.tb_bno08x_spi_master__DOT__data_length 
        = (0xffffU & __Vtask_tb_bno08x_spi_master__DOT__spi_read__6__length);
    vlSelfRef.tb_bno08x_spi_master__DOT__tx_data = 0U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         127);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 1U;
    co_await vlSelfRef.__VtrigSched_hc35b8c84__0.trigger(0U, 
                                                         nullptr, 
                                                         "@(posedge tb_bno08x_spi_master.clk)", 
                                                         "tb_bno08x_spi_master.sv", 
                                                         129);
    vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer = 0U;
    while ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done)))) {
        co_await vlSelfRef.__VtrigSched_ha629cbb0__0.trigger(1U, 
                                                             nullptr, 
                                                             "@( tb_bno08x_spi_master.transfer_done)", 
                                                             "tb_bno08x_spi_master.sv", 
                                                             133);
    }
    co_await vlSelfRef.__VdlySched.delay(0x1967e8ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         134);
    VL_WRITEF_NX("[%0t] SPI Read Complete\n",0,64,VL_TIME_UNITED_Q(1000),
                 -9);
    co_await vlSelfRef.__VdlySched.delay(0x32cfd0ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         191);
    VL_WRITEF_NX("\n=========================================\nAll Tests Complete\n=========================================\n",0);
    co_await vlSelfRef.__VdlySched.delay(0x1fc1e20ULL, 
                                         nullptr, "tb_bno08x_spi_master.sv", 
                                         196);
    VL_FINISH_MT("tb_bno08x_spi_master.sv", 197, "");
}

void Vtb_bno08x_spi_master___024root___act_comb__TOP__0(Vtb_bno08x_spi_master___024root* vlSelf);

void Vtb_bno08x_spi_master___024root___eval_act(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_act\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((0x380ULL & vlSelfRef.__VactTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___act_comb__TOP__0(vlSelf);
    }
}

VL_INLINE_OPT void Vtb_bno08x_spi_master___024root___act_comb__TOP__0(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___act_comb__TOP__0\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
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

void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__0(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__1(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__2(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__3(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__4(Vtb_bno08x_spi_master___024root* vlSelf);

void Vtb_bno08x_spi_master___024root___eval_nba(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_nba\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((0x101ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___nba_sequent__TOP__0(vlSelf);
    }
    if ((0xfcULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___nba_sequent__TOP__1(vlSelf);
    }
    if ((0x101ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___nba_sequent__TOP__2(vlSelf);
    }
    if ((3ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___nba_sequent__TOP__3(vlSelf);
    }
    if ((0x381ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___act_comb__TOP__0(vlSelf);
    }
    if ((0x101ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vtb_bno08x_spi_master___024root___nba_sequent__TOP__4(vlSelf);
    }
}

VL_INLINE_OPT void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__0(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___nba_sequent__TOP__0\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div 
        = vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div;
    vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n 
        = vlSelfRef.tb_bno08x_spi_master__DOT__spi_cs_n;
    vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter 
        = vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter;
    vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div 
        = ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__rst_n) 
           && (1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div))));
}

VL_INLINE_OPT void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__2(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___nba_sequent__TOP__2\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if (vlSelfRef.tb_bno08x_spi_master__DOT__rst_n) {
        if ((1U & (~ ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state) 
                      >> 2U)))) {
            if ((2U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
                if ((1U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
                    vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n = 1U;
                } else if (vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div) {
                    if (vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__sck_phase) {
                        if ((8U > (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter))) {
                            vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__rx_shift_reg 
                                = ((0xfeU & ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__rx_shift_reg) 
                                             << 1U)) 
                                   | (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_miso));
                            vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter 
                                = (0xfU & ((IData)(1U) 
                                           + (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter)));
                        }
                        vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck = 1U;
                        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__sck_phase = 0U;
                        if ((7U == (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter))) {
                            vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter = 0U;
                            vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__byte_counter 
                                = (0xffffU & ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__byte_counter) 
                                              - (IData)(1U)));
                        }
                    } else {
                        vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck = 0U;
                        if ((8U > (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter))) {
                            vlSelfRef.tb_bno08x_spi_master__DOT__spi_mosi 
                                = (1U & ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg) 
                                         >> 7U));
                            vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg 
                                = (0xfeU & ((IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg) 
                                            << 1U));
                        }
                        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__sck_phase = 1U;
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state))) {
                vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n = 0U;
            } else {
                if (vlSelfRef.tb_bno08x_spi_master__DOT__start_transfer) {
                    vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__byte_counter 
                        = vlSelfRef.tb_bno08x_spi_master__DOT__data_length;
                    vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter = 0U;
                    vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg 
                        = vlSelfRef.tb_bno08x_spi_master__DOT__tx_data;
                }
                vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n = 1U;
                vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck = 1U;
                vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__sck_phase = 0U;
            }
        }
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state 
            = vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__next_state;
    } else {
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__byte_counter = 0U;
        vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter = 0U;
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__tx_shift_reg = 0U;
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__rx_shift_reg = 0U;
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__sck_phase = 0U;
        vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n = 1U;
        vlSelfRef.tb_bno08x_spi_master__DOT__spi_sck = 1U;
        vlSelfRef.tb_bno08x_spi_master__DOT__spi_mosi = 0U;
        vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state = 0U;
    }
    vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div 
        = vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__spi_clk_div;
    vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__bit_counter 
        = vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__dut__DOT__bit_counter;
    vlSelfRef.tb_bno08x_spi_master__DOT__transfer_done 
        = (4U == (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state));
    vlSelfRef.tb_bno08x_spi_master__DOT__transfer_busy 
        = ((0U != (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state)) 
           & (4U != (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__dut__DOT__state)));
}

VL_INLINE_OPT void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__3(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___nba_sequent__TOP__3\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if (vlSelfRef.tb_bno08x_spi_master__DOT__rst_n) {
        if ((1U & (~ (IData)(vlSelfRef.tb_bno08x_spi_master__DOT__spi_cs_n)))) {
            if (VL_GTS_III(32, 0x10U, vlSelfRef.tb_bno08x_spi_master__DOT__rx_index)) {
                vlSelfRef.tb_bno08x_spi_master__DOT__spi_miso 
                    = (1U & (vlSelfRef.tb_bno08x_spi_master__DOT__rx_buffer
                             [(0xfU & vlSelfRef.tb_bno08x_spi_master__DOT__rx_index)] 
                             >> (7U & ((IData)(7U) 
                                       - VL_MODDIVS_III(32, vlSelfRef.tb_bno08x_spi_master__DOT__rx_index, (IData)(8U))))));
            }
        }
    } else {
        vlSelfRef.tb_bno08x_spi_master__DOT__spi_miso = 0U;
    }
}

VL_INLINE_OPT void Vtb_bno08x_spi_master___024root___nba_sequent__TOP__4(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___nba_sequent__TOP__4\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.tb_bno08x_spi_master__DOT__spi_cs_n = vlSelfRef.__Vdly__tb_bno08x_spi_master__DOT__spi_cs_n;
}

void Vtb_bno08x_spi_master___024root___timing_commit(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___timing_commit\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((! (0x100ULL & vlSelfRef.__VactTriggered.word(0U)))) {
        vlSelfRef.__VtrigSched_hc35b8c84__0.commit(
                                                   "@(posedge tb_bno08x_spi_master.clk)");
    }
    if ((! (0x80ULL & vlSelfRef.__VactTriggered.word(0U)))) {
        vlSelfRef.__VtrigSched_ha629cbb0__0.commit(
                                                   "@( tb_bno08x_spi_master.transfer_done)");
    }
}

void Vtb_bno08x_spi_master___024root___timing_resume(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___timing_resume\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((0x100ULL & vlSelfRef.__VactTriggered.word(0U))) {
        vlSelfRef.__VtrigSched_hc35b8c84__0.resume(
                                                   "@(posedge tb_bno08x_spi_master.clk)");
    }
    if ((0x80ULL & vlSelfRef.__VactTriggered.word(0U))) {
        vlSelfRef.__VtrigSched_ha629cbb0__0.resume(
                                                   "@( tb_bno08x_spi_master.transfer_done)");
    }
    if ((0x200ULL & vlSelfRef.__VactTriggered.word(0U))) {
        vlSelfRef.__VdlySched.resume();
    }
}

void Vtb_bno08x_spi_master___024root___eval_triggers__act(Vtb_bno08x_spi_master___024root* vlSelf);

bool Vtb_bno08x_spi_master___024root___eval_phase__act(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_phase__act\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    VlTriggerVec<10> __VpreTriggered;
    CData/*0:0*/ __VactExecute;
    // Body
    Vtb_bno08x_spi_master___024root___eval_triggers__act(vlSelf);
    Vtb_bno08x_spi_master___024root___timing_commit(vlSelf);
    __VactExecute = vlSelfRef.__VactTriggered.any();
    if (__VactExecute) {
        __VpreTriggered.andNot(vlSelfRef.__VactTriggered, vlSelfRef.__VnbaTriggered);
        vlSelfRef.__VnbaTriggered.thisOr(vlSelfRef.__VactTriggered);
        Vtb_bno08x_spi_master___024root___timing_resume(vlSelf);
        Vtb_bno08x_spi_master___024root___eval_act(vlSelf);
    }
    return (__VactExecute);
}

bool Vtb_bno08x_spi_master___024root___eval_phase__nba(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_phase__nba\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    CData/*0:0*/ __VnbaExecute;
    // Body
    __VnbaExecute = vlSelfRef.__VnbaTriggered.any();
    if (__VnbaExecute) {
        Vtb_bno08x_spi_master___024root___eval_nba(vlSelf);
        vlSelfRef.__VnbaTriggered.clear();
    }
    return (__VnbaExecute);
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__nba(Vtb_bno08x_spi_master___024root* vlSelf);
#endif  // VL_DEBUG
#ifdef VL_DEBUG
VL_ATTR_COLD void Vtb_bno08x_spi_master___024root___dump_triggers__act(Vtb_bno08x_spi_master___024root* vlSelf);
#endif  // VL_DEBUG

void Vtb_bno08x_spi_master___024root___eval(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    IData/*31:0*/ __VnbaIterCount;
    CData/*0:0*/ __VnbaContinue;
    // Body
    __VnbaIterCount = 0U;
    __VnbaContinue = 1U;
    while (__VnbaContinue) {
        if (VL_UNLIKELY(((0x64U < __VnbaIterCount)))) {
#ifdef VL_DEBUG
            Vtb_bno08x_spi_master___024root___dump_triggers__nba(vlSelf);
#endif
            VL_FATAL_MT("tb_bno08x_spi_master.sv", 9, "", "NBA region did not converge.");
        }
        __VnbaIterCount = ((IData)(1U) + __VnbaIterCount);
        __VnbaContinue = 0U;
        vlSelfRef.__VactIterCount = 0U;
        vlSelfRef.__VactContinue = 1U;
        while (vlSelfRef.__VactContinue) {
            if (VL_UNLIKELY(((0x64U < vlSelfRef.__VactIterCount)))) {
#ifdef VL_DEBUG
                Vtb_bno08x_spi_master___024root___dump_triggers__act(vlSelf);
#endif
                VL_FATAL_MT("tb_bno08x_spi_master.sv", 9, "", "Active region did not converge.");
            }
            vlSelfRef.__VactIterCount = ((IData)(1U) 
                                         + vlSelfRef.__VactIterCount);
            vlSelfRef.__VactContinue = 0U;
            if (Vtb_bno08x_spi_master___024root___eval_phase__act(vlSelf)) {
                vlSelfRef.__VactContinue = 1U;
            }
        }
        if (Vtb_bno08x_spi_master___024root___eval_phase__nba(vlSelf)) {
            __VnbaContinue = 1U;
        }
    }
}

#ifdef VL_DEBUG
void Vtb_bno08x_spi_master___024root___eval_debug_assertions(Vtb_bno08x_spi_master___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtb_bno08x_spi_master___024root___eval_debug_assertions\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}
#endif  // VL_DEBUG
