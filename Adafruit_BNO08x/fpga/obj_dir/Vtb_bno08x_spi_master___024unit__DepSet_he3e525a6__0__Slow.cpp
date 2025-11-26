// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vtb_bno08x_spi_master.h for the primary calling header

#include "Vtb_bno08x_spi_master__pch.h"
#include "Vtb_bno08x_spi_master___024unit.h"

VL_ATTR_COLD void Vtb_bno08x_spi_master___024unit___ctor_var_reset(Vtb_bno08x_spi_master___024unit* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+  Vtb_bno08x_spi_master___024unit___ctor_var_reset\n"); );
    Vtb_bno08x_spi_master__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    const uint64_t __VscopeHash = VL_MURMUR64_HASH(vlSelf->name());
    vlSelf->__VmonitorOff = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9011783253012809090ull);
}
