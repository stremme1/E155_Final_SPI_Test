// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Model implementation (design independent parts)

#include "Vtb_bno08x_spi_master__pch.h"

//============================================================
// Constructors

Vtb_bno08x_spi_master::Vtb_bno08x_spi_master(VerilatedContext* _vcontextp__, const char* _vcname__)
    : VerilatedModel{*_vcontextp__}
    , vlSymsp{new Vtb_bno08x_spi_master__Syms(contextp(), _vcname__, this)}
    , __PVT____024unit{vlSymsp->TOP.__PVT____024unit}
    , rootp{&(vlSymsp->TOP)}
{
    // Register model with the context
    contextp()->addModel(this);
}

Vtb_bno08x_spi_master::Vtb_bno08x_spi_master(const char* _vcname__)
    : Vtb_bno08x_spi_master(Verilated::threadContextp(), _vcname__)
{
}

//============================================================
// Destructor

Vtb_bno08x_spi_master::~Vtb_bno08x_spi_master() {
    delete vlSymsp;
}

//============================================================
// Evaluation function

#ifdef VL_DEBUG
void Vtb_bno08x_spi_master___024root___eval_debug_assertions(Vtb_bno08x_spi_master___024root* vlSelf);
#endif  // VL_DEBUG
void Vtb_bno08x_spi_master___024root___eval_static(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___eval_initial(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___eval_settle(Vtb_bno08x_spi_master___024root* vlSelf);
void Vtb_bno08x_spi_master___024root___eval(Vtb_bno08x_spi_master___024root* vlSelf);

void Vtb_bno08x_spi_master::eval_step() {
    VL_DEBUG_IF(VL_DBG_MSGF("+++++TOP Evaluate Vtb_bno08x_spi_master::eval_step\n"); );
#ifdef VL_DEBUG
    // Debug assertions
    Vtb_bno08x_spi_master___024root___eval_debug_assertions(&(vlSymsp->TOP));
#endif  // VL_DEBUG
    vlSymsp->__Vm_deleter.deleteAll();
    if (VL_UNLIKELY(!vlSymsp->__Vm_didInit)) {
        vlSymsp->__Vm_didInit = true;
        VL_DEBUG_IF(VL_DBG_MSGF("+ Initial\n"););
        Vtb_bno08x_spi_master___024root___eval_static(&(vlSymsp->TOP));
        Vtb_bno08x_spi_master___024root___eval_initial(&(vlSymsp->TOP));
        Vtb_bno08x_spi_master___024root___eval_settle(&(vlSymsp->TOP));
    }
    VL_DEBUG_IF(VL_DBG_MSGF("+ Eval\n"););
    Vtb_bno08x_spi_master___024root___eval(&(vlSymsp->TOP));
    // Evaluate cleanup
    Verilated::endOfEval(vlSymsp->__Vm_evalMsgQp);
}

//============================================================
// Events and timing
bool Vtb_bno08x_spi_master::eventsPending() { return !vlSymsp->TOP.__VdlySched.empty(); }

uint64_t Vtb_bno08x_spi_master::nextTimeSlot() { return vlSymsp->TOP.__VdlySched.nextTimeSlot(); }

//============================================================
// Utilities

const char* Vtb_bno08x_spi_master::name() const {
    return vlSymsp->name();
}

//============================================================
// Invoke final blocks

void Vtb_bno08x_spi_master___024root___eval_final(Vtb_bno08x_spi_master___024root* vlSelf);

VL_ATTR_COLD void Vtb_bno08x_spi_master::final() {
    Vtb_bno08x_spi_master___024root___eval_final(&(vlSymsp->TOP));
}

//============================================================
// Implementations of abstract methods from VerilatedModel

const char* Vtb_bno08x_spi_master::hierName() const { return vlSymsp->name(); }
const char* Vtb_bno08x_spi_master::modelName() const { return "Vtb_bno08x_spi_master"; }
unsigned Vtb_bno08x_spi_master::threads() const { return 1; }
void Vtb_bno08x_spi_master::prepareClone() const { contextp()->prepareClone(); }
void Vtb_bno08x_spi_master::atClone() const {
    contextp()->threadPoolpOnClone();
}
