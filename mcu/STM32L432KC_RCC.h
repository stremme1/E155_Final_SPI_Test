// STM32L432KC_RCC.h
// Header for RCC functions

#ifndef STM32L4_RCC_H
#define STM32L4_RCC_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h" // Include STM32 definitions for RCC_TypeDef, RCC, __IO

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

// Base addresses (only define if not already defined)
#ifndef RCC_BASE
#define RCC_BASE (0x40021000UL) // base address of RCC
#endif

// PLL
#define PLLSRC_HSI 0
#define PLLSRC_HSE 1

// Clock configuration
#define SW_HSI  0
#define SW_HSE  1
#define SW_PLL  2

// RCC_TypeDef and RCC are now defined in stm32l432xx.h

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configurePLL(void);
void configureClock(void);

#endif