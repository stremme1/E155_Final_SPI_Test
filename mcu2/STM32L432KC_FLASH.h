// STM32L432KC_FLASH.h
// Header for FLASH functions

#ifndef STM32L4_FLASH_H
#define STM32L4_FLASH_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h" // Include STM32 definitions for FLASH_TypeDef, FLASH, __IO

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

// Base addresses (only define if not already defined)
#ifndef FLASH_BASE
#define FLASH_BASE (0x40022000UL) // base address of FLASH
#endif

// FLASH_TypeDef and FLASH are now defined in stm32l432xx.h

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configureFlash(void);

#endif