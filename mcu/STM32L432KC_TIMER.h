// STM32L432KC_TIMER.h
// Timer library for STM32L432KC
//
// Author: Emmett Stralka
// Email: estralka@hmc.edu
// Date: 9/29/25
//
// Description: Timer library for PWM generation and timing functions

#ifndef STM32L4_TIMER_H
#define STM32L4_TIMER_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h" // Include STM32 definitions for TIM_TypeDef, GPIO_TypeDef, GPIOA, TIM2

// Timer definitions
#define __IO volatile

// Base addresses (only define if not already defined)
#ifndef TIM2_BASE
#define TIM2_BASE (0x40000000UL)
#endif
#ifndef GPIOA_BASE
#define GPIOA_BASE (0x48000000UL)
#endif

// TIM_TypeDef, TIM2, GPIO_TypeDef, and GPIOA are now defined in stm32l432xx.h

// Function prototypes
void TIM2_Init(void);
void TIM2_Start(void);
void TIM2_Stop(void);
void TIM2_SetFrequency(uint32_t frequency);
void ms_delay(int ms);
void TIM2_EnableGPIOClock(void);
void TIM2_ConfigurePA5(void);  // Configure PA5 for TIM2_CH1
void TIM2_InitAudio(void);  // Complete audio setup function
void TIM2_Silence(void);  // Set PWM duty cycle to 0% for silence

#endif
