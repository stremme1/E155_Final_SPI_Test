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
#include "STM32L4xx/Device/Include/stm32l432xx.h"  // Includes TIM_TypeDef, GPIO_TypeDef, TIM2, GPIOA, etc.

// All type definitions (TIM_TypeDef, GPIO_TypeDef) and macros (TIM2_BASE, GPIOA_BASE, TIM2, GPIOA)
// are now provided by stm32l432xx.h - no need to redefine them

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

// TIM6 Audio Interrupt Functions (for non-blocking multi-channel audio)
void TIM6_InitAudioInterrupt(void);  // Initialize TIM6 for 22.05kHz audio interrupts
void TIM6_StartAudioInterrupt(void);  // Start TIM6 audio interrupt timer
void TIM6_StopAudioInterrupt(void);   // Stop TIM6 audio interrupt timer

#endif
