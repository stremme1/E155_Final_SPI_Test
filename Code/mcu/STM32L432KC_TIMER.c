// STM32L432KC_TIMER.c
// Timer library implementation for STM32L432KC
//
// Author: Emmett Stralka
// Email: estralka@hmc.edu
// Date: 9/29/25
//
// Description: Timer library implementation for PWM generation

#include "STM32L432KC_TIMER.h"
#include "STM32L432KC_RCC.h"

// Initialize TIM2 for PWM generation
void TIM2_Init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR1 |= (1 << 0);
    
    // Configure for 1MHz timer frequency (80MHz / 80)
    TIM2->PSC = 79;  // 80MHz / 80 = 1MHz
    
    // Set default values
    TIM2->ARR = 1000;  // Default 1kHz
    TIM2->CCR1 = 500;  // 50% duty cycle
    
    // Configure PWM mode 1
    TIM2->CCMR1 |= (0b110 << 4) | (1 << 3);  // PWM mode 1, preload enable
    TIM2->CCER |= (1 << 0);  // Enable channel 1
    TIM2->CR1 |= (1 << 7);   // Auto-reload preload enable
}

// Start timer
void TIM2_Start(void) {
    TIM2->CR1 |= (1 << 0);
}

// Stop timer
void TIM2_Stop(void) {
    TIM2->CR1 &= ~(1 << 0);
}

// Set frequency for PWM output
void TIM2_SetFrequency(uint32_t frequency) {
    if (frequency == 0) {
        TIM2_Silence();
        return;
    }
    
    // Re-enable the PWM output channel
    TIM2->CCER |= (1 << 0);  // Enable channel 1 output
    
    // Calculate timer period for desired frequency
    // Timer frequency = 1MHz, so period = 1,000,000 / frequency
    uint32_t period = 1000000 / frequency;
    
    // Update timer period and duty cycle
    TIM2->ARR = period;
    TIM2->CCR1 = period / 2;  // 50% duty cycle
    
    // Start timer
    TIM2_Start();
}

// Function for dummy delay by executing nops
// Calibrated for 80MHz system clock
void ms_delay(int ms) {
   while (ms-- > 0) {
      volatile int x=8000;  // 8,000 NOPs for 1ms at 80MHz
      while (x-- > 0)
         __asm("nop");
   }
}

// Enable GPIOA clock for TIM2_CH1 (PA5)
void TIM2_EnableGPIOClock(void) {
    RCC->AHB2ENR |= (1 << 0);  // Enable GPIOA clock
}

// Configure PA5 as alternate function for TIM2_CH1
void TIM2_ConfigurePA5(void) {
    // Set PA5 to alternate function mode
    GPIOA->MODER &= ~(0b11 << 10);  // Clear bits 11:10 (PA5)
    GPIOA->MODER |= (0b10 << 10);   // Set bits 11:10 to 10 (alternate function)
    
    // Set alternate function to AF1 (TIM2_CH1)
    // PA5 is pin 5, which uses AFR[0] (low register), bits 20-23 (5 * 4 = 20)
    GPIOA->AFR[0] &= ~(0b1111 << 20);  // Clear bits 23:20 (PA5)
    GPIOA->AFR[0] |= (0b0001 << 20);   // Set bits 23:20 to 0001 (AF1 = TIM2_CH1)
}


// Disable PWM output channel for complete silence
void TIM2_Silence(void) {
    // Stop timer first to avoid glitches
    TIM2_Stop();
    // Disable the PWM output channel
    TIM2->CCER &= ~(1 << 0);  // Disable channel 1 output
}

// Complete audio initialization function
void TIM2_InitAudio(void) {
    // Enable GPIOA clock
    TIM2_EnableGPIOClock();
    
    // Configure PA5 as alternate function for TIM2_CH1
    TIM2_ConfigurePA5();
    
    // Initialize TIM2
    TIM2_Init();
}

// ============================================================================
// TIM6 Audio Interrupt Functions
// ============================================================================
// TIM6 is used for audio sample rate timing (22.05kHz interrupts)
// This enables non-blocking audio playback with multi-channel mixing

// Initialize TIM6 for audio sample rate interrupts (22.05kHz)
// System clock: 80MHz, APB1 timer clock: 80MHz (PPRE1 = 1)
// Target frequency: 22,050 Hz
// Calculation: ARR = (80,000,000 / 22,050) - 1 ≈ 3628
void TIM6_InitAudioInterrupt(void) {
    // Enable TIM6 clock (APB1ENR1 bit 4)
    RCC->APB1ENR1 |= (1 << 4);
    
    // Small delay for clock stabilization
    volatile int delay = 10;
    while (delay-- > 0) {
        __asm("nop");
    }
    
    // Stop timer before configuration
    TIM6->CR1 &= ~(1 << 0);  // Clear CEN bit
    
    // Configure prescaler: PSC = 0 (no prescaling, timer runs at 80MHz)
    TIM6->PSC = 0;
    
    // Configure auto-reload register for 22.05kHz
    // ARR = (Timer_Clock / Target_Frequency) - 1
    // ARR = (80,000,000 / 22,050) - 1 = 3628.12... ≈ 3628
    TIM6->ARR = 3628;
    
    // Enable update interrupt (UIE bit in DIER register)
    TIM6->DIER |= (1 << 0);
    
    // Clear any pending interrupt
    TIM6->SR &= ~(1 << 0);  // Clear UIF bit
    
    // Enable TIM6 interrupt in NVIC
    // TIM6_DAC_IRQn = 54 (from stm32l432xx.h)
    // ISER[0] covers interrupts 0-31, ISER[1] covers 32-63
    // So interrupt 54 is in ISER[1], bit (54 - 32) = bit 22
    NVIC->ISER[1] |= (1 << 22);  // Enable TIM6_DAC interrupt
    
    // Set interrupt priority (lower number = higher priority)
    // Use priority 2 (moderate priority, allows other interrupts)
    // Each interrupt has 4 bits for priority (bits 7:4 of IPR register)
    NVIC->IPR[54] = (2 << 4);  // Priority 2 (bits 7:4)
}

// Start TIM6 audio interrupt timer
void TIM6_StartAudioInterrupt(void) {
    // Enable counter (CEN bit)
    TIM6->CR1 |= (1 << 0);
}

// Stop TIM6 audio interrupt timer
void TIM6_StopAudioInterrupt(void) {
    // Disable counter (CEN bit)
    TIM6->CR1 &= ~(1 << 0);
}
