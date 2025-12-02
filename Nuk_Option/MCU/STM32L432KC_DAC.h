// STM32L432KC_DAC.h
// DAC library for STM32L432KC
//
// Author: Emmett Stralka
// Email: estralka@hmc.edu
// Date: 9/29/25
//
// Description: DAC library for audio output generation

#ifndef STM32L4_DAC_H
#define STM32L4_DAC_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h"  // Includes DAC_TypeDef, DAC, GPIO_TypeDef, GPIOA, etc.

// All type definitions (DAC_TypeDef, GPIO_TypeDef) and macros (APB1PERIPH_BASE, DAC_BASE, DAC, GPIOA)
// are now provided by stm32l432xx.h - no need to redefine them

// DAC channel definitions
#define DAC_CHANNEL_1 1
#define DAC_CHANNEL_2 2

// DAC output pins
#define DAC_OUT1_PIN 4  // PA4
#define DAC_OUT2_PIN 5  // PA5

// Function prototypes
void DAC_EnableClock(void);
void DAC_ConfigureGPIO(int channel);
void DAC_Init(int channel);
void DAC_Start(int channel);
void DAC_Stop(int channel);
void DAC_SetValue(int channel, uint16_t value);
void DAC_InitAudio(int channel);
void DAC_PlaySineWave(float frequency, uint32_t duration_ms, uint32_t sample_rate);
void DAC_PlayWAV(const int16_t* sample_data, uint32_t sample_length, uint32_t sample_rate);
void DAC_TestOutput(int channel, uint16_t value, uint32_t duration_ms);  // Test function - output constant DC value

#endif
