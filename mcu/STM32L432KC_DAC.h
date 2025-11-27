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
#include "STM32L4xx/Device/Include/stm32l432xx.h"  // Include STM32 definitions for DAC_TypeDef, GPIO_TypeDef, GPIOA, __IO

// Base addresses (only define if not already defined)
#ifndef APB1PERIPH_BASE
#define APB1PERIPH_BASE (0x40000000UL)
#endif
#ifndef DAC_BASE
#define DAC_BASE (APB1PERIPH_BASE + 0x7400UL)
#endif

// DAC_TypeDef and DAC are now defined in stm32l432xx.h
// GPIO_TypeDef and GPIOA are also defined in stm32l432xx.h

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
