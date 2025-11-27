// STM32L432KC_GPIO.h
// Header for GPIO functions

#ifndef STM32L4_GPIO_H
#define STM32L4_GPIO_H

#include <stdint.h> // Include stdint header
#include "STM32L4xx/Device/Include/stm32l432xx.h" // Include STM32 definitions for GPIO_TypeDef, GPIOA, GPIOB

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

// Values for GPIO pins ("val" arguments)
#define GPIO_LOW    0
#define GPIO_HIGH   1

// Base addresses for GPIO ports (only define if not already defined)
#ifndef GPIOA_BASE
#define GPIOA_BASE  (0x48000000UL)
#endif
#ifndef GPIOB_BASE
#define GPIOB_BASE  (0x48000400UL)
#endif

// Pin definitions for convenience
#define PA0  0
#define PA1  1
#define PA2  2
#define PA3  3
#define PA4  4
#define PA5  5
#define PA6  6
#define PA7  7
#define PA8  8
#define PA9  9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15

#define PB0  0
#define PB1  1
#define PB2  2
#define PB3  3
#define PB4  4
#define PB5  5
#define PB6  6
#define PB7  7
#define PB8  8
#define PB9  9
#define PB10 10
#define PB11 11
#define PB12 12
#define PB13 13
#define PB14 14
#define PB15 15

// Arbitrary GPIO functions for pinMode()
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_ALT    2
#define GPIO_ANALOG 3

// GPIO_TypeDef, GPIOA, and GPIOB are now defined in stm32l432xx.h

// Legacy compatibility
#ifndef GPIO
#define GPIO GPIOB
#endif

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void pinMode(int pin, int function);
void pinModePortA(int pin, int function);
void pinModePortB(int pin, int function);

int digitalRead(int pin);
int digitalReadPortA(int pin);
int digitalReadPortB(int pin);

void digitalWrite(int pin, int val);
void digitalWritePortA(int pin, int val);
void digitalWritePortB(int pin, int val);

void togglePin(int pin);
void togglePinPortA(int pin);
void togglePinPortB(int pin);

#endif