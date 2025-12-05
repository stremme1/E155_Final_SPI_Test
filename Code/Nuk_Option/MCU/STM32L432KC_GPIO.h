// STM32L432KC_GPIO.h
// Header for GPIO functions

#ifndef STM32L4_GPIO_H
#define STM32L4_GPIO_H

#include <stdint.h> // Include stdint header
#include "STM32L4xx/Device/Include/stm32l432xx.h" // Include STM32 definitions for GPIO_TypeDef, GPIOA, GPIOB

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

// Values which "val" can take on in digitalWrite()
#define PIO_LOW  0 // Value to write a pin low (0 V)
#define PIO_HIGH 1 // Value to write a pin high (3.3 V)

// For backward compatibility
#define GPIO_LOW    PIO_LOW
#define GPIO_HIGH   PIO_HIGH

// Arbitrary port IDs used to easily find a pin's port
#define GPIO_PORT_A 0 // Arbitrary ID for GPIO Port A
#define GPIO_PORT_B 1 // Arbitrary ID for GPIO Port B
#define GPIO_PORT_C 2 // Arbitrary ID for GPIO Port C

// Base addresses for GPIO ports (only define if not already defined)
#ifndef GPIOA_BASE
#define GPIOA_BASE  (0x48000000UL)
#endif
#ifndef GPIOB_BASE
#define GPIOB_BASE  (0x48000400UL)
#endif

// Pin definitions for every GPIO pin (matching starter code)
// Port A: 0-15, Port B: 16-31, Port C: 32-47
#define PA0    0
#define PA1    1
#define PA2    2
#define PA3    3
#define PA4    4
#define PA5    5
#define PA6    6
#define PA7    7
#define PA8    8
#define PA9    9
#define PA10   10
#define PA11   11
#define PA12   12
#define PA13   13
#define PA14   14
#define PA15   15
#define PB0    16
#define PB1    17
#define PB2    18
#define PB3    19
#define PB4    20
#define PB5    21
#define PB6    22
#define PB7    23
#define PB8    24
#define PB9    25
#define PB10   26
#define PB11   27
#define PB12   28
#define PB13   29
#define PB14   30
#define PB15   31

// Arbitrary GPIO functions for pinMode()
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_ALT    2
#define GPIO_ANALOG 3

// Values which "setting" can take on in pinResistor()
#define GPIO_PULL_UP   0 // Arbitrary ID for a pull-up resistor
#define GPIO_PULL_DOWN 1 // Arbitrary ID for a pull-down resistor
#define GPIO_FLOATING  2 // Arbitrary ID for a floating pin (neither resistor is active)

// GPIO_TypeDef, GPIOA, and GPIOB are now defined in stm32l432xx.h

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void gpioEnable(int port_id);
int gpioPinOffset(int gpio_pin);
int gpioPinToPort(int gpio_pin);
GPIO_TypeDef * gpioPortToBase(int port);
GPIO_TypeDef * gpioPinToBase(int gpio_pin);

void pinMode(int gpio_pin, int function);
int digitalRead(int gpio_pin);
void digitalWrite(int gpio_pin, int val);
void togglePin(int gpio_pin);

// Legacy functions for backward compatibility
void pinModePortA(int pin, int function);
void pinModePortB(int pin, int function);
int digitalReadPortA(int pin);
int digitalReadPortB(int pin);
void digitalWritePortA(int pin, int val);
void digitalWritePortB(int pin, int val);
void togglePinPortA(int pin);
void togglePinPortB(int pin);

#endif