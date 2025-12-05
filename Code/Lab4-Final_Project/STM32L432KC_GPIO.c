// STM32L432KC_GPIO.c
// Source code for GPIO functions

#include "STM32L432KC_GPIO.h"

// Helper function to set pin mode on a specific port
static void pinModeHelper(GPIO_TypeDef *port, int pin, int function) {
    switch(function) {
        case GPIO_INPUT:
            port->MODER &= ~(0b11 << 2*pin);
            break;
        case GPIO_OUTPUT:
            port->MODER |= (0b1 << 2*pin);
            port->MODER &= ~(0b1 << (2*pin+1));
            break;
        case GPIO_ALT:
            port->MODER &= ~(0b1 << 2*pin);
            port->MODER |= (0b1 << (2*pin+1));
            break;
        case GPIO_ANALOG:
            port->MODER |= (0b11 << 2*pin);
            break;
    }
}

// Legacy function (defaults to GPIOB for backward compatibility)
void pinMode(int pin, int function) {
    pinModePortB(pin, function);
}

void pinModePortA(int pin, int function) {
    pinModeHelper(GPIOA, pin, function);
}

void pinModePortB(int pin, int function) {
    pinModeHelper(GPIOB, pin, function);
}

// Helper function to read pin on a specific port
static int digitalReadHelper(GPIO_TypeDef *port, int pin) {
    return ((port->IDR) >> pin) & 1;
}

// Legacy function (defaults to GPIOB for backward compatibility)
int digitalRead(int pin) {
    return digitalReadPortB(pin);
}

int digitalReadPortA(int pin) {
    return digitalReadHelper(GPIOA, pin);
}

int digitalReadPortB(int pin) {
    return digitalReadHelper(GPIOB, pin);
}

// Helper function to write pin on a specific port
static void digitalWriteHelper(GPIO_TypeDef *port, int pin, int val) {
    if (val) {
        port->BSRR = (1 << pin);  // Set bit
    } else {
        port->BSRR = (1 << (pin + 16));  // Reset bit
    }
}

// Legacy function (defaults to GPIOB for backward compatibility)
void digitalWrite(int pin, int val) {
    digitalWritePortB(pin, val);
}

void digitalWritePortA(int pin, int val) {
    digitalWriteHelper(GPIOA, pin, val);
}

void digitalWritePortB(int pin, int val) {
    digitalWriteHelper(GPIOB, pin, val);
}

// Helper function to toggle pin on a specific port
static void togglePinHelper(GPIO_TypeDef *port, int pin) {
    port->ODR ^= (1 << pin);
}

// Legacy function (defaults to GPIOB for backward compatibility)
void togglePin(int pin) {
    togglePinPortB(pin);
}

void togglePinPortA(int pin) {
    togglePinHelper(GPIOA, pin);
}

void togglePinPortB(int pin) {
    togglePinHelper(GPIOB, pin);
}