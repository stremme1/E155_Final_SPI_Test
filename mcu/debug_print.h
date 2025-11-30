// debug_print.h
// Debug print utilities using SEGGER RTT (works over debugger connection)
// Output appears in SEGGER Embedded Studio Terminal window

#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include <stdint.h>
#include "STM32L432KC_USART.h"  // For USART_TypeDef (kept for compatibility)

// Debug USART instance (kept for compatibility, but RTT/printf is used)
extern USART_TypeDef *debug_usart;

// Initialize debug - RTT is automatically initialized by SEGGER runtime
void debug_init(void);

// Print a string
void debug_print(const char *str);

// Print formatted string (uses standard printf - supports %d, %x, %s, %c, %.2f, etc.)
void debug_printf(const char *format, ...);

// Print a byte as hex (e.g., "0xAA")
void debug_print_hex_byte(uint8_t byte);

// Print a 16-bit value as hex (e.g., "0xAABB")
void debug_print_hex_16(uint16_t value);

// Print a 32-bit value as hex (e.g., "0xAABBCCDD")
void debug_print_hex_32(uint32_t value);

// Print an array of bytes as hex dump
void debug_print_bytes(const uint8_t *data, uint32_t length);

// Print newline
void debug_newline(void);

#endif

