// debug_print.c
// Debug print utilities using SEGGER RTT (works over debugger connection)

#include "debug_print.h"
#include <stdarg.h>
#include <stddef.h>  // For NULL
#include <stdio.h>   // For printf (uses RTT in SEGGER Embedded Studio)

// Debug USART instance (kept for compatibility, but we use RTT/printf now)
USART_TypeDef *debug_usart = NULL;

// Initialize debug - RTT is automatically initialized by SEGGER runtime
// No hardware initialization needed - works over debugger connection
void debug_init(void) {
    // RTT is automatically available in SEGGER Embedded Studio
    // No initialization needed - printf will use RTT automatically
    // Optionally initialize USART for physical serial connection too
    // debug_usart = initUSART(USART2_ID, 115200);
}

// Print a string using printf (RTT)
void debug_print(const char *str) {
    if (str == NULL) return;
    printf("%s", str);
    fflush(stdout);  // Ensure output is flushed immediately
}

// Print newline
void debug_newline(void) {
    printf("\r\n");
    fflush(stdout);
}

// Print a byte as hex (e.g., "0xAA")
void debug_print_hex_byte(uint8_t byte) {
    // Build hex string manually to avoid format string issues
    char hex_chars[] = "0123456789ABCDEF";
    char hex_str[5] = "0x00";
    hex_str[2] = hex_chars[(byte >> 4) & 0x0F];
    hex_str[3] = hex_chars[byte & 0x0F];
    printf("%s", hex_str);
    fflush(stdout);
}

// Print a 16-bit value as hex (e.g., "0xAABB")
void debug_print_hex_16(uint16_t value) {
    printf("0x%04X", value);
    fflush(stdout);
}

// Print a 32-bit value as hex (e.g., "0xAABBCCDD")
void debug_print_hex_32(uint32_t value) {
    printf("0x%08X", value);
    fflush(stdout);
}

// Print an array of bytes as hex dump
void debug_print_bytes(const uint8_t *data, uint32_t length) {
    if (data == NULL) return;
    
    for (uint32_t i = 0; i < length; i++) {
        debug_print_hex_byte(data[i]);
        if (i < length - 1) {
            printf(" ");
        }
    }
    fflush(stdout);
}

// Simple printf-like function (supports %d, %x, %s, %c, %.2f, etc.)
// Uses standard printf which goes to RTT in SEGGER Embedded Studio
void debug_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);  // Ensure output is flushed immediately
}


