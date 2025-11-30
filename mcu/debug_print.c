// debug_print.c
// Debug print utilities for USART serial output

#include "debug_print.h"
#include <stdarg.h>
#include <stddef.h>  // For NULL

// Debug USART instance
USART_TypeDef *debug_usart = NULL;

// Initialize debug USART (USART2 on PA2/PA15)
void debug_init(void) {
    debug_usart = initUSART(USART2_ID, 115200);
}

// Print a string
void debug_print(const char *str) {
    if (debug_usart == NULL) return;
    sendString(debug_usart, (char *)str);
}

// Print newline
void debug_newline(void) {
    if (debug_usart == NULL) return;
    sendChar(debug_usart, '\r');
    sendChar(debug_usart, '\n');
}

// Print a byte as hex (e.g., "0xAA")
void debug_print_hex_byte(uint8_t byte) {
    if (debug_usart == NULL) return;
    
    char hex_chars[] = "0123456789ABCDEF";
    sendChar(debug_usart, '0');
    sendChar(debug_usart, 'x');
    sendChar(debug_usart, hex_chars[(byte >> 4) & 0x0F]);
    sendChar(debug_usart, hex_chars[byte & 0x0F]);
}

// Print a 16-bit value as hex (e.g., "0xAABB")
void debug_print_hex_16(uint16_t value) {
    if (debug_usart == NULL) return;
    
    char hex_chars[] = "0123456789ABCDEF";
    sendChar(debug_usart, '0');
    sendChar(debug_usart, 'x');
    sendChar(debug_usart, hex_chars[(value >> 12) & 0x0F]);
    sendChar(debug_usart, hex_chars[(value >> 8) & 0x0F]);
    sendChar(debug_usart, hex_chars[(value >> 4) & 0x0F]);
    sendChar(debug_usart, hex_chars[value & 0x0F]);
}

// Print a 32-bit value as hex (e.g., "0xAABBCCDD")
void debug_print_hex_32(uint32_t value) {
    if (debug_usart == NULL) return;
    
    char hex_chars[] = "0123456789ABCDEF";
    sendChar(debug_usart, '0');
    sendChar(debug_usart, 'x');
    for (int i = 28; i >= 0; i -= 4) {
        sendChar(debug_usart, hex_chars[(value >> i) & 0x0F]);
    }
}

// Print an array of bytes as hex dump
void debug_print_bytes(const uint8_t *data, uint32_t length) {
    if (debug_usart == NULL || data == NULL) return;
    
    for (uint32_t i = 0; i < length; i++) {
        debug_print_hex_byte(data[i]);
        if (i < length - 1) {
            sendChar(debug_usart, ' ');
        }
    }
}

// Simple integer to string conversion
static void itoa(int value, char *str, int base) {
    char *ptr = str;
    char *ptr1 = str;
    char tmp_char;
    int tmp_value;
    int is_negative = 0;

    if (value < 0 && base == 10) {
        is_negative = 1;
        value = -value;
    }

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while (value);

    if (is_negative) {
        *ptr++ = '-';
    }
    *ptr-- = '\0';
    
    // Reverse string
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
}

// Simple float to string conversion (2 decimal places)
static void ftoa(float value, char *str) {
    int int_part = (int)value;
    float frac = value - int_part;
    if (frac < 0) frac = -frac;
    int frac_part = (int)(frac * 100);
    
    itoa(int_part, str, 10);
    char *p = str;
    while (*p != '\0') p++;
    *p++ = '.';
    if (frac_part < 10) *p++ = '0';
    itoa(frac_part, p, 10);
}

// Simple printf-like function (supports %d, %x, %s, %c, %.2f)
void debug_printf(const char *format, ...) {
    if (debug_usart == NULL) return;
    
    va_list args;
    va_start(args, format);
    
    char buffer[32];
    const char *p = format;
    
    while (*p != '\0') {
        if (*p == '%') {
            p++;
            // Check for %.2f format
            if (*p == '.' && *(p+1) == '2' && *(p+2) == 'f') {
                float val = (float)va_arg(args, double);
                ftoa(val, buffer);
                debug_print(buffer);
                p += 2; // Skip past '2f'
            } else {
                switch (*p) {
                    case 'd': {
                        int val = va_arg(args, int);
                        itoa(val, buffer, 10);
                        debug_print(buffer);
                        break;
                    }
                    case 'x': {
                        uint32_t val = va_arg(args, uint32_t);
                        debug_print_hex_32(val);
                        break;
                    }
                    case 's': {
                        char *str = va_arg(args, char *);
                        debug_print(str);
                        break;
                    }
                    case 'c': {
                        char c = (char)va_arg(args, int);
                        sendChar(debug_usart, c);
                        break;
                    }
                    case '%': {
                        sendChar(debug_usart, '%');
                        break;
                    }
                    default:
                        sendChar(debug_usart, '%');
                        sendChar(debug_usart, *p);
                        break;
                }
            }
        } else {
            sendChar(debug_usart, *p);
        }
        p++;
    }
    
    va_end(args);
}

