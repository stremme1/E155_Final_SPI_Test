// Mock debug_print functions for unit testing
// This provides stub implementations so tests can compile without STM32 hardware

#include <stdio.h>
#include <stdint.h>

// Mock debug_print functions - just print to stdout for testing
void debug_init(void) {
    // No-op for testing
}

void debug_print(const char *str) {
    // Uncomment to see debug output during tests
    // printf("%s", str);
}

void debug_printf(const char *format, ...) {
    // Uncomment to see debug output during tests
    // va_list args;
    // va_start(args, format);
    // vprintf(format, args);
    // va_end(args);
}

void debug_print_bytes(const uint8_t *bytes, uint16_t length) {
    // Uncomment to see debug output during tests
    // for (uint16_t i = 0; i < length; i++) {
    //     printf("%02X ", bytes[i]);
    // }
}

void debug_print_hex_byte(uint8_t byte) {
    // Uncomment to see debug output during tests
    // printf("%02X", byte);
}

void debug_newline(void) {
    // Uncomment to see debug output during tests
    // printf("\n");
}

