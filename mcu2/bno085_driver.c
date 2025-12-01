// bno085_driver.c
// BNO085 Sensor Driver for STM32L432KC
// Handles SHTP (Sensor Hub Transport Protocol) communication over SPI
// Ported from fpga/bno085_controller.sv
//
// Reference: Adafruit BNO08x library (https://github.com/adafruit/Adafruit_BNO08x)
// - Uses SPI Mode 3 (CPOL=1, CPHA=1) - matches FPGA implementation
// - Always waits for INT pin before read/write operations
// - Reads header first (4 bytes), then full packet based on length

#include "bno085_driver.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_TIMER.h"
#include "debug_print.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>  // For NULL

// Pin definitions (user must set these in main.c before calling bno085_init)
int BNO085_CS_PIN = 0;      // Chip Select (active low) - GPIO output
int BNO085_PS0_WAKE_PIN = 0; // PS0/WAKE (active low) - GPIO output
int BNO085_INT_PIN = 0;     // INT (active low) - GPIO input
int BNO085_RST_PIN = 0;      // Reset (active low, optional) - GPIO output

// State machine states
typedef enum {
    STATE_IDLE,
    STATE_INIT_WAIT_RESET,
    STATE_INIT_WAKE,
    STATE_INIT_WAIT_INT,
    STATE_INIT_CS_SETUP,
    STATE_INIT_SEND_BODY,
    STATE_INIT_WAIT_RESPONSE,
    STATE_INIT_READ_RESPONSE,
    STATE_INIT_DONE_CHECK,
    STATE_WAIT_DATA,
    STATE_READ_HEADER_START,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_ERROR
} bno085_state_t;

// Driver state
static bno085_state_t state = STATE_INIT_WAIT_RESET;
static bno085_data_t sensor_data = {0};
static bno085_status_t status = {false, false};

// State machine variables
static uint8_t byte_cnt = 0;
static uint16_t packet_length = 0;
static uint8_t channel = 0;
static uint8_t current_report_id = 0;
static uint32_t delay_counter = 0;
static uint8_t cmd_select = 0;  // 0=ProdID, 1=Rot, 2=Gyro
static uint8_t temp_byte_lsb = 0;
static uint8_t last_seq_num = 0;
static uint8_t report_status = 0;
static uint8_t report_delay = 0;

// Timing constants (for 80MHz system clock)
#define DELAY_100MS_CYCLES     8000000   // 100ms at 80MHz
#define DELAY_150US_CYCLES     12000     // 150µs at 80MHz
#define DELAY_10MS_CYCLES      800000    // 10ms at 80MHz
#define DELAY_50MS_CYCLES      4000000   // 50ms at 80MHz
#define DELAY_CS_SETUP_CYCLES  10        // ~0.1µs at 80MHz

// Get initialization command byte
static uint8_t get_init_byte(uint8_t cmd, uint8_t idx) {
    switch (cmd) {
        case 0:  // Product ID Request (5 bytes)
            switch (idx) {
                case 0: return 0x05;  // Length LSB
                case 1: return 0x00;  // Length MSB
                case 2: return 0x02;  // Channel 2 (SH-2 control)
                case 3: return 0x00;  // Seq 0
                case 4: return 0xF9;  // Report ID (Product ID Request)
                default: return 0x00;
            }
        case 1:  // Enable Rotation Vector (17 bytes)
            switch (idx) {
                case 0: return 17;    // Length LSB
                case 1: return 0x00;  // Length MSB
                case 2: return 0x02;  // Channel Control
                case 3: return 0x01;  // Seq 1
                case 4: return 0xFD;  // Set Feature
                case 5: return 0x05;  // Report ID (Rot Vec)
                case 6: return 0x00;  // Flags
                case 7: return 0x00;  // Sensitivity LSB
                case 8: return 0x00;  // Sensitivity MSB
                case 9: return 0x20;  // Report Interval LSB (20,000 µs = 50 Hz)
                case 10: return 0x4E;
                case 11: return 0x00;
                case 12: return 0x00; // Report Interval MSB
                default: return 0x00;
            }
        case 2:  // Enable Gyroscope (17 bytes)
            switch (idx) {
                case 0: return 17;    // Length LSB
                case 1: return 0x00;  // Length MSB
                case 2: return 0x02;  // Channel Control
                case 3: return 0x02;  // Seq 2
                case 4: return 0xFD;  // Set Feature
                case 5: return 0x02;  // Report ID (Calibrated Gyro)
                case 6: return 0x00;  // Flags
                case 7: return 0x00;  // Sensitivity LSB
                case 8: return 0x00;  // Sensitivity MSB
                case 9: return 0x20;  // Report Interval LSB (20,000 µs = 50 Hz)
                case 10: return 0x4E;
                case 11: return 0x00;
                case 12: return 0x00; // Report Interval MSB
                default: return 0x00;
            }
        default: return 0x00;
    }
}

// Get command length
static uint8_t get_cmd_len(uint8_t cmd) {
    switch (cmd) {
        case 0: return 5;   // Product ID
        case 1: return 17;  // Set Feature (Rot Vec)
        case 2: return 17;  // Set Feature (Gyro)
        default: return 0;
    }
}

// Delay function (busy wait)
static void delay_cycles(uint32_t cycles) {
    volatile uint32_t count = cycles;
    while (count-- > 0) {
        __asm("nop");
    }
}

// Check INT pin (active low)
static bool int_pin_low(void) {
    return (digitalRead(BNO085_INT_PIN) == 0);
}

// Wait for INT pin with timeout (based on Adafruit library approach)
// Returns true if INT asserted, false on timeout
static bool wait_for_int(uint32_t timeout_ms) {
    uint32_t timeout_cycles = (timeout_ms * 80000);  // 80MHz clock
    uint32_t counter = 0;
    
    while (counter < timeout_cycles) {
        if (int_pin_low()) {
            return true;
        }
        counter++;
        // Small delay to avoid too-fast polling
        if (counter % 1000 == 0) {
            volatile int nop_delay = 10;
            while (nop_delay-- > 0) __asm("nop");
        }
    }
    
    return false;  // Timeout
}

// Initialize BNO085 sensor
bool bno085_init(void) {
    // Check that pins are configured
    if (BNO085_CS_PIN == 0 || BNO085_PS0_WAKE_PIN == 0 || BNO085_INT_PIN == 0) {
        debug_print("[BNO085] ERROR: Pins not configured! Set BNO085_CS_PIN, BNO085_PS0_WAKE_PIN, BNO085_INT_PIN before calling init\r\n");
        return false;
    }
    
    // Configure GPIO pins
    debug_printf("[BNO085] Configuring GPIO pins: CS=PA%d, PS0=PA%d, INT=PA%d\r\n", 
                 BNO085_CS_PIN & 0x0F, BNO085_PS0_WAKE_PIN & 0x0F, BNO085_INT_PIN & 0x0F);
    pinMode(BNO085_CS_PIN, GPIO_OUTPUT);
    pinMode(BNO085_PS0_WAKE_PIN, GPIO_OUTPUT);
    pinMode(BNO085_INT_PIN, GPIO_INPUT);
    
    // Set initial pin states
    digitalWrite(BNO085_CS_PIN, 1);      // CS high (idle)
    digitalWrite(BNO085_PS0_WAKE_PIN, 1); // PS0 high (required for SPI mode)
    debug_print("[BNO085] Initial pin states: CS=HIGH, PS0=HIGH\r\n");
    
    // Reset state machine
    state = STATE_INIT_WAIT_RESET;
    status.initialized = false;
    status.error = false;
    delay_counter = 0;
    byte_cnt = 0;
    cmd_select = 0;
    
    debug_print("[BNO085] Initialization started - waiting 100ms after reset\r\n");
    return true;
}

// Service function - call this periodically in main loop
void bno085_service(void) {
    // Default: clear valid flags
    sensor_data.quat_valid = false;
    sensor_data.gyro_valid = false;
    
    switch (state) {
        case STATE_INIT_WAIT_RESET: {
            // Wait after reset to ensure sensor is ready (~100ms)
            digitalWrite(BNO085_CS_PIN, 1);
            digitalWrite(BNO085_PS0_WAKE_PIN, 1);
            
            if (delay_counter == 0) {
                debug_print("[BNO085] [INIT] Waiting 100ms after reset...\r\n");
            }
            
            if (delay_counter < DELAY_100MS_CYCLES) {
                delay_counter++;
            } else {
                delay_counter = 0;
                cmd_select = 0;  // Start with ProdID
                
                // Check if INT is already asserted
                uint8_t int_state = digitalRead(BNO085_INT_PIN);
                debug_printf("[BNO085] [INIT] Reset delay complete. INT pin state: %d\r\n", int_state);
                
                if (int_pin_low()) {
                    // INT already low - skip wake signal
                    debug_print("[BNO085] [INIT] INT already asserted - skipping wake signal\r\n");
                    state = STATE_INIT_CS_SETUP;
                } else {
                    // INT still high - need to use wake signal
                    debug_print("[BNO085] [INIT] INT not asserted - sending wake signal\r\n");
                    state = STATE_INIT_WAKE;
                }
            }
            break;
        }
        
        case STATE_INIT_WAKE: {
            // Wake the sensor (PS0 Low) - per datasheet 1.2.4.3
            if (delay_counter == 0) {
                debug_print("[BNO085] [INIT] Sending wake signal (PS0 LOW for 150µs)...\r\n");
            }
            digitalWrite(BNO085_PS0_WAKE_PIN, 0);  // Drive PS0 low
            
            if (delay_counter < DELAY_150US_CYCLES) {
                delay_counter++;
            } else {
                delay_counter = 0;
                debug_print("[BNO085] [INIT] Wake signal complete - waiting for INT\r\n");
                state = STATE_INIT_WAIT_INT;
            }
            break;
        }
        
        case STATE_INIT_WAIT_INT: {
            // Wait for INT low (Sensor Ready) with timeout
            // Use polling approach similar to Adafruit library
            if (delay_counter == 0) {
                debug_print("[BNO085] [INIT] Waiting for INT (timeout: 50ms)...\r\n");
            }
            
            if (int_pin_low()) {
                // INT asserted, sensor is ready
                debug_print("[BNO085] [INIT] INT asserted - sensor ready!\r\n");
                delay_counter = 0;
                state = STATE_INIT_CS_SETUP;
            } else if (delay_counter >= DELAY_50MS_CYCLES) {
                // Timeout - sensor didn't respond
                debug_printf("[BNO085] [ERROR] Timeout waiting for INT after wake (waited %d ms)\r\n", 
                            delay_counter / 80000);
                state = STATE_ERROR;
                status.error = true;
            } else {
                delay_counter++;
                // Add small delay every 1000 cycles to avoid too-fast polling
                if (delay_counter % 1000 == 0) {
                    volatile int nop_delay = 10;
                    while (nop_delay-- > 0) __asm("nop");
                }
            }
            break;
        }
        
        case STATE_INIT_CS_SETUP: {
            // CS setup before SPI transaction
            const char* cmd_names[] = {"Product ID", "Rotation Vector", "Gyroscope"};
            debug_printf("[BNO085] [INIT] Sending command: %s (cmd_select=%d)\r\n", cmd_names[cmd_select], cmd_select);
            digitalWrite(BNO085_CS_PIN, 0);  // Assert CS
            digitalWrite(BNO085_PS0_WAKE_PIN, 1);  // Release Wake once CS is asserted
            delay_cycles(DELAY_CS_SETUP_CYCLES);
            delay_counter = 0;
            byte_cnt = 0;
            state = STATE_INIT_SEND_BODY;
            break;
        }
        
        case STATE_INIT_SEND_BODY: {
            // Send command body
            digitalWrite(BNO085_CS_PIN, 0);
            
            uint8_t cmd_len = get_cmd_len(cmd_select);
            if (byte_cnt == 0) {
                debug_printf("[BNO085] [INIT] Sending command packet (%d bytes)...\r\n", cmd_len);
            }
            
            if (byte_cnt < cmd_len) {
                // Send byte
                uint8_t tx_byte = get_init_byte(cmd_select, byte_cnt);
                char rx_byte = spiSendReceive(tx_byte);
                debug_printf("[BNO085] [SPI] TX[%d/%d]: 0x%02X RX: 0x%02X\r\n", 
                            byte_cnt + 1, cmd_len, tx_byte, (uint8_t)rx_byte);
                byte_cnt++;
            } else {
                // Done sending all bytes
                debug_print("[BNO085] [INIT] Command sent - releasing CS, waiting for response\r\n");
                digitalWrite(BNO085_CS_PIN, 1);
                byte_cnt = 0;
                packet_length = 0;
                state = STATE_INIT_WAIT_RESPONSE;
                delay_counter = 0;
            }
            break;
        }
        
        case STATE_INIT_WAIT_RESPONSE: {
            // Wait for INT (response ready) after sending command
            // Based on Adafruit library: always wait for INT before reading
            digitalWrite(BNO085_CS_PIN, 1);
            digitalWrite(BNO085_PS0_WAKE_PIN, 1);
            
            if (delay_counter == 0) {
                debug_print("[BNO085] [INIT] Waiting for response (INT)...\r\n");
            }
            
            if (int_pin_low()) {
                // Response ready - read it
                debug_print("[BNO085] [INIT] INT asserted - reading response\r\n");
                byte_cnt = 0;
                packet_length = 0;
                state = STATE_INIT_READ_RESPONSE;
            } else if (delay_counter >= DELAY_50MS_CYCLES) {
                // Timeout - sensor didn't respond, move on anyway
                debug_printf("[BNO085] [INIT] Timeout waiting for response (waited %d ms) - continuing anyway\r\n", 
                            delay_counter / 80000);
                delay_counter = 0;
                state = STATE_INIT_DONE_CHECK;
            } else {
                delay_counter++;
                // Add small delay every 1000 cycles to avoid too-fast polling
                if (delay_counter % 1000 == 0) {
                    volatile int nop_delay = 10;
                    while (nop_delay-- > 0) __asm("nop");
                }
            }
            break;
        }
        
        case STATE_INIT_READ_RESPONSE: {
            // Read response to drain it
            digitalWrite(BNO085_CS_PIN, 0);
            
            if (packet_length == 0) {
                // Reading header
                if (byte_cnt == 0) {
                    debug_print("[BNO085] [INIT] Reading response header...\r\n");
                    char rx = spiSendReceive(0x00);
                    packet_length = (uint16_t)rx;
                    debug_printf("[BNO085] [SPI] Header[0]: 0x%02X (length LSB)\r\n", (uint8_t)rx);
                    byte_cnt = 1;
                } else if (byte_cnt == 1) {
                    char rx = spiSendReceive(0x00);
                    packet_length |= ((uint16_t)rx << 8);
                    packet_length &= 0x7FFF;  // Clear continuation bit
                    debug_printf("[BNO085] [SPI] Header[1]: 0x%02X (length MSB, total=%d)\r\n", 
                                (uint8_t)rx, packet_length);
                    byte_cnt = 2;
                } else if (byte_cnt == 2) {
                    char rx = spiSendReceive(0x00);
                    channel = (uint8_t)rx;
                    debug_printf("[BNO085] [SPI] Header[2]: 0x%02X (channel)\r\n", (uint8_t)rx);
                    byte_cnt = 3;
                } else if (byte_cnt == 3) {
                    char rx = spiSendReceive(0x00);
                    debug_printf("[BNO085] [SPI] Header[3]: 0x%02X (sequence)\r\n", (uint8_t)rx);
                    // Header complete - validate
                    if (packet_length > 4 && packet_length < 32767) {
                        debug_printf("[BNO085] [INIT] Header valid: length=%d, channel=0x%02X\r\n", 
                                    packet_length, channel);
                        debug_printf("[BNO085] [INIT] Reading payload (%d bytes)...\r\n", packet_length - 4);
                        byte_cnt = 0;  // Reset for payload counting
                    } else {
                        // Invalid length - done
                        debug_printf("[BNO085] [INIT] Invalid packet length: %d (ignoring)\r\n", packet_length);
                        digitalWrite(BNO085_CS_PIN, 1);
                        byte_cnt = 0;
                        packet_length = 0;
                        state = STATE_INIT_DONE_CHECK;
                        delay_counter = 0;
                        break;
                    }
                }
            } else {
                // Reading payload bytes
                char rx = spiSendReceive(0x00);
                if (byte_cnt < 8) {  // Only log first 8 bytes to avoid spam
                    debug_printf("[BNO085] [SPI] Payload[%d]: 0x%02X\r\n", byte_cnt, (uint8_t)rx);
                }
                byte_cnt++;
                
                if (byte_cnt >= (packet_length - 4)) {
                    // Done reading full response
                    debug_printf("[BNO085] [INIT] Response read complete (%d payload bytes)\r\n", byte_cnt);
                    digitalWrite(BNO085_CS_PIN, 1);
                    byte_cnt = 0;
                    packet_length = 0;
                    state = STATE_INIT_DONE_CHECK;
                    delay_counter = 0;
                }
            }
            break;
        }
        
        case STATE_INIT_DONE_CHECK: {
            digitalWrite(BNO085_CS_PIN, 1);
            digitalWrite(BNO085_PS0_WAKE_PIN, 1);
            
            if (cmd_select < 2) {
                // More commands to send - delay 10ms between commands
                if (delay_counter == 0) {
                    debug_printf("[BNO085] [INIT] Command %d complete - waiting 10ms before next command\r\n", cmd_select);
                }
                if (delay_counter < DELAY_10MS_CYCLES) {
                    delay_counter++;
                } else {
                    delay_counter = 0;
                    cmd_select++;
                    debug_printf("[BNO085] [INIT] Moving to next command (cmd_select=%d)\r\n", cmd_select);
                    state = STATE_INIT_WAKE;  // Go back to Wake for next command
                }
            } else {
                // All commands sent - wait additional time for sensor to process
                if (delay_counter == 0) {
                    debug_print("[BNO085] [INIT] All commands sent - waiting 100ms for sensor to process...\r\n");
                }
                if (delay_counter < DELAY_100MS_CYCLES) {
                    delay_counter++;
                } else {
                    status.initialized = true;
                    delay_counter = 0;
                    state = STATE_WAIT_DATA;
                    debug_print("[BNO085] [INIT] Initialization complete! Sensor ready for data reading.\r\n");
                }
            }
            break;
        }
        
        case STATE_WAIT_DATA: {
            // Normal Operation: Wait for Data Ready
            // Based on Adafruit library: poll INT pin for data ready
            digitalWrite(BNO085_CS_PIN, 1);
            digitalWrite(BNO085_PS0_WAKE_PIN, 1);
            
            if (int_pin_low()) {
                // INT asserted - data ready to read
                state = STATE_READ_HEADER_START;
                byte_cnt = 0;
            }
            // No timeout here - just keep polling (main loop will call this frequently)
            break;
        }
        
        case STATE_READ_HEADER_START: {
            // CS setup before SPI
            debug_print("[BNO085] [DATA] INT asserted - starting data read\r\n");
            digitalWrite(BNO085_CS_PIN, 0);
            delay_cycles(DELAY_CS_SETUP_CYCLES);
            byte_cnt = 0;
            state = STATE_READ_HEADER;
            break;
        }
        
        case STATE_READ_HEADER: {
            digitalWrite(BNO085_CS_PIN, 0);
            
            if (byte_cnt == 0) {
                char rx = spiSendReceive(0x00);
                packet_length = (uint16_t)rx;
                debug_printf("[BNO085] [SPI] Header[0]: 0x%02X (length LSB)\r\n", (uint8_t)rx);
                byte_cnt = 1;
            } else if (byte_cnt == 1) {
                char rx = spiSendReceive(0x00);
                packet_length |= ((uint16_t)rx << 8);
                packet_length &= 0x7FFF;  // Clear continuation bit
                debug_printf("[BNO085] [SPI] Header[1]: 0x%02X (length MSB, total=%d)\r\n", 
                            (uint8_t)rx, packet_length);
                byte_cnt = 2;
            } else if (byte_cnt == 2) {
                char rx = spiSendReceive(0x00);
                channel = (uint8_t)rx;
                debug_printf("[BNO085] [SPI] Header[2]: 0x%02X (channel)\r\n", (uint8_t)rx);
                byte_cnt = 3;
            } else if (byte_cnt == 3) {
                char rx = spiSendReceive(0x00);
                debug_printf("[BNO085] [SPI] Header[3]: 0x%02X (sequence)\r\n", (uint8_t)rx);
                // Validate packet length (SHTP packets: min 4 bytes header, max ~512 bytes typical)
                if (packet_length >= 4 && packet_length <= 512) {
                    debug_printf("[BNO085] [DATA] Header valid: length=%d, channel=0x%02X\r\n", 
                                packet_length, channel);
                    byte_cnt = 0;
                    state = STATE_READ_PAYLOAD;
                } else {
                    // Invalid length - consume remaining header bytes and discard
                    debug_printf("[BNO085] [ERROR] Invalid packet length: %d (expected 4-512, ignoring packet)\r\n", 
                                packet_length);
                    digitalWrite(BNO085_CS_PIN, 1);
                    byte_cnt = 0;
                    packet_length = 0;
                    state = STATE_WAIT_DATA;
                }
            }
            break;
        }
        
        case STATE_READ_PAYLOAD: {
            digitalWrite(BNO085_CS_PIN, 0);
            
            // Validate packet length to prevent buffer overrun
            if (byte_cnt >= (packet_length - 4)) {
                debug_printf("[BNO085] [ERROR] Packet read overflow: byte_cnt=%d, max=%d\r\n", 
                            byte_cnt, packet_length - 4);
                digitalWrite(BNO085_CS_PIN, 1);
                state = STATE_WAIT_DATA;
                byte_cnt = 0;
                break;
            }
            
            char rx = spiSendReceive(0x00);
            uint8_t rx_byte = (uint8_t)rx;
            
            // Parse on the fly - accept reports from Channel 3 or Channel 5
            if (channel == CHANNEL_REPORTS || channel == CHANNEL_GYRO_RV) {
                switch (byte_cnt) {
                    case 0:
                        current_report_id = rx_byte;
                        // Clear valid flags at start of new report
                        sensor_data.quat_valid = false;
                        sensor_data.gyro_valid = false;
                        debug_printf("[BNO085] [DATA] Payload[0]: 0x%02X (Report ID: 0x%02X)\r\n", 
                                    rx_byte, current_report_id);
                        
                        // Validate report ID - only accept known reports
                        if (current_report_id != REPORT_ID_ROTATION_VECTOR && 
                            current_report_id != REPORT_ID_GYROSCOPE) {
                            debug_printf("[BNO085] [WARN] Unknown report ID: 0x%02X (ignoring)\r\n", 
                                        current_report_id);
                            // Continue reading to consume packet, but don't parse
                        }
                        break;
                    case 1:
                        last_seq_num = rx_byte;
                        break;
                    case 2:
                        report_status = rx_byte;
                        break;
                    case 3:
                        report_delay = rx_byte;
                        break;
                    case 4:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            temp_byte_lsb = rx_byte;
                        } else if (current_report_id == REPORT_ID_GYROSCOPE) {
                            temp_byte_lsb = rx_byte;  // Gyro X LSB
                        }
                        break;
                    case 5:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            sensor_data.quat_x = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                        } else if (current_report_id == REPORT_ID_GYROSCOPE) {
                            sensor_data.gyro_x = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                        }
                        break;
                    case 6:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            temp_byte_lsb = rx_byte;  // Y-axis LSB
                        } else if (current_report_id == REPORT_ID_GYROSCOPE) {
                            temp_byte_lsb = rx_byte;  // Gyro Y LSB
                        }
                        break;
                    case 7:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            sensor_data.quat_y = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                        } else if (current_report_id == REPORT_ID_GYROSCOPE) {
                            sensor_data.gyro_y = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                        }
                        break;
                    case 8:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            temp_byte_lsb = rx_byte;  // Z-axis LSB
                        } else if (current_report_id == REPORT_ID_GYROSCOPE) {
                            temp_byte_lsb = rx_byte;  // Gyro Z LSB
                        }
                        break;
                    case 9:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            sensor_data.quat_z = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                        } else if (current_report_id == REPORT_ID_GYROSCOPE) {
                            sensor_data.gyro_z = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                            sensor_data.gyro_valid = true;
                            debug_printf("[BNO085] [DATA] Gyro complete: x=%d y=%d z=%d\r\n", 
                                        sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
                        }
                        break;
                    case 10:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            temp_byte_lsb = rx_byte;  // W-axis LSB
                        }
                        break;
                    case 11:
                        if (current_report_id == REPORT_ID_ROTATION_VECTOR) {
                            sensor_data.quat_w = (int16_t)((rx_byte << 8) | temp_byte_lsb);
                            
                            // Validate quaternion data (Q14 format: should be in range -16384 to +16384)
                            // Check for reasonable values (not all zeros, not max values)
                            int16_t quat_mag_sq = (sensor_data.quat_w * sensor_data.quat_w) / 16384 +
                                                  (sensor_data.quat_x * sensor_data.quat_x) / 16384 +
                                                  (sensor_data.quat_y * sensor_data.quat_y) / 16384 +
                                                  (sensor_data.quat_z * sensor_data.quat_z) / 16384;
                            
                            // Quaternion magnitude should be approximately 1.0 (in Q14: ~16384)
                            // Allow some tolerance: 0.5 to 2.0 (8192 to 32768 in Q14 squared)
                            if (quat_mag_sq > 8192 && quat_mag_sq < 32768) {
                                sensor_data.quat_valid = true;
                                debug_printf("[BNO085] [DATA] Quaternion complete: w=%d x=%d y=%d z=%d (mag^2=%d)\r\n", 
                                            sensor_data.quat_w, sensor_data.quat_x, 
                                            sensor_data.quat_y, sensor_data.quat_z, quat_mag_sq);
                            } else {
                                debug_printf("[BNO085] [WARN] Invalid quaternion magnitude: mag^2=%d (ignoring)\r\n", 
                                            quat_mag_sq);
                                sensor_data.quat_valid = false;
                            }
                        }
                        break;
                }
            } else {
                // Unknown channel - just consume bytes to complete packet
                debug_printf("[BNO085] [WARN] Unknown channel: 0x%02X (consuming packet)\r\n", channel);
            }
            
            byte_cnt++;
            
            // Continue reading if more data
            if (byte_cnt >= (packet_length - 4)) {
                // Packet complete
                debug_printf("[BNO085] [DATA] Packet read complete (%d payload bytes)\r\n", byte_cnt);
                digitalWrite(BNO085_CS_PIN, 1);
                byte_cnt = 0;
                state = STATE_WAIT_DATA;
            }
            break;
        }
        
        case STATE_ERROR: {
            if (!status.error) {
                debug_print("[BNO085] [ERROR] Entering error state - initialization failed\r\n");
                status.error = true;
                status.initialized = false;
            }
            digitalWrite(BNO085_CS_PIN, 1);
            digitalWrite(BNO085_PS0_WAKE_PIN, 1);
            
            // Recovery logic: attempt to reinitialize after delay
            if (delay_counter == 0) {
                debug_print("[BNO085] [ERROR] Waiting 1 second before recovery attempt...\r\n");
            }
            if (delay_counter < (DELAY_100MS_CYCLES * 10)) {  // 1 second
                delay_counter++;
            } else {
                debug_print("[BNO085] [ERROR] Attempting recovery - resetting state machine\r\n");
                delay_counter = 0;
                status.error = false;
                state = STATE_INIT_WAIT_RESET;  // Restart initialization
            }
            break;
        }
        
        default:
            state = STATE_IDLE;
            break;
    }
}

// Check if sensor is initialized
bool bno085_is_initialized(void) {
    return status.initialized;
}

// Check if sensor has error
bool bno085_has_error(void) {
    return status.error;
}

// Read sensor data (non-blocking)
// Returns true if new data was read, false otherwise
bool bno085_read_data(bno085_data_t *data) {
    if (data == NULL) return false;
    
    // Copy current sensor data
    *data = sensor_data;
    
    // Return true if we have valid data
    bool has_data = (sensor_data.quat_valid || sensor_data.gyro_valid);
    
    // Debug: log when new data is available (but not every time to avoid spam)
    static uint32_t last_log_counter = 0;
    if (has_data && (last_log_counter++ % 50 == 0)) {  // Log every 50th read
        if (sensor_data.quat_valid) {
            debug_printf("[BNO085] [DATA] Quat: w=%d x=%d y=%d z=%d\r\n", 
                        sensor_data.quat_w, sensor_data.quat_x, 
                        sensor_data.quat_y, sensor_data.quat_z);
        }
        if (sensor_data.gyro_valid) {
            debug_printf("[BNO085] [DATA] Gyro: x=%d y=%d z=%d\r\n", 
                        sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        }
    }
    
    return has_data;
}

