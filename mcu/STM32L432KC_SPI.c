// STM32L432KC_SPI.c
// Source code for SPI functions
// Based on Lab07

#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "debug_print.h"

/* Enables the SPI peripheral and intializes its clock speed (baud rate), polarity, and phase.
 *    -- br: (0b000 - 0b111). The SPI clk will be the master clock / 2^(BR+1).
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPI(int br, int cpol, int cpha) {
    // Turn on GPIOA and GPIOB clock domains (GPIOAEN and GPIOBEN bits in AHB2ENR)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on SPI1 clock domain (SPI1EN bit in APB2ENR)

    // Initially assigning SPI pins
    pinMode(SPI_SCK, GPIO_ALT); // SPI1_SCK
    pinMode(SPI_MISO, GPIO_ALT); // SPI1_MISO
    pinMode(SPI_MOSI, GPIO_ALT); // SPI1_MOSI
    pinMode(SPI_CE, GPIO_OUTPUT); // Manual CS

    // Set output speed type to high for SCK
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3);

    // Set to AF05 for SPI alternate functions
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL3, 5);
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL4, 5);
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL5, 5);
    
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_BR, br); // Set baud rate divider
    SPI1->CR1 |= (SPI_CR1_MSTR);
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST | SPI_CR1_SSM);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPHA, cpha);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPOL, cpol);
    SPI1->CR2 |= _VAL2FLD(SPI_CR2_DS, 0b0111);
    SPI1->CR2 |= (SPI_CR2_FRXTH | SPI_CR2_SSOE);

    SPI1->CR1 |= (SPI_CR1_SPE); // Enable SPI
    
    // Debug: SPI initialization complete with register verification
    // Calculate actual SPI clock based on BR value
    int spi_clock_khz = 80000 / (1 << (br + 1));  // 80MHz / 2^(BR+1)
    debug_printf("[SPI] SPI initialized: Mode 0, BR=%d (%d kHz), CS=PA11\r\n", br, spi_clock_khz);
    debug_printf("[SPI] CR1 register: 0x%08X\r\n", SPI1->CR1);
    debug_printf("[SPI] CR2 register: 0x%08X\r\n", SPI1->CR2);
    debug_printf("[SPI] CPOL=%d, CPHA=%d, MSTR=%d, SPE=%d\r\n", 
                 (SPI1->CR1 & SPI_CR1_CPOL) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_CPHA) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_MSTR) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SPE) ? 1 : 0);
    
    // Verify CS pin (PA11) is configured correctly
    // CS should be output and initially high
    digitalWrite(PA11, 1);  // Ensure CS starts high
    debug_printf("[SPI] CS pin (PA11) state: %d (should be 1/high)\r\n", digitalRead(PA11));
    debug_print("[SPI] SPI configuration verified\r\n");
}

/* Transmits a character (1 byte) over SPI and returns the received character.
 *    -- send: the character to send over SPI
 *    -- return: the character received over SPI */
char spiSendReceive(char send) {
    while(!(SPI1->SR & SPI_SR_TXE)); // Wait until the transmit buffer is empty
    *(volatile char *) (&SPI1->DR) = send; // Transmit the character over SPI
    while(!(SPI1->SR & SPI_SR_RXNE)); // Wait until data has been received
    char rec = (volatile char) SPI1->DR;
    
    // Debug: log SPI transaction (can be verbose - comment out if too slow)
    // debug_printf("[SPI] TX: 0x%x RX: 0x%x\r\n", (uint8_t)send, (uint8_t)rec);
    
    return rec; // Return received character
}

/* Initialize SPI control pins (LOAD and DONE) - Lab07 style */
void initSPIControlPins(void) {
    // This function is now redundant - pins are configured in main() like Lab07
    // Keeping for compatibility but it's not needed
}

/* Read drum command from FPGA via SPI - Lab07 style */
uint8_t readDrumCommand(void) {
    uint8_t command;
    
    // Wait for DONE signal (like Lab07 line 133)
    while(!digitalRead(PA6));
    
    // Read command byte (like Lab07 lines 136-138)
    digitalWrite(PA11, 1);
    command = spiSendReceive(0);
    digitalWrite(PA11, 0);
    
    while(SPI1->SR & SPI_SR_BSY); // Confirm all SPI transactions are completed (like Lab07 line 129)
    
    // Acknowledge with LOAD
    digitalWrite(PA5, 1);
    digitalWrite(PA5, 0);
    
    return command;
}

/* Read 15-byte sensor data packet from FPGA via SPI - Lab07 style */
void readSensorDataPacket15(uint8_t *packet) {
    int i;
    
    // Wait for DONE signal (like Lab07 line 142)
    while(!digitalRead(PA6));
    
    // Read 15 bytes (like Lab07 lines 144-148)
    for(i = 0; i < 15; i++) {
        digitalWrite(PA11, 1); // CE high (like Lab07 line 145)
        packet[i] = spiSendReceive(0);  
        digitalWrite(PA11, 0); // CE low (like Lab07 line 147)
    }
    
    while(SPI1->SR & SPI_SR_BSY); // Confirm all SPI transactions are completed (like Lab07 line 138)
    
    // Acknowledge with LOAD (like Lab07 pattern)
    digitalWrite(PA5, 1);
    digitalWrite(PA5, 0);
}

/* Parse 15-byte sensor data packet into structured format
 * Packet format: [Header(0xAA)][Quat W LSB][Quat W MSB][Quat X LSB][Quat X MSB]...
 * All 16-bit values are little-endian (LSB first, MSB second)
 */
void parseSensorDataPacket15(const uint8_t *packet, 
                              int16_t *quat_w, int16_t *quat_x, int16_t *quat_y, int16_t *quat_z,
                              int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    // Verify header
    if (packet[0] != 0xAA) {
        // Invalid header - set all values to 0
        *quat_w = *quat_x = *quat_y = *quat_z = 0;
        *gyro_x = *gyro_y = *gyro_z = 0;
        return;
    }
    
    // Parse quaternion (little-endian: LSB first, MSB second)
    // Byte 1 = LSB, Byte 2 = MSB
    *quat_w = (int16_t)(packet[1] | (packet[2] << 8));
    *quat_x = (int16_t)(packet[3] | (packet[4] << 8));
    *quat_y = (int16_t)(packet[5] | (packet[6] << 8));
    *quat_z = (int16_t)(packet[7] | (packet[8] << 8));
    
    // Parse gyroscope (little-endian: LSB first, MSB second)
    *gyro_x = (int16_t)(packet[9]  | (packet[10] << 8));
    *gyro_y = (int16_t)(packet[11] | (packet[12] << 8));
    *gyro_z = (int16_t)(packet[13] | (packet[14] << 8));
}

/* Read 16-byte sensor data packet from FPGA via SPI - CS-based protocol
 * Packet format: [Header(0xAA)][Sensor1_Quat][Sensor1_Gyro][Sensor1_Flags]
 * All 16-bit values are MSB,LSB format (MSB first, LSB second)
 * Single sensor only - sensor 2 data is not included in packet
 * Uses CS (chip select) control: CS low → read bytes → CS high
 */
void readSensorDataPacket(uint8_t *packet) {
    int i;
    
    // Debug: Start of packet read
    debug_print("[SPI] Starting packet read - CS low\r\n");
    
    // CS-based protocol: Pull CS low to start transaction
    // CRITICAL: CS must stay low for the ENTIRE 16-byte transaction
    // If CS toggles between bytes, the FPGA resets and reloads the first byte
    digitalWrite(PA11, 0);  // CS low
    
    // Read 16 bytes using dummy bytes (0x00) to generate SCK
    // CS stays low for all 16 bytes - this is critical!
    // No delay needed - FPGA pre-initializes shift_out when CS is high
    // First bit should be ready immediately when CS goes low
    for(i = 0; i < 16; i++) {
        packet[i] = spiSendReceive(0x00);
        // Debug: log each byte received
        debug_printf("[SPI] Byte[%d] = 0x%x\r\n", i, packet[i]);
    }
    
    // Wait for SPI transaction to complete
    while(SPI1->SR & SPI_SR_BSY);  // Wait until SPI is not busy
    
    // Pull CS high to end transaction (only after all 16 bytes are read)
    digitalWrite(PA11, 1);  // CS high
    
    // Debug: Complete packet dump
    debug_print("[SPI] Packet complete - CS high. Full packet: ");
    debug_print_bytes(packet, 16);
    debug_newline();
    
    // Validate packet for floating pin patterns
    int all_ff = 1, all_00 = 1;
    for(i = 0; i < 16; i++) {
        if(packet[i] != 0xFF) all_ff = 0;
        if(packet[i] != 0x00) all_00 = 0;
    }
    
    if(all_ff) {
        debug_print("[SPI] WARNING: All bytes are 0xFF - MISO pin may be floating!\r\n");
    } else if(all_00) {
        debug_print("[SPI] WARNING: All bytes are 0x00 - MISO pin may be floating or pulled low!\r\n");
    }
}

/* Parse 16-byte sensor data packet into structured format
 * See DATA_PIPELINE_VERIFICATION.md for complete pipeline documentation
 * 
 * Data Pipeline: Arduino → FPGA → MCU (this code)
 * 
 * Arduino packet format (16 bytes):
 * Byte 0:    Header (0xAA)
 * Bytes 1-2: Roll (int16_t, MSB first) - Euler angle scaled by 100
 * Bytes 3-4: Pitch (int16_t, MSB first) - Euler angle scaled by 100
 * Bytes 5-6: Yaw (int16_t, MSB first) - Euler angle scaled by 100
 * Bytes 7-8: Gyro X (int16_t, MSB first) - scaled by 2000
 * Bytes 9-10: Gyro Y (int16_t, MSB first) - scaled by 2000
 * Bytes 11-12: Gyro Z (int16_t, MSB first) - scaled by 2000
 * Byte 13:   Flags (bit 0 = Euler valid, bit 1 = Gyro valid)
 * Bytes 14-15: Reserved (0x00)
 * 
 * Mapped to quaternion format:
 * quat_w = 16384 (Q14 format = 1.0, hardcoded for Euler angles)
 * quat_x = Roll
 * quat_y = Pitch
 * quat_z = Yaw
 * 
 * All 16-bit values are MSB,LSB format (MSB first, LSB second)
 * Single sensor only - sensor 2 outputs are set to 0/invalid for future compatibility
 */
void parseSensorDataPacket(const uint8_t *packet,
                           // Sensor 1 (Right Hand)
                           int16_t *quat1_w, int16_t *quat1_x, int16_t *quat1_y, int16_t *quat1_z,
                           int16_t *gyro1_x, int16_t *gyro1_y, int16_t *gyro1_z,
                           uint8_t *quat1_valid, uint8_t *gyro1_valid,
                           // Sensor 2 (Left Hand) - set to 0/invalid (not in packet)
                           int16_t *quat2_w, int16_t *quat2_x, int16_t *quat2_y, int16_t *quat2_z,
                           int16_t *gyro2_x, int16_t *gyro2_y, int16_t *gyro2_z,
                           uint8_t *quat2_valid, uint8_t *gyro2_valid) {
    // Verify header
    debug_printf("[SENSOR] Parsing packet - Header: 0x%x\r\n", packet[0]);
    
    // Check for floating pin patterns first
    int all_ff = 1, all_00 = 1;
    for(int i = 0; i < 16; i++) {
        if(packet[i] != 0xFF) all_ff = 0;
        if(packet[i] != 0x00) all_00 = 0;
    }
    
    if(all_ff) {
        debug_print("[SENSOR] ERROR: All bytes are 0xFF - MISO pin floating or not connected!\r\n");
    } else if(all_00) {
        debug_print("[SENSOR] ERROR: All bytes are 0x00 - MISO pin floating or pulled low!\r\n");
    }
    
    if (packet[0] != 0xAA) {
        // Invalid header - set all values to 0
        debug_printf("[SENSOR] ERROR: Invalid header! Expected 0xAA, got 0x%x\r\n", packet[0]);
        if(!all_ff && !all_00) {
            debug_print("[SENSOR] Data appears valid but header is wrong - check FPGA shift register!\r\n");
        }
        *quat1_w = *quat1_x = *quat1_y = *quat1_z = 0;
        *gyro1_x = *gyro1_y = *gyro1_z = 0;
        *quat1_valid = *gyro1_valid = 0;
        *quat2_w = *quat2_x = *quat2_y = *quat2_z = 0;
        *gyro2_x = *gyro2_y = *gyro2_z = 0;
        *quat2_valid = *gyro2_valid = 0;
        return;
    }
    
    debug_print("[SENSOR] Header valid (0xAA)\r\n");
    
    // Debug: Show raw sensor data bytes to help diagnose zero data issue
    // Use manual hex formatting to avoid printf format string issues
    debug_print("[SENSOR] Raw data bytes: Roll=");
    debug_print_hex_byte(packet[1]); debug_print_hex_byte(packet[2]);
    debug_print(" Pitch=");
    debug_print_hex_byte(packet[3]); debug_print_hex_byte(packet[4]);
    debug_print(" Yaw=");
    debug_print_hex_byte(packet[5]); debug_print_hex_byte(packet[6]);
    debug_newline();
    
    debug_print("[SENSOR] Raw gyro bytes: gyro_x=");
    debug_print_hex_byte(packet[7]); debug_print_hex_byte(packet[8]);
    debug_print(" gyro_y=");
    debug_print_hex_byte(packet[9]); debug_print_hex_byte(packet[10]);
    debug_print(" gyro_z=");
    debug_print_hex_byte(packet[11]); debug_print_hex_byte(packet[12]);
    debug_print(" flags=");
    debug_print_hex_byte(packet[13]);
    debug_newline();
    
    // Parse Arduino packet format (Roll/Pitch/Yaw)
    int16_t roll = (int16_t)((packet[1] << 8) | packet[2]);
    int16_t pitch = (int16_t)((packet[3] << 8) | packet[4]);
    int16_t yaw = (int16_t)((packet[5] << 8) | packet[6]);
    
    // Map to quaternion format (Euler angles → quaternion)
    *quat1_w = 16384;  // Q14 format = 1.0 (hardcoded for Euler angles)
    *quat1_x = roll;   // Roll → quat_x
    *quat1_y = pitch;  // Pitch → quat_y
    *quat1_z = yaw;    // Yaw → quat_z
    
    // Sensor 1 Gyroscope (bytes 7-12, MSB,LSB format)
    *gyro1_x = (int16_t)((packet[7] << 8) | packet[8]);
    *gyro1_y = (int16_t)((packet[9] << 8) | packet[10]);
    *gyro1_z = (int16_t)((packet[11] << 8) | packet[12]);
    
    // Sensor 1 Flags (byte 13: bit 0 = Euler valid, bit 1 = Gyro valid)
    *quat1_valid = packet[13] & 0x01;  // Euler valid → quat_valid
    *gyro1_valid = (packet[13] >> 1) & 0x01;  // Gyro valid
    
    // Debug: Print parsed values
    debug_printf("[SENSOR] Quat1: w=%d x=%d y=%d z=%d\r\n", *quat1_w, *quat1_x, *quat1_y, *quat1_z);
    debug_printf("[SENSOR] Gyro1: x=%d y=%d z=%d\r\n", *gyro1_x, *gyro1_y, *gyro1_z);
    debug_printf("[SENSOR] Flags: quat_valid=%d gyro_valid=%d\r\n", *quat1_valid, *gyro1_valid);
    
    // Sensor 2 - set to 0/invalid (not in 16-byte packet, kept for future compatibility)
    *quat2_w = *quat2_x = *quat2_y = *quat2_z = 0;
    *gyro2_x = *gyro2_y = *gyro2_z = 0;
    *quat2_valid = *gyro2_valid = 0;
}

