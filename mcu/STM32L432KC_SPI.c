// STM32L432KC_SPI.c
// Source code for SPI functions
// Based on Lab07

#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "debug_print.h"

/* Enables the SPI peripheral as SLAVE and initializes polarity and phase.
 *    -- br: Not used in slave mode (slave follows master's clock)
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPI(int br, int cpol, int cpha) {
    // Turn on GPIOA and GPIOB clock domains (GPIOAEN and GPIOBEN bits in AHB2ENR)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on SPI1 clock domain (SPI1EN bit in APB2ENR)

    // Configure SPI pins for SLAVE mode
    pinMode(SPI_SCK, GPIO_ALT);  // SPI1_SCK (input in slave mode)
    pinMode(SPI_MISO, GPIO_ALT); // SPI1_MISO (output in slave mode)
    pinMode(SPI_MOSI, GPIO_ALT); // SPI1_MOSI (input in slave mode)
    pinMode(SPI_CE, GPIO_INPUT);  // CS pin as INPUT (master controls CS)

    // Set output speed type to high for MISO (slave output)
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4);

    // Set to AF05 for SPI alternate functions
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL3, 5);  // SCK
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL4, 5);  // MISO
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL5, 5);  // MOSI
    
    // Configure SPI as SLAVE
    // Clear MSTR bit (stay in slave mode)
    SPI1->CR1 &= ~(SPI_CR1_MSTR);
    
    // Set CPOL and CPHA (Mode 0: CPOL=0, CPHA=0)
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPHA, cpha);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPOL, cpol);
    
    // Enable software slave management (SSM) - ignore hardware NSS pin
    // Set SSI (internal slave select) high so we're always selected
    // Note: When SSM=1, the NSS pin (PA11) is free for GPIO use (we use it to detect CS)
    // The SPI peripheral will always think it's selected (SSI=1), which is correct
    // for our application where master controls CS via GPIO
    SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);
    
    // Configure data size (8 bits) - DS[3:0] = 0b0111 for 8-bit frames
    // Per datasheet: "The data frame size is chosen by using the DS bits. It can be set
    // from 4-bit up to 16-bit length and the setting applies for both transmission and reception."
    SPI1->CR2 |= _VAL2FLD(SPI_CR2_DS, 0b0111);
    
    // Enable RX threshold (FRXTH=1 means RXNE when 1 byte in FIFO)
    // Per datasheet Section 40.4.9: "If FRXTH is set, RXNE goes high and stays high until
    // the RXFIFO level is greater or equal to 1/4 (8-bit)."
    // This is correct for 8-bit data frames
    SPI1->CR2 |= (SPI_CR2_FRXTH);
    
    // Don't set SSOE (slave select output enable) - we're slave, not master

    SPI1->CR1 |= (SPI_CR1_SPE); // Enable SPI
    
    // Debug: SPI initialization complete with register verification
    debug_printf("[SPI] SPI initialized as SLAVE: Mode %d (CPOL=%d, CPHA=%d)\r\n", 
                 (cpol << 1) | cpha, cpol, cpha);
    debug_printf("[SPI] CR1 register: 0x%08X\r\n", SPI1->CR1);
    debug_printf("[SPI] CR2 register: 0x%08X\r\n", SPI1->CR2);
    debug_printf("[SPI] CPOL=%d, CPHA=%d, MSTR=%d, SSM=%d, SSI=%d, SPE=%d\r\n", 
                 (SPI1->CR1 & SPI_CR1_CPOL) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_CPHA) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_MSTR) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SSM) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SSI) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SPE) ? 1 : 0);
    
    // Verify CS pin (PA11) is configured as input
    debug_printf("[SPI] CS pin (PA11) configured as INPUT (master controls CS)\r\n");
    debug_print("[SPI] SPI SLAVE configuration verified\r\n");
}

/* Receives a character (1 byte) over SPI in slave mode.
 *    -- return: the character received over SPI
 * Note: In slave mode, we receive data when master clocks it in.
 * According to datasheet Section 40.4.8: "The data register of the slave must already
 * contain data to be sent before starting communication with the master (either on the
 * first edge of the communication clock, or before the end of the ongoing communication
 * if the clock signal is continuous)."
 * 
 * For full-duplex mode: We pre-load dummy data (0x00) into TXFIFO before master clocks.
 * The master will clock in our dummy data while we receive the actual data from master. */
char spiReceive(void) {
    // CRITICAL: Pre-load dummy byte into TXFIFO BEFORE master starts clocking
    // Wait until transmit buffer is empty (can write)
    while(!(SPI1->SR & SPI_SR_TXE));
    
    // Write dummy byte to TXFIFO (will be sent when master clocks)
    // This must be done BEFORE master starts clocking (per datasheet 40.4.8)
    *(volatile char *) (&SPI1->DR) = 0x00; // Dummy byte to send
    
    // Wait until data has been received from master
    // RXNE is set when RXFIFO threshold is reached (FRXTH=1 means 1 byte)
    while(!(SPI1->SR & SPI_SR_RXNE));
    
    // Read received data from master
    char rec = (volatile char) SPI1->DR;
    
    // Debug: log SPI transaction (can be verbose - comment out if too slow)
    // debug_printf("[SPI] RX: 0x%x\r\n", (uint8_t)rec);
    
    return rec; // Return received character
}

/* Transmits a character (1 byte) over SPI and returns the received character.
 *    -- send: the character to send over SPI
 *    -- return: the character received over SPI
 * Note: In slave mode, master controls the clock. We write to DR and wait for RXNE. */
char spiSendReceive(char send) {
    // Wait until transmit buffer is empty
    while(!(SPI1->SR & SPI_SR_TXE));
    
    // Write data to transmit (slave mode - master will clock it out)
    *(volatile char *) (&SPI1->DR) = send;
    
    // Wait until data has been received (master clocks in data)
    while(!(SPI1->SR & SPI_SR_RXNE));
    
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

/* Read 16-byte sensor data packet from Arduino/ESP32 via SPI (SLAVE mode)
 * Packet format: [Header(0xAA)][Sensor1_Quat][Sensor1_Gyro][Sensor1_Flags]
 * All 16-bit values are MSB,LSB format (MSB first, LSB second)
 * Single sensor only - sensor 2 data is not included in packet
 * 
 * In SLAVE mode: Master (Arduino) controls CS and clock
 * - Wait for CS to go LOW (master starts transaction)
 * - Read 16 bytes as master clocks them in
 * - Wait for CS to go HIGH (transaction complete)
 */
void readSensorDataPacket(uint8_t *packet) {
    int i;
    uint32_t timeout;
    
    // Debug: Start of packet read
    debug_print("[SPI] Waiting for CS low (master starting transaction)...\r\n");
    
    // Wait for CS to go LOW (master pulls CS low to start transaction)
    // CS pin is PA11, configured as INPUT
    timeout = 1000000;  // Timeout counter (~12.5ms at 80MHz)
    while(digitalRead(PA11) == 1) {  // Wait for CS to go LOW
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] TIMEOUT: CS did not go low - no transaction from master\r\n");
            // Fill packet with zeros on timeout
            for(i = 0; i < 16; i++) {
                packet[i] = 0x00;
            }
            return;
        }
    }
    
    debug_print("[SPI] CS low detected - reading 16 bytes from master\r\n");
    
    // CRITICAL: Per datasheet Section 40.4.8, slave must have data ready BEFORE master clocks
    // Pre-load first dummy byte into TXFIFO immediately after CS goes low
    // This ensures data is ready when master starts clocking
    while(!(SPI1->SR & SPI_SR_TXE));  // Wait for TXFIFO space
    *(volatile char *) (&SPI1->DR) = 0x00;  // Pre-load first dummy byte
    
    // Small delay to allow TXFIFO to be ready (master may start clocking soon)
    volatile int setup_delay = 10;  // ~0.125us at 80MHz - minimal delay
    while(setup_delay-- > 0) __asm("nop");
    
    // Read 16 bytes - master will clock them in
    // Note: spiReceive() will pre-load the NEXT byte while reading the current one
    // This ensures continuous data flow per datasheet requirements
    for(i = 0; i < 16; i++) {
        packet[i] = spiReceive();  // Receive byte (master clocks it in)
        // Debug: log each byte received
        debug_printf("[SPI] Byte[%d] = 0x%x\r\n", i, packet[i]);
    }
    
    // Wait for SPI transaction to complete
    while(SPI1->SR & SPI_SR_BSY);  // Wait until SPI is not busy
    
    // Wait for CS to go HIGH (master ends transaction)
    timeout = 1000000;
    while(digitalRead(PA11) == 0) {  // Wait for CS to go HIGH
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] WARNING: CS did not go high after transaction\r\n");
            break;
        }
    }
    
    debug_print("[SPI] CS high - transaction complete. Full packet: ");
    debug_print_bytes(packet, 16);
    debug_newline();
    
    // Validate packet for floating pin patterns
    int all_ff = 1, all_00 = 1;
    for(i = 0; i < 16; i++) {
        if(packet[i] != 0xFF) all_ff = 0;
        if(packet[i] != 0x00) all_00 = 0;
    }
    
    if(all_ff) {
        debug_print("[SPI] WARNING: All bytes are 0xFF - check MISO connection!\r\n");
    } else if(all_00) {
        debug_print("[SPI] WARNING: All bytes are 0x00 - check MISO connection!\r\n");
    }
}

/* Parse 16-byte sensor data packet into structured format
 * Packet format: [Header(0xAA)][Sensor1_Quat][Sensor1_Gyro][Sensor1_Flags]
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
    debug_print("[SENSOR] Raw data bytes: quat_w=");
    debug_print_hex_byte(packet[1]); debug_print_hex_byte(packet[2]);
    debug_print(" quat_x=");
    debug_print_hex_byte(packet[3]); debug_print_hex_byte(packet[4]);
    debug_print(" quat_y=");
    debug_print_hex_byte(packet[5]); debug_print_hex_byte(packet[6]);
    debug_print(" quat_z=");
    debug_print_hex_byte(packet[7]); debug_print_hex_byte(packet[8]);
    debug_newline();
    
    debug_print("[SENSOR] Raw gyro bytes: gyro_x=");
    debug_print_hex_byte(packet[9]); debug_print_hex_byte(packet[10]);
    debug_print(" gyro_y=");
    debug_print_hex_byte(packet[11]); debug_print_hex_byte(packet[12]);
    debug_print(" gyro_z=");
    debug_print_hex_byte(packet[13]); debug_print_hex_byte(packet[14]);
    debug_print(" flags=");
    debug_print_hex_byte(packet[15]);
    debug_newline();
    
    // Sensor 1 Quaternion (bytes 1-8, MSB,LSB format)
    *quat1_w = (int16_t)((packet[1] << 8) | packet[2]);
    *quat1_x = (int16_t)((packet[3] << 8) | packet[4]);
    *quat1_y = (int16_t)((packet[5] << 8) | packet[6]);
    *quat1_z = (int16_t)((packet[7] << 8) | packet[8]);
    
    // Sensor 1 Gyroscope (bytes 9-14, MSB,LSB format)
    *gyro1_x = (int16_t)((packet[9] << 8) | packet[10]);
    *gyro1_y = (int16_t)((packet[11] << 8) | packet[12]);
    *gyro1_z = (int16_t)((packet[13] << 8) | packet[14]);
    
    // Sensor 1 Flags (byte 15)
    *quat1_valid = packet[15] & 0x01;
    *gyro1_valid = (packet[15] >> 1) & 0x01;
    uint8_t initialized = (packet[15] >> 2) & 0x01;
    uint8_t error = (packet[15] >> 3) & 0x01;
    
    // Debug: Print parsed values
    debug_printf("[SENSOR] Quat1: w=%d x=%d y=%d z=%d\r\n", *quat1_w, *quat1_x, *quat1_y, *quat1_z);
    debug_printf("[SENSOR] Gyro1: x=%d y=%d z=%d\r\n", *gyro1_x, *gyro1_y, *gyro1_z);
    debug_printf("[SENSOR] Flags: quat_valid=%d gyro_valid=%d init=%d error=%d\r\n", 
                 *quat1_valid, *gyro1_valid, initialized, error);
    
    // Sensor 2 - set to 0/invalid (not in 16-byte packet, kept for future compatibility)
    *quat2_w = *quat2_x = *quat2_y = *quat2_z = 0;
    *gyro2_x = *gyro2_y = *gyro2_z = 0;
    *quat2_valid = *gyro2_valid = 0;
}

