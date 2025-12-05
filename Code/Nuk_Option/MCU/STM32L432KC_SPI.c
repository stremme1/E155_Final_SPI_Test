// STM32L432KC_SPI.c
// Source code for SPI functions
// Modified for SPI Slave mode (Arduino as master)

#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "debug_print.h"

/* Enables the SPI peripheral as SLAVE and initializes polarity and phase.
 * Follows datasheet Section 40.4.7 configuration procedure.
 *    -- br: Not used in slave mode (slave follows master's clock)
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPI(int br, int cpol, int cpha) {
    // Step 1: Configure GPIO pins (Section 40.4.7, step 1)
    // Turn on GPIOA and GPIOB clock domains (GPIOAEN and GPIOBEN bits in AHB2ENR)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    
    // Turn on SPI1 clock domain (SPI1EN bit in APB2ENR)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI pins for SLAVE mode
    // PA11 (SPI_CE/NSS): GPIO_INPUT (master controls CS)
    pinMode(SPI_CE, GPIO_INPUT);
    
    // PB3 (SPI_SCK): GPIO_ALT with AF5 (input in slave mode)
    pinMode(SPI_SCK, GPIO_ALT);
    
    // PB4 (SPI_MISO): GPIO_ALT with AF5 (output in slave mode)
    pinMode(SPI_MISO, GPIO_ALT);
    
    // PB5 (SPI_MOSI): GPIO_ALT with AF5 (input in slave mode)
    pinMode(SPI_MOSI, GPIO_ALT);

    // Set output speed type to high for MISO (slave output)
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4);

    // Clear AFR bits before setting (per datasheet best practice)
    // PB3 (SCK): Clear bits 12-15, then set AF5 (0b0101)
    GPIOB->AFR[0] &= ~(0xF << (4 * 3));  // Clear AFRL bits for PB3
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL3, 5);  // Set AF5 for PB3 (SCK)
    
    // PB4 (MISO): Clear bits 16-19, then set AF5
    GPIOB->AFR[0] &= ~(0xF << (4 * 4));  // Clear AFRL bits for PB4
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL4, 5);  // Set AF5 for PB4 (MISO)
    
    // PB5 (MOSI): Clear bits 20-23, then set AF5
    GPIOB->AFR[0] &= ~(0xF << (4 * 5));  // Clear AFRL bits for PB5
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL5, 5);  // Set AF5 for PB5 (MOSI)
    
    // Step 2: Configure SPI_CR1 register (Section 40.4.7, step 2)
    // CRITICAL: Configure with SPE=0 (SPI disabled during configuration)
    SPI1->CR1 &= ~(SPI_CR1_SPE);  // Disable SPI during configuration
    
    // Clear all bits first, then set only what we need
    SPI1->CR1 = 0;  // Reset CR1 to ensure clean state
    
    // 1. MSTR = 0: SLAVE MODE (bit 2 MUST be cleared - critical!)
    //    (Already 0 from reset, but explicitly ensure it's cleared)
    SPI1->CR1 &= ~(SPI_CR1_MSTR);
    
    // 2. CPOL = 0: Clock idle low (matches Arduino Mode 0)
    // 3. CPHA = 0: First edge capture (matches Arduino Mode 0)
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPHA, cpha);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPOL, cpol);
    
    // 4. LSBFIRST = 0: MSB first (matches Arduino MSBFIRST)
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
    
    // 5. SSM = 1: Software slave management (bit 9) - allows manual CS control
    // 6. SSI = 1: Internal slave select high (bit 8) - slave always selected internally
    SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);
    
    // 7. BR[2:0] = 000: Not used in slave mode (slave follows master's clock), clear bits 5:3
    SPI1->CR1 &= ~(SPI_CR1_BR);
    
    // 8. Clear all master-mode and unused bits
    SPI1->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_CRCEN | 
                   SPI_CR1_CRCNEXT | SPI_CR1_CRCL | SPI_CR1_RXONLY);
    
    // Step 3: Configure SPI_CR2 register (Section 40.4.7, step 3)
    // Clear CR2 first
    SPI1->CR2 = 0;
    
    // 1. DS[3:0] = 0b0111: 8-bit data size (bits 15:12)
    SPI1->CR2 |= _VAL2FLD(SPI_CR2_DS, 0b0111);
    
    // 2. FRXTH = 1: RXNE when 1 byte in FIFO (bit 12)
    SPI1->CR2 |= (SPI_CR2_FRXTH);
    
    // 3. SSOE = 0: CRITICAL - Slave select output disabled (bit 2)
    //    SLAVES DO NOT CONTROL NSS - ensure this bit is cleared
    SPI1->CR2 &= ~(SPI_CR2_SSOE);
    
    // 4. Clear all master-mode and unused bits
    SPI1->CR2 &= ~(SPI_CR2_NSSP | SPI_CR2_LDMARX | SPI_CR2_LDMATX | 
                   SPI_CR2_FRF | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    
    // Step 4: Enable SPI Peripheral (Section 40.4.7, step 4)
    // Set SPE = 1 (bit 6 in CR1) to enable SPI
    // This must be done AFTER all configuration is complete
    SPI1->CR1 |= (SPI_CR1_SPE);
    
    // Step 5: Verify SLAVE MODE configuration
    // CRITICAL: Verify MSTR=0 after enabling (ensure we're still in slave mode)
    uint32_t cr1_after_enable = SPI1->CR1;
    uint8_t mstr_after = (cr1_after_enable & SPI_CR1_MSTR) ? 1 : 0;
    
    if (mstr_after != 0) {
        debug_print("[SPI] ERROR: MSTR bit is set! MCU is in MASTER mode - this is WRONG!\r\n");
        debug_print("[SPI] Forcing MSTR=0 to ensure slave mode...\r\n");
        SPI1->CR1 &= ~(SPI_CR1_SPE);  // Disable SPI
        SPI1->CR1 &= ~(SPI_CR1_MSTR); // Clear MSTR
        SPI1->CR1 |= (SPI_CR1_SPE);   // Re-enable SPI
    }
    
    // Debug: SPI initialization complete with register verification
    debug_printf("[SPI] SPI initialized as SLAVE: Mode %d (CPOL=%d, CPHA=%d)\r\n", 
                 (cpol << 1) | cpha, cpol, cpha);
    debug_printf("[SPI] CR1 register: 0x%08X\r\n", SPI1->CR1);
    debug_printf("[SPI] CR2 register: 0x%08X\r\n", SPI1->CR2);
    debug_printf("[SPI] SLAVE MODE VERIFICATION:\r\n");
    debug_printf("[SPI]   MSTR=%d (MUST be 0 for slave) %s\r\n", 
                 (SPI1->CR1 & SPI_CR1_MSTR) ? 1 : 0,
                 ((SPI1->CR1 & SPI_CR1_MSTR) == 0) ? "✓" : "✗ ERROR!");
    debug_printf("[SPI]   SSM=%d, SSI=%d (software slave management) %s\r\n",
                 (SPI1->CR1 & SPI_CR1_SSM) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SSI) ? 1 : 0,
                 ((SPI1->CR1 & SPI_CR1_SSM) && (SPI1->CR1 & SPI_CR1_SSI)) ? "✓" : "✗");
    debug_printf("[SPI]   SSOE=%d (MUST be 0 for slave) %s\r\n",
                 (SPI1->CR2 & SPI_CR2_SSOE) ? 1 : 0,
                 ((SPI1->CR2 & SPI_CR2_SSOE) == 0) ? "✓" : "✗ ERROR!");
    debug_printf("[SPI]   CPOL=%d, CPHA=%d, LSBFIRST=%d, SPE=%d\r\n", 
                 (SPI1->CR1 & SPI_CR1_CPOL) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_CPHA) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_LSBFIRST) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SPE) ? 1 : 0);
    
    // Verify CS pin (PA11) is configured as input
    debug_printf("[SPI] CS pin (PA11) configured as INPUT (master controls CS)\r\n");
    debug_print("[SPI] SPI SLAVE configuration verified - MCU is SLAVE, Arduino is MASTER\r\n");
}

/* Receives a character (1 byte) over SPI in slave mode.
 * Follows datasheet Section 40.4.9 procedure for slave mode.
 *    -- return: the character received over SPI
 * Note: In slave mode, we receive data when master clocks it in.
 * According to datasheet Section 40.4.8: "The data register of the slave must 
 * already contain data to be sent before starting communication with the master."
 */
char spiReceive(void) {
    uint32_t timeout;
    
    // Step 1: Wait for TXE (transmit buffer empty) - can write to DR
    // Per datasheet Section 40.4.9: TXE flag indicates TXFIFO has space
    timeout = 100000;  // Timeout (~1.25ms at 80MHz)
    while(!(SPI1->SR & SPI_SR_TXE)) {
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] ERROR: TXE timeout in spiReceive()\r\n");
            return 0x00;  // Return error value
        }
    }
    
    // Step 2: Write dummy byte to DR (pre-load for transaction)
    // Per datasheet Section 40.4.8: Slave data register must contain data before master starts
    // In slave mode, writing to DR prepares the slave to send data when master clocks
    *(volatile char *) (&SPI1->DR) = 0x00; // Dummy byte
    
    // Step 3: Wait for RXNE (data received in RXFIFO)
    // Per datasheet Section 40.4.9: RXNE flag indicates data is available
    timeout = 100000;  // Timeout (~1.25ms at 80MHz)
    while(!(SPI1->SR & SPI_SR_RXNE)) {
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] ERROR: RXNE timeout in spiReceive() - master not clocking?\r\n");
            return 0x00;  // Return error value
        }
    }
    
    // Step 4: Read from DR (clears RXNE flag automatically)
    char rec = (volatile char) SPI1->DR;
    
    // Debug: log SPI transaction (can be verbose - comment out if too slow)
    // debug_printf("[SPI] RX: 0x%x\r\n", (uint8_t)rec);
    
    return rec; // Return received character
}

/* Read 16-byte sensor data packet from Arduino/ESP32 via SPI (SLAVE mode)
 * Packet format (Roll, Pitch, Yaw order):
 *   Byte 0:  Header (0xAA)
 *   Bytes 1-2:  Roll (int16_t, MSB first, scaled by 100)
 *   Bytes 3-4:  Pitch (int16_t, MSB first, scaled by 100)
 *   Bytes 5-6:  Yaw (int16_t, MSB first, scaled by 100)
 *   Bytes 7-8:  Gyro X (int16_t, MSB first, scaled by 2000)
 *   Bytes 9-10: Gyro Y (int16_t, MSB first, scaled by 2000)
 *   Bytes 11-12: Gyro Z (int16_t, MSB first, scaled by 2000)
 *   Byte 13: Flags (bit 0 = Euler valid, bit 1 = Gyro valid)
 *   Bytes 14-15: Reserved (0x00)
 * All 16-bit values are MSB-first (big-endian per value)
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
    
    // Small delay to allow synchronization after CS goes low
    volatile int setup_delay = 50;  // ~0.625us at 80MHz
    while(setup_delay-- > 0) __asm("nop");
    
    // Pre-load first dummy byte before master starts clocking
    // Per datasheet Section 40.4.8: Slave data register must contain data before master starts
    if(SPI1->SR & SPI_SR_TXE) {
        *(volatile char *) (&SPI1->DR) = 0x00; // Pre-load first dummy byte
    }
    
    // Read 16 bytes - master will clock them in
    // In slave mode, we write dummy bytes to DR and read received data
    for(i = 0; i < 16; i++) {
        packet[i] = spiReceive();  // Receive byte (master clocks it in)
        // Debug: log each byte received
        debug_printf("[SPI] Byte[%d] = 0x%02X\r\n", i, packet[i]);
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

///////////////////////////////////////////////////////////////////////////////
// Software SPI (Bit-banged) Implementation
// Simple implementation that reads data on MOSI on rising clock edges
///////////////////////////////////////////////////////////////////////////////

/* Initialize pins for software SPI (GPIO mode, not alternate function)
 * Pins are configured as:
 * - CS (PA11): Input with pull-up (active LOW)
 * - SCK (PB3): Input with pull-up (idle LOW, rising edge = data capture)
 * - MOSI (PB5): Input with pull-up (data from master)
 */
void initSoftwareSPI(void) {
    // Enable GPIO clocks
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    
    // Configure CS pin (PA11) as input with pull-up
    pinMode(SPI_CE, GPIO_INPUT);
    GPIOA->PUPDR &= ~(0b11 << (2 * 11));  // Clear pull config for PA11
    GPIOA->PUPDR |= (0b01 << (2 * 11));   // Set pull-up for PA11
    
    // Configure SCK pin (PB3) as input with pull-up
    pinMode(SPI_SCK, GPIO_INPUT);
    GPIOB->PUPDR &= ~(0b11 << (2 * 3));   // Clear pull config for PB3
    GPIOB->PUPDR |= (0b01 << (2 * 3));    // Set pull-up for PB3
    
    // Configure MOSI pin (PB5) as input with pull-up
    pinMode(SPI_MOSI, GPIO_INPUT);
    GPIOB->PUPDR &= ~(0b11 << (2 * 5));   // Clear pull config for PB5
    GPIOB->PUPDR |= (0b01 << (2 * 5));    // Set pull-up for PB5
    
    debug_print("[SW_SPI] Software SPI initialized - CS=PA11, SCK=PB3, MOSI=PB5\r\n");
}

/* Wait for CS to go LOW (master starts transaction)
 * Returns 1 when CS goes low, 0 on timeout
 */
static uint8_t waitForCSLow(uint32_t timeout) {
    while(digitalRead(SPI_CE) == 1) {  // CS is HIGH (idle)
        timeout--;
        if(timeout == 0) {
            return 0;  // Timeout
        }
    }
    return 1;  // CS went LOW
}

/* Wait for clock rising edge (LOW -> HIGH transition)
 * For SPI Mode 0 (CPOL=0, CPHA=0): Clock idle is LOW, data captured on rising edge
 * Returns 1 when rising edge detected, 0 on timeout
 * 
 * Strategy: Poll the clock pin and detect LOW->HIGH transition
 */
static uint8_t waitForClockRisingEdge(uint32_t timeout) {
    // For SPI Mode 0, clock idle is LOW
    // We need to detect a LOW->HIGH transition (rising edge)
    
    // Read initial clock state
    uint8_t clock_state = digitalRead(SPI_SCK);
    uint8_t last_state = clock_state;
    
    // Poll for rising edge: wait for transition from LOW (0) to HIGH (1)
    while(timeout > 0) {
        clock_state = digitalRead(SPI_SCK);
        
        // Detect rising edge: was LOW (0) and now HIGH (1)
        if(last_state == 0 && clock_state == 1) {
            return 1;  // Rising edge detected
        }
        
        last_state = clock_state;
        timeout--;
    }
    
    return 0;  // Timeout - no rising edge detected
}

/* Receive one byte via software SPI
 * Reads 8 bits on rising clock edges (MSB first)
 * For SPI Mode 0: Clock idle LOW, data captured on rising edge
 * Returns the received byte, or 0xFF on error/timeout
 */
uint8_t softwareSPIReceiveByte(void) {
    uint8_t byte = 0;
    // At 100kHz, one bit period = 10us, so 8 bits = 80us
    // At 80MHz, 80us = 6400 cycles. Use 20000 cycles (~250us) for safety margin
    uint32_t timeout = 20000;  // Timeout for each bit (~250us at 80MHz)
    
    // Read 8 bits (MSB first)
    for(int bit = 7; bit >= 0; bit--) {
        // Wait for rising clock edge
        if(!waitForClockRisingEdge(timeout)) {
            debug_printf("[SW_SPI] ERROR: Clock timeout while receiving bit %d\r\n", bit);
            return 0xFF;  // Error
        }
        
        // Small delay to ensure data is stable on MOSI after rising edge
        // The master should have data stable before the rising edge
        volatile int sample_delay = 5;
        while(sample_delay-- > 0) __asm("nop");
        
        // Read MOSI bit on rising edge
        if(digitalRead(SPI_MOSI)) {
            byte |= (1 << bit);  // Set bit
        }
        // else bit is already 0
        
        // Optional: Wait for clock to go LOW (falling edge) to prepare for next bit
        // This ensures we're ready for the next rising edge
        uint32_t fall_timeout = timeout / 4;
        while(digitalRead(SPI_SCK) == 1 && fall_timeout > 0) {
            fall_timeout--;
        }
    }
    
    return byte;
}

/* Read 16-byte packet via software SPI
 * Waits for CS to go LOW, reads 16 bytes, waits for CS to go HIGH
 * Packet format matches hardware SPI version
 */
void softwareSPIReadPacket(uint8_t *packet) {
    uint32_t timeout;
    
    debug_print("[SW_SPI] Waiting for CS low...\r\n");
    
    // Wait for CS to go LOW (master starts transaction)
    timeout = 1000000;  // Timeout (~12.5ms at 80MHz)
    if(!waitForCSLow(timeout)) {
        debug_print("[SW_SPI] TIMEOUT: CS did not go low\r\n");
        // Fill packet with zeros on timeout
        for(int i = 0; i < 16; i++) {
            packet[i] = 0x00;
        }
        return;
    }
    
    debug_print("[SW_SPI] CS low detected - reading 16 bytes\r\n");
    
    // Synchronize with clock: Wait for clock to be LOW (idle state for Mode 0)
    // This ensures we start reading at the beginning of a clock cycle
    uint32_t sync_timeout = 10000;  // ~125us timeout
    while(digitalRead(SPI_SCK) == 1 && sync_timeout > 0) {
        sync_timeout--;
    }
    
    if(sync_timeout == 0) {
        debug_print("[SW_SPI] WARNING: Clock not in idle state (LOW) after CS low\r\n");
    }
    
    // Small delay to allow setup time after CS goes low
    // At 100kHz, the master should start clocking soon after CS goes low
    volatile int setup_delay = 50;  // ~0.625us at 80MHz
    while(setup_delay-- > 0) __asm("nop");
    
    // Read 16 bytes
    // Note: The master will clock all 16 bytes continuously
    for(int i = 0; i < 16; i++) {
        packet[i] = softwareSPIReceiveByte();
        
        // Check if we got an error (0xFF due to timeout)
        if(packet[i] == 0xFF && i > 0) {
            // Might be a real 0xFF or a timeout error
            // Check if CS is still low - if not, master finished early
            if(digitalRead(SPI_CE) == 1) {
                debug_printf("[SW_SPI] WARNING: CS went high early at byte %d - master finished transmission\r\n", i);
                // Fill remaining bytes with 0x00
                for(int j = i; j < 16; j++) {
                    packet[j] = 0x00;
                }
                break;
            }
        }
        
        debug_printf("[SW_SPI] Byte[%d] = 0x%02x\r\n", i, packet[i]);
    }
    
    // Wait for CS to go HIGH (master ends transaction)
    timeout = 1000000;
    while(digitalRead(SPI_CE) == 0) {  // Wait for CS HIGH
        timeout--;
        if(timeout == 0) {
            debug_print("[SW_SPI] WARNING: CS did not go high after transaction\r\n");
            break;
        }
    }
    
    debug_print("[SW_SPI] CS high - transaction complete\r\n");
    debug_print("[SW_SPI] Full packet: ");
    debug_print_bytes(packet, 16);
    debug_newline();
}

