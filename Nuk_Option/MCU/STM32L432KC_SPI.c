// STM32L432KC_SPI.c
// Source code for SPI functions
// Modified for SPI Slave mode (Arduino as master)

#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "debug_print.h"

/* Initialize SPI as SLAVE for receiving data from Arduino
 *    -- br: (0b000 - 0b111). Not used in slave mode (clock comes from master), but kept for compatibility
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1)
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPISlave(int br, int cpol, int cpha) {
    // Turn on GPIOA and GPIOB clock domains (GPIOAEN and GPIOBEN bits in AHB2ENR)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on SPI1 clock domain (SPI1EN bit in APB2ENR)

    // Configure SPI pins as alternate function
    pinMode(SPI_SCK, GPIO_ALT);  // SPI1_SCK (PB3) - input in slave mode
    pinMode(SPI_MISO, GPIO_ALT); // SPI1_MISO (PB4) - output in slave mode
    pinMode(SPI_MOSI, GPIO_ALT); // SPI1_MOSI (PB5) - input in slave mode
    pinMode(SPI_CE, GPIO_ALT);   // NSS pin (PA11) - alternate function for hardware slave select

    // Set to AF05 for SPI alternate functions
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL3, 5); // PB3 SCK
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL4, 5); // PB4 MISO
    GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL5, 5); // PB5 MOSI
    GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRH_AFSEL11, 5); // PA11 NSS
    
    // Configure SPI1 as SLAVE
    SPI1->CR1 &= ~(SPI_CR1_MSTR);  // Clear MSTR bit (slave mode)
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST | SPI_CR1_SSM);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPHA, cpha);
    SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPOL, cpol);
    
    // Configure data size (8 bits)
    SPI1->CR2 |= _VAL2FLD(SPI_CR2_DS, 0b0111); // 8-bit data
    
    // Hardware NSS mode (SSM=0, SSOE=0 for slave)
    SPI1->CR1 &= ~(SPI_CR1_SSM);  // Hardware NSS mode
    SPI1->CR2 &= ~(SPI_CR2_SSOE); // NSS output disabled (slave mode)
    
    // Enable SPI
    SPI1->CR1 |= (SPI_CR1_SPE);
    
    // Debug: SPI slave initialization complete
    debug_printf("[SPI] SPI Slave initialized: Mode %d, NSS=PA11 (hardware)\r\n", (cpol << 1) | cpha);
    debug_printf("[SPI] CR1 register: 0x%08X\r\n", SPI1->CR1);
    debug_printf("[SPI] CR2 register: 0x%08X\r\n", SPI1->CR2);
    debug_printf("[SPI] CPOL=%d, CPHA=%d, MSTR=%d, SPE=%d\r\n", 
                 (SPI1->CR1 & SPI_CR1_CPOL) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_CPHA) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_MSTR) ? 1 : 0,
                 (SPI1->CR1 & SPI_CR1_SPE) ? 1 : 0);
    debug_print("[SPI] SPI Slave configuration verified\r\n");
}

/* Receive a byte over SPI (slave mode)
 *    -- return: the character received over SPI */
char spiReceiveByte(void) {
    // Wait until data has been received (RXNE flag set)
    // The master (Arduino) generates clocks and sends data
    while(!(SPI1->SR & SPI_SR_RXNE));
    char rec = (volatile char) SPI1->DR;
    return rec;
}

/* Receive a packet from Arduino via SPI (slave mode)
 *    -- packet: buffer to store received data
 *    -- length: number of bytes to receive
 *    -- return: 1 if successful, 0 if timeout or error
 */
uint8_t receiveSPIPacket(uint8_t *packet, uint16_t length) {
    uint16_t i;
    uint32_t timeout;
    
    // Wait for NSS to go low (Arduino selects this slave)
    timeout = 1000000; // Timeout counter
    while(digitalRead(SPI_CE) == 1) {  // NSS is high (not selected)
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] Timeout waiting for NSS low\r\n");
            return 0; // Timeout
        }
    }
    
    debug_print("[SPI] NSS low - receiving packet\r\n");
    
    // Receive all bytes
    for(i = 0; i < length; i++) {
        packet[i] = spiReceiveByte();
        // Debug: log each byte received (can be verbose)
        // debug_printf("[SPI] Byte[%d] = 0x%x\r\n", i, packet[i]);
    }
    
    // Wait for NSS to go high (transaction complete)
    timeout = 1000000;
    while(digitalRead(SPI_CE) == 0) {  // NSS is still low
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] Timeout waiting for NSS high\r\n");
            return 0; // Timeout
        }
    }
    
    // Wait for SPI to finish (BSY flag cleared)
    timeout = 1000000;
    while(SPI1->SR & SPI_SR_BSY) {
        timeout--;
        if(timeout == 0) {
            debug_print("[SPI] Timeout waiting for SPI BSY clear\r\n");
            return 0; // Timeout
        }
    }
    
    debug_print("[SPI] Packet received successfully\r\n");
    return 1; // Success
}

/* Parse Arduino packet format (24 bytes) into Euler angles and gyroscope data
 * Packet format: [Sync(0xAA)][SensorID][Timestamp(4)][Roll(4)][Pitch(4)][Yaw(4)][GyroX(2)][GyroY(2)][GyroZ(2)]
 * All float values are IEEE 754 single precision (4 bytes)
 * Gyroscope values are int16_t (2 bytes, little-endian)
 */
void parseArduinoPacket(const uint8_t *packet,
                       float *roll, float *pitch, float *yaw,
                       int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z,
                       uint8_t *sensor_id, uint8_t *valid) {
    // Verify sync byte
    if (packet[0] != 0xAA) {
        debug_printf("[SENSOR] ERROR: Invalid sync byte! Expected 0xAA, got 0x%x\r\n", packet[0]);
        *roll = *pitch = *yaw = 0.0f;
        *gyro_x = *gyro_y = *gyro_z = 0;
        *sensor_id = 0;
        *valid = 0;
        return;
    }
    
    // Extract sensor ID
    *sensor_id = packet[1];
    
    // Extract Euler angles as floats (already in degrees from Arduino)
    // Roll (bytes 6-9)
    *roll = *((float*)(&packet[6]));
    // Pitch (bytes 10-13)
    *pitch = *((float*)(&packet[10]));
    // Yaw (bytes 14-17)
    *yaw = *((float*)(&packet[14]));
    
    // Extract gyroscope data (int16_t, little-endian)
    // Gyro X (bytes 18-19)
    *gyro_x = (int16_t)(packet[18] | (packet[19] << 8));
    // Gyro Y (bytes 20-21)
    *gyro_y = (int16_t)(packet[20] | (packet[21] << 8));
    // Gyro Z (bytes 22-23)
    *gyro_z = (int16_t)(packet[22] | (packet[23] << 8));
    
    // Validate Euler angles (check if values are reasonable)
    // Roll, pitch should be -90 to +90, yaw should be -180 to +180 (or 0-360)
    if (*roll < -180.0f || *roll > 180.0f ||
        *pitch < -90.0f || *pitch > 90.0f ||
        *yaw < -180.0f || *yaw > 360.0f) {
        debug_print("[SENSOR] WARNING: Euler angles out of range\r\n");
        *valid = 0;
    } else {
        *valid = 1;
    }
    
    debug_printf("[SENSOR] Parsed: ID=0x%x, Euler: Roll=%.2f Pitch=%.2f Yaw=%.2f, Gyro: X=%d Y=%d Z=%d\r\n",
                 *sensor_id, *roll, *pitch, *yaw, *gyro_x, *gyro_y, *gyro_z);
}
