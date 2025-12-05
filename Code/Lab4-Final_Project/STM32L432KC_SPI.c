// STM32L432KC_SPI.c
// Source code for SPI functions
// Copied from Lab07 and adapted for Lab4 project

#include "STM32L432KC_SPI.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"

/* Enables the SPI peripheral and initializes its clock speed (baud rate), polarity, and phase.
 *    -- br: (0b000 - 0b111). The SPI clk will be the master clock / 2^(BR+1).
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPI(int br, int cpol, int cpha) {
    // Turn on GPIOA and GPIOB clock domains (GPIOAEN and GPIOBEN bits in AHB2ENR)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on SPI1 clock domain (SPI1EN bit in APB2ENR)

    // Configure SPI pins on GPIOB
    pinModePortB(SPI_SCK, GPIO_ALT); // SPI1_SCK
    pinModePortB(SPI_MISO, GPIO_ALT); // SPI1_MISO
    pinModePortB(SPI_MOSI, GPIO_ALT); // SPI1_MOSI
    pinModePortA(SPI_CE, GPIO_OUTPUT); // Manual CS

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
}

/* Transmits a character (1 byte) over SPI and returns the received character.
 *    -- send: the character to send over SPI
 *    -- return: the character received over SPI */
char spiSendReceive(char send) {
    while(!(SPI1->SR & SPI_SR_TXE)); // Wait until the transmit buffer is empty
    *(volatile char *) (&SPI1->DR) = send; // Transmit the character over SPI
    while(!(SPI1->SR & SPI_SR_RXNE)); // Wait until data has been received
    char rec = (volatile char) SPI1->DR;
    return rec; // Return received character
}

/* Initialize SPI control pins (LOAD and DONE) */
void initSPIControlPins(void) {
    // Configure LOAD pin (PA5) as output, initially low
    pinModePortA(SPI_LOAD, GPIO_OUTPUT);
    digitalWritePortA(SPI_LOAD, GPIO_LOW);
    
    // Configure DONE pin (PA6) as input
    pinModePortA(SPI_DONE, GPIO_INPUT);
    
    // Configure CE pin (PA11) as output, initially high
    pinModePortA(SPI_CE, GPIO_OUTPUT);
    digitalWritePortA(SPI_CE, GPIO_HIGH);
}

/* Read drum command from FPGA via SPI
 * Following Lab07 pattern: blocking wait for DONE, then read, then acknowledge
 */
uint8_t readDrumCommand(void) {
    uint8_t command = 0x00;
    
    // Wait for DONE signal to be asserted by FPGA (blocking wait like Lab07)
    // This naturally handles clock domain crossing - MCU will read multiple times
    // until FPGA's DONE signal is stable
    while(!digitalReadPortA(SPI_DONE)) {
        // Wait - FPGA will assert DONE when command is ready
    }
    
    // FPGA has data ready, read command byte
    digitalWritePortA(SPI_CE, GPIO_LOW);  // Lower CE to start transaction
    
    // Read command byte from FPGA (send dummy, receive command)
    command = (uint8_t)spiSendReceive(0x00);
    
    // Wait for SPI transaction to complete (like Lab07 line 129)
    while(SPI1->SR & SPI_SR_BSY);  // Confirm all SPI transactions are completed
    
    digitalWritePortA(SPI_CE, GPIO_HIGH);  // Raise CE to end transaction
    
    // Acknowledge by toggling LOAD (following Lab07 pattern)
    // Lab07 sets LOAD high before sending, then low after sending
    // For our case: set LOAD high to acknowledge, then low
    // No delays needed - FPGA samples on its clock edge, blocking wait ensures stability
    digitalWritePortA(SPI_LOAD, GPIO_HIGH);
    digitalWritePortA(SPI_LOAD, GPIO_LOW);
    
    return command;
}

