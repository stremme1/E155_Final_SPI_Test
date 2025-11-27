// STM32L432KC_SPI.h
// Header for SPI functions
// Copied from Lab07 and adapted for Lab4 project

#ifndef STM32L4_SPI_H
#define STM32L4_SPI_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h"

// SPI pin definitions
#define SPI_CE PA11
#define SPI_SCK PB3
#define SPI_MOSI PB5
#define SPI_MISO PB4

// Control pins for FPGA communication
#define SPI_LOAD PA5  // Load signal to FPGA
#define SPI_DONE PA6  // Done signal from FPGA

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

/* Enables the SPI peripheral and initializes its clock speed (baud rate), polarity, and phase.
 *    -- br: (0b000 - 0b111). The SPI clk will be the master clock / 2^(BR+1).
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPI(int br, int cpol, int cpha);

/* Transmits a character (1 byte) over SPI and returns the received character.
 *    -- send: the character to send over SPI
 *    -- return: the character received over SPI */
char spiSendReceive(char send);

/* Initialize SPI control pins (LOAD and DONE) */
void initSPIControlPins(void);

/* Read drum command from FPGA via SPI */
uint8_t readDrumCommand(void);

#endif

