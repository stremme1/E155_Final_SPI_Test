// STM32L432KC_SPI.h
// Header for SPI functions
// Modified for SPI Slave mode (Arduino as master)

#ifndef STM32L4_SPI_H
#define STM32L4_SPI_H

#include <stdint.h>
#include "STM32L4xx/Device/Include/stm32l432xx.h"

// SPI pin definitions
#define SPI_CE PA11  // NSS (chip select) pin
#define SPI_SCK PB3  // SPI clock
#define SPI_MOSI PB5 // Master Out Slave In (data from Arduino)
#define SPI_MISO PB4 // Master In Slave Out (not used in this application)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

/* Initialize SPI as SLAVE for receiving data from Arduino
 *    -- br: (0b000 - 0b111). Not used in slave mode (clock comes from master), but kept for compatibility
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1)
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 
void initSPISlave(int br, int cpol, int cpha);

/* Receive a byte over SPI (slave mode)
 *    -- return: the character received over SPI */
char spiReceiveByte(void);

/* Receive a packet from Arduino via SPI (slave mode)
 *    -- packet: buffer to store received data
 *    -- length: number of bytes to receive
 *    -- return: 1 if successful, 0 if timeout or error
 */
uint8_t receiveSPIPacket(uint8_t *packet, uint16_t length);

/* Parse Arduino packet format (24 bytes) into Euler angles and gyroscope data
 * Packet format: [Sync(0xAA)][SensorID][Timestamp(4)][Roll(4)][Pitch(4)][Yaw(4)][GyroX(2)][GyroY(2)][GyroZ(2)]
 * All float values are IEEE 754 single precision (4 bytes)
 * Gyroscope values are int16_t (2 bytes, little-endian)
 *    -- packet: pointer to received packet data
 *    -- roll, pitch, yaw: output Euler angles in degrees (float)
 *    -- gyro_x, gyro_y, gyro_z: output gyroscope data (int16_t)
 *    -- sensor_id: output sensor ID
 *    -- valid: output validity flag (1 if valid, 0 if invalid)
 */
void parseArduinoPacket(const uint8_t *packet,
                       float *roll, float *pitch, float *yaw,
                       int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z,
                       uint8_t *sensor_id, uint8_t *valid);

#endif
